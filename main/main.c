#include <stdbool.h>
#include <stdint.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "easy_gpio.h"
#include "lora.h"

#if CONFIG_FREERTOS_HZ < 1000
#error "Atur CONFIG_FREERTOS_HZ=1000 untuk firmware CAKSA"
#endif

typedef enum {
    NODE_MAIN   = 1,
    NODE_PANTRY = 2,
    NODE_ICE    = 3,
    NODE_OVEN   = 4,
    NODE_DRONE  = 5
} node_id_t;

static const uint8_t s_node_id = NODE_DRONE;

#define DRONE_UNIT_ID             1
#define PKT_LEN                   3
#define SLOT_COUNT                6

#define PIN_LORA_SCK              4
#define PIN_LORA_MISO             5
#define PIN_LORA_MOSI             6
#define PIN_LORA_NSS              7
#define PIN_LORA_RST              2
#define PIN_LORA_DIO0             8

#define PIN_SERVO_1               0
#define PIN_SERVO_2               1
#define PIN_SERVO_3               3
#define PIN_SERVO_4               10
#define PIN_SERVO_5               20
#define PIN_SERVO_6               21

#define DRONE_ACTUATOR_MODE       0
#define DRONE_HW_ARMED            0
#define DRONE_SERVO_CALIBRATED    0
#define SERVO_HOLD_US             1100
#define SERVO_RELEASE_US          1900
#define SERVO_DROP_HOLD_MS        350
#define SERVO_PWM_HZ              50
#define SERVO_PWM_BITS            14

#define MAIN_FRESH_MS             250
#define PANTRY_FRESH_MS           250
#define STATUS_INTERVAL_MS        30000
#define TX_ERROR_LOG_MS          10000
#define SERVO_TASK_POLL_MS        5
#define LORA_FREQ_HZ              433000000UL
#define LORA_TX_POWER_MW          9

#define CMD_NONE                  0
#define CMD_MAIN1                 1
#define CMD_MAIN2                 2
#define CMD_SW_DRONE              5
#define CMD_SW_CAM                6

#define PAYLOAD_EMPTY             0
#define PAYLOAD_RED               1
#define PAYLOAD_GREEN             2
#define PAYLOAD_BALL              3
#define PAYLOAD_CUBE              4
#define PAYLOAD_FAMILY_PANTRY     1
#define PAYLOAD_FAMILY_ICE_OVEN   2

#if DRONE_ACTUATOR_MODE && (!DRONE_HW_ARMED || !DRONE_SERVO_CALIBRATED)
#error "Aktuator Drone butuh persetujuan arming dan kalibrasi servo"
#endif

#if DRONE_ACTUATOR_MODE && !defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
#error "GPIO20/21 servo membutuhkan konsol USB Serial JTAG dan UART0 nonaktif"
#endif

_Static_assert(PKT_LEN == 3, "Panjang paket LoRa harus 3 byte");
_Static_assert(DRONE_UNIT_ID >= 1 && DRONE_UNIT_ID <= 3, "ID Drone tidak valid");
_Static_assert(SLOT_COUNT == 6, "Protokol trial memakai enam slot");

static const char *TAG = "DRONE";
static const int s_servo_pins[SLOT_COUNT] = {
    PIN_SERVO_1, PIN_SERVO_2, PIN_SERVO_3,
    PIN_SERVO_4, PIN_SERVO_5, PIN_SERVO_6
};

static lora_t *s_radio;
static QueueHandle_t s_drop_queue;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t s_slots[SLOT_COUNT];
static bool s_releasing[SLOT_COUNT];
static int64_t s_servo_return_us[SLOT_COUNT];
static uint8_t s_latched_cmd;
static uint8_t s_idle_frames;
static uint8_t s_payload_family = PAYLOAD_FAMILY_PANTRY;
static uint8_t s_pending_cmd;
static int64_t s_pending_cmd_us;
static uint16_t s_last_event_token;
static bool s_event_seen;
static bool s_has_pantry;
static bool s_docked;
static bool s_ever_docked;
static uint8_t s_stock_red = 6;
static uint8_t s_stock_green = 6;
static int64_t s_last_main_us;
static int64_t s_last_pantry_us;
static uint32_t s_rx_main;
static uint32_t s_tx_drone;
static uint32_t s_tx_errors;
static uint32_t s_tx_error_logged_ms;
static bool s_tx_error_active;
static bool s_tx_error_notified;
static uint32_t s_drops_ok;
static uint32_t s_drops_rejected;

static const char *payload_name(uint8_t payload)
{
    if (payload == PAYLOAD_RED) return "MERAH";
    if (payload == PAYLOAD_GREEN) return "HIJAU";
    if (payload == PAYLOAD_BALL) return "BALL";
    if (payload == PAYLOAD_CUBE) return "CUBE";
    return "KOSONG";
}

static void slots_to_text(char text[SLOT_COUNT + 1])
{
    portENTER_CRITICAL(&s_mux);
    for (unsigned i = 0; i < SLOT_COUNT; ++i) {
        text[i] = s_slots[i] == PAYLOAD_RED ? 'R' :
                  s_slots[i] == PAYLOAD_GREEN ? 'G' :
                  s_slots[i] == PAYLOAD_BALL ? 'B' :
                  s_slots[i] == PAYLOAD_CUBE ? 'C' : '-';
    }
    portEXIT_CRITICAL(&s_mux);
    text[SLOT_COUNT] = '\0';
}

static bool pack_slots(uint16_t *packed_out)
{
    uint16_t packed = 0;
    bool valid = true;
    portENTER_CRITICAL(&s_mux);
    for (unsigned i = 0; i < SLOT_COUNT; ++i) {
        uint8_t payload = s_slots[i];
        uint8_t value = 0;
        if (payload == PAYLOAD_EMPTY) {
            value = 0;
        } else if (s_payload_family == PAYLOAD_FAMILY_PANTRY) {
            if (payload == PAYLOAD_RED) value = 1;
            else if (payload == PAYLOAD_GREEN) value = 2;
            else valid = false;
        } else if (s_payload_family == PAYLOAD_FAMILY_ICE_OVEN) {
            if (payload == PAYLOAD_BALL) value = 1;
            else if (payload == PAYLOAD_CUBE) value = 2;
            else valid = false;
        } else {
            valid = false;
        }
        packed |= (uint16_t)(value << (10 - 2 * i));
    }
    portEXIT_CRITICAL(&s_mux);
    if (valid) *packed_out = packed;
    return valid;
}

bool drone_select_payload_family(uint8_t family)
{
    if (family != PAYLOAD_FAMILY_PANTRY && family != PAYLOAD_FAMILY_ICE_OVEN) return false;
    bool empty = true;
    portENTER_CRITICAL(&s_mux);
    for (unsigned i = 0; i < SLOT_COUNT; ++i) {
        if (s_slots[i] != PAYLOAD_EMPTY || s_releasing[i]) {
            empty = false;
            break;
        }
    }
    if (empty) s_payload_family = family;
    portEXIT_CRITICAL(&s_mux);
    if (empty) {
        ESP_LOGI(TAG, "Mode payload dipilih: %s",
                 family == PAYLOAD_FAMILY_PANTRY ? "MERAH-HIJAU" : "BALL-CUBE");
    } else {
        ESP_LOGW(TAG, "Pergantian jenis payload ditolak: magazine belum kosong");
    }
    return empty;
}

static uint32_t pulse_to_duty(uint16_t pulse_us)
{
    return (uint32_t)(((uint64_t)pulse_us * ((1UL << SERVO_PWM_BITS) - 1u) + 10000u) / 20000u);
}

static esp_err_t servo_write(unsigned slot, uint16_t pulse_us)
{
    if (slot >= SLOT_COUNT) return ESP_ERR_INVALID_ARG;
    if (!DRONE_ACTUATOR_MODE) return ESP_ERR_INVALID_STATE;
    return analogWrite((uint8_t)s_servo_pins[slot], pulse_to_duty(pulse_us));
}

static esp_err_t servo_init(void)
{
    if (!DRONE_ACTUATOR_MODE) return ESP_OK;
    esp_err_t err = analogWriteFrequency(SERVO_PWM_HZ);
    if (err != ESP_OK) return err;
    err = analogWriteResolution(SERVO_PWM_BITS);
    if (err != ESP_OK) return err;
    for (unsigned i = 0; i < SLOT_COUNT; ++i) {
        err = servo_write(i, SERVO_HOLD_US);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static void handle_pick_event(uint8_t red, uint8_t green, uint8_t event)
{
    if (event == 0) return;
    if (event > 12) {
        ESP_LOGW(TAG, "Event picking PRS tidak valid: %u", (unsigned)event);
        return;
    }
    uint16_t token = (uint16_t)(((uint16_t)red << 8) | ((uint16_t)green << 4) | event);
    unsigned slot = (unsigned)((event - 1u) % SLOT_COUNT);
    uint8_t payload = event <= SLOT_COUNT ? PAYLOAD_GREEN : PAYLOAD_RED;
    bool accepted = false;
    bool conflict = false;
    portENTER_CRITICAL(&s_mux);
    if (!s_event_seen || s_last_event_token != token) {
        if (s_slots[slot] == PAYLOAD_EMPTY && !s_releasing[slot]) {
            s_slots[slot] = payload;
            accepted = true;
        } else if (s_slots[slot] != payload || s_releasing[slot]) {
            conflict = true;
        }
        if (!conflict) {
            s_last_event_token = token;
            s_event_seen = true;
        }
    }
    portEXIT_CRITICAL(&s_mux);
    if (accepted) {
        char text[SLOT_COUNT + 1];
        slots_to_text(text);
        ESP_LOGI(TAG, "PICK: slot %u=%s, magazine=%s", slot + 1u, payload_name(payload), text);
    } else if (conflict) {
        ESP_LOGE(TAG, "Konflik sinkronisasi pada slot %u, picking tidak dicatat", slot + 1u);
    }
}

static bool drop_allowed(int64_t now)
{
    bool allowed;
    portENTER_CRITICAL(&s_mux);
    allowed = s_has_pantry && !s_docked && s_ever_docked &&
              s_last_main_us > 0 && s_last_pantry_us > 0 &&
              (now - s_last_main_us) <= (int64_t)MAIN_FRESH_MS * 1000 &&
              (now - s_last_pantry_us) <= (int64_t)PANTRY_FRESH_MS * 1000;
    portEXIT_CRITICAL(&s_mux);
    return allowed;
}

static void start_drop(uint8_t cmd)
{
    int64_t now = esp_timer_get_time();
    uint8_t payload;
    portENTER_CRITICAL(&s_mux);
    payload = cmd == CMD_MAIN1 ?
              (s_payload_family == PAYLOAD_FAMILY_PANTRY ? PAYLOAD_GREEN : PAYLOAD_BALL) :
              (s_payload_family == PAYLOAD_FAMILY_PANTRY ? PAYLOAD_RED : PAYLOAD_CUBE);
    portEXIT_CRITICAL(&s_mux);
    if (!drop_allowed(now)) {
        s_drops_rejected++;
        ESP_LOGW(TAG, "DROP ONE %s ditolak: dock, status LoRa, atau riwayat landing belum aman", payload_name(payload));
        return;
    }
    int selected = -1;
    portENTER_CRITICAL(&s_mux);
    for (unsigned i = 0; i < SLOT_COUNT; ++i) {
        if (s_slots[i] == payload && !s_releasing[i]) {
            selected = (int)i;
            s_releasing[i] = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_mux);
    if (selected < 0) {
        s_drops_rejected++;
        ESP_LOGW(TAG, "DROP ONE %s ditolak: tidak ada slot yang sesuai", payload_name(payload));
        return;
    }
    if (!DRONE_ACTUATOR_MODE) {
        portENTER_CRITICAL(&s_mux);
        s_slots[selected] = PAYLOAD_EMPTY;
        s_releasing[selected] = false;
        portEXIT_CRITICAL(&s_mux);
        s_drops_ok++;
        char slots[SLOT_COUNT + 1];
        slots_to_text(slots);
        ESP_LOGI(TAG, "DROP VIRTUAL: %s dari slot %d, magazine=%s", payload_name(payload), selected + 1, slots);
        return;
    }
    esp_err_t err = servo_write((unsigned)selected, SERVO_RELEASE_US);
    if (err != ESP_OK) {
        portENTER_CRITICAL(&s_mux);
        s_releasing[selected] = false;
        portEXIT_CRITICAL(&s_mux);
        s_drops_rejected++;
        ESP_LOGE(TAG, "Servo slot %d gagal melepas payload: %s", selected + 1, esp_err_to_name(err));
        return;
    }
    s_servo_return_us[selected] = now + (int64_t)SERVO_DROP_HOLD_MS * 1000;
    ESP_LOGI(TAG, "DROP FISIK: %s slot %d, servo bergerak (belum diverifikasi)", payload_name(payload), selected + 1);
}

static void service_servo_returns(void)
{
    if (!DRONE_ACTUATOR_MODE) return;
    int64_t now = esp_timer_get_time();
    for (unsigned i = 0; i < SLOT_COUNT; ++i) {
        if (!s_releasing[i] || now < s_servo_return_us[i]) continue;
        esp_err_t err = servo_write(i, SERVO_HOLD_US);
        if (err != ESP_OK) {
            s_servo_return_us[i] = now + 200000;
            ESP_LOGE(TAG, "Servo slot %u gagal kembali ke posisi tahan: %s", i + 1, esp_err_to_name(err));
            continue;
        }
        portENTER_CRITICAL(&s_mux);
        s_releasing[i] = false;
        s_slots[i] = PAYLOAD_EMPTY;
        portEXIT_CRITICAL(&s_mux);
        s_drops_ok++;
        ESP_LOGI(TAG, "DROP SELESAI: slot %u ditandai kosong (tanpa sensor)", i + 1);
    }
}

static void reply_node5(void)
{
    int64_t now = esp_timer_get_time();
    if (!s_last_main_us || now - s_last_main_us > (int64_t)MAIN_FRESH_MS * 1000) return;
    uint16_t packed = 0;
    if (!pack_slots(&packed)) {
        s_tx_errors++;
        uint32_t now_ms = (uint32_t)(now / 1000);
        if (s_tx_errors == 1 || (uint32_t)(now_ms - s_tx_error_logged_ms) >= TX_ERROR_LOG_MS) {
            ESP_LOGE(TAG, "Telemetri slot tidak cocok dengan protokol 12-bit, total gagal=%lu",
                     (unsigned long)s_tx_errors);
            s_tx_error_logged_ms = now_ms;
            s_tx_error_notified = true;
        }
        s_tx_error_active = true;
        return;
    }
    const uint8_t pkt[PKT_LEN] = {
        s_node_id,
        (uint8_t)((DRONE_UNIT_ID << 4) | ((packed >> 8) & 0x0Fu)),
        (uint8_t)(packed & 0xFFu)
    };
    esp_err_t err = lora_send(s_radio, pkt, PKT_LEN, 0);
    if (err == ESP_OK) {
        s_tx_drone++;
        if (s_tx_error_active && s_tx_error_notified) {
            ESP_LOGI(TAG, "TX Drone pulih, total gagal=%lu", (unsigned long)s_tx_errors);
            s_tx_error_notified = false;
        }
        s_tx_error_active = false;
    } else {
        s_tx_errors++;
        uint32_t now_ms = (uint32_t)(now / 1000);
        if (s_tx_errors == 1 || (uint32_t)(now_ms - s_tx_error_logged_ms) >= TX_ERROR_LOG_MS) {
            ESP_LOGE(TAG, "TX Drone gagal (%s), total=%lu", esp_err_to_name(err),
                     (unsigned long)s_tx_errors);
            s_tx_error_logged_ms = now_ms;
            s_tx_error_notified = true;
        }
        s_tx_error_active = true;
    }
}

static void on_rx(lora_t *radio, const lora_packet_t *p, void *ctx)
{
    (void)radio;
    (void)ctx;
    if (!p->crc_ok || p->len != PKT_LEN) return;
    const uint8_t node = p->data[0];
    const uint8_t b1 = p->data[1];
    const uint8_t b2 = p->data[2];
    const int64_t now = esp_timer_get_time();

    if (node == NODE_MAIN) {
        if ((b1 & 0x0Fu) != 0 || b2 != 0) return;
        uint8_t cmd = (uint8_t)(b1 >> 4);
        s_last_main_us = now;
        s_rx_main++;
        if (cmd == CMD_NONE) {
            if (s_idle_frames < 2) s_idle_frames++;
            if (s_idle_frames >= 2) s_latched_cmd = CMD_NONE;
        } else {
            s_idle_frames = 0;
            if (cmd != s_latched_cmd) {
            s_latched_cmd = cmd;
            if (cmd == CMD_MAIN1 || cmd == CMD_MAIN2) {
                s_pending_cmd = cmd;
                s_pending_cmd_us = now;
            } else if (cmd == CMD_SW_DRONE) {
                ESP_LOGW(TAG, "CMD Switch Drone diabaikan: trial hanya satu Drone");
            } else if (cmd == CMD_SW_CAM) {
                ESP_LOGI(TAG, "CMD Switch Camera diterima; output PPM dikendalikan Main");
            }
            }
        }
        return;
    }
    if (node != NODE_PANTRY) return;
    uint8_t red = (uint8_t)(b1 >> 4);
    uint8_t green = (uint8_t)(b1 & 0x0Fu);
    uint8_t landed = (uint8_t)(b2 >> 4);
    uint8_t event = (uint8_t)(b2 & 0x0Fu);
    if (red > 6 || green > 6 || landed > 1 || event > 12) return;

    bool docked = landed == 1;
    bool changed = !s_has_pantry || docked != s_docked;
    portENTER_CRITICAL(&s_mux);
    s_has_pantry = true;
    s_docked = docked;
    if (docked) s_ever_docked = true;
    s_stock_red = red;
    s_stock_green = green;
    s_last_pantry_us = now;
    portEXIT_CRITICAL(&s_mux);
    if (changed) {
        ESP_LOGI(TAG, "DOCK: %s (laporan Pantry)", docked ? "MENDARAT" : "LEPAS");
    }
    handle_pick_event(red, green, event);

    if (s_pending_cmd == CMD_MAIN1 || s_pending_cmd == CMD_MAIN2) {
        uint8_t cmd = s_pending_cmd;
        s_pending_cmd = CMD_NONE;
        if (now - s_pending_cmd_us <= (int64_t)MAIN_FRESH_MS * 1000 && !docked) {
            if (xQueueSend(s_drop_queue, &cmd, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Antrean DROP ONE penuh, CMD diabaikan");
            }
        }
    }
    reply_node5();
}

static void drop_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint8_t cmd;
        if (xQueueReceive(s_drop_queue, &cmd, pdMS_TO_TICKS(SERVO_TASK_POLL_MS)) == pdTRUE) {
            start_drop(cmd);
        }
        service_servo_returns();
    }
}

static void status_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(STATUS_INTERVAL_MS));
        char slots[SLOT_COUNT + 1];
        slots_to_text(slots);
        uint8_t counts[PAYLOAD_CUBE + 1] = {0};
        portENTER_CRITICAL(&s_mux);
        for (unsigned i = 0; i < SLOT_COUNT; ++i) {
            if (s_slots[i] <= PAYLOAD_CUBE) counts[s_slots[i]]++;
        }
        portEXIT_CRITICAL(&s_mux);
        ESP_LOGI(TAG, "RING: dock=%s slot=%s isi R%u/G%u/B%u/C%u PRS=R%u/G%u RX1=%lu TX5=%lu gagal=%lu drop=%lu tolak=%lu",
                 s_docked ? "YA" : "TIDAK", slots,
                 (unsigned)counts[PAYLOAD_RED], (unsigned)counts[PAYLOAD_GREEN],
                 (unsigned)counts[PAYLOAD_BALL], (unsigned)counts[PAYLOAD_CUBE],
                 (unsigned)s_stock_red, (unsigned)s_stock_green,
                 (unsigned long)s_rx_main, (unsigned long)s_tx_drone,
                 (unsigned long)s_tx_errors, (unsigned long)s_drops_ok,
                 (unsigned long)s_drops_rejected);
    }
}

void app_main(void)
{
    s_drop_queue = xQueueCreate(8, sizeof(uint8_t));
    configASSERT(s_drop_queue);
    ESP_ERROR_CHECK(servo_init());

    lora_config_t cfg = LORA_CONFIG_DEFAULT(PIN_LORA_MOSI, PIN_LORA_MISO,
                                            PIN_LORA_SCK, PIN_LORA_NSS,
                                            PIN_LORA_RST, PIN_LORA_DIO0);
    cfg.frequency_hz = LORA_FREQ_HZ;
    cfg.sf = LORA_SF6;
    cfg.bw = LORA_BW_500K;
    cfg.cr = LORA_CR_4_5;
    cfg.preamble_len = 6;
    cfg.crc = true;
    cfg.header_mode = LORA_HEADER_IMPLICIT;
    cfg.implicit_payload_len = PKT_LEN;
    cfg.ldro = LORA_LDRO_OFF;
    cfg.tx_power_mw = LORA_TX_POWER_MW;

    ESP_ERROR_CHECK(lora_begin(&cfg, &s_radio));
    ESP_ERROR_CHECK(lora_start_receive(s_radio, on_rx, NULL));
    configASSERT(xTaskCreate(drop_task, "drone_drop", 4096, NULL, 5, NULL) == pdPASS);
    configASSERT(xTaskCreate(status_task, "drone_status", 3072, NULL, 2, NULL) == pdPASS);
    ESP_LOGI(TAG, "SIAP: Drone Node 5, ring 1->2->5->1, servo=%s, LoRa=%.0f mW",
             DRONE_ACTUATOR_MODE ? "FISIK" : "VIRTUAL", (double)LORA_TX_POWER_MW);
    ESP_LOGI(TAG, "Mode Pantry merah/hijau; Ball/Cube menunggu PRS Ice/Oven");
}