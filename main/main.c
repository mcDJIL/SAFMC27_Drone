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
#error "Atur CONFIG_FREERTOS_HZ=1000 (sama seperti persyaratan project Main Control)"
#endif

static const char *TAG = "DRONE";

typedef enum {
    NODE_MAIN   = 1,
    NODE_PANTRY,
    NODE_ICE,
    NODE_OVEN,
    NODE_DRONE
} node_id_t;

typedef enum {
    DRONE_1 = 1,
    DRONE_2 = 2,
    DRONE_3 = 3
} drone_id_t;

#define DRONE_UNIT_ID  DRONE_1
#define PKT_LEN        3
_Static_assert(PKT_LEN == 3, "Paket ring harus tetap 3 byte");
_Static_assert(DRONE_UNIT_ID >= 1 && DRONE_UNIT_ID <= 3, "ID unit drone harus 1 sampai 3");

static const uint8_t s_node_id = NODE_DRONE;
static const uint8_t s_drone_unit_id = DRONE_UNIT_ID;

#define MAIN_STATUS_TIMEOUT_MS 250
#define PRS_STATUS_TIMEOUT_MS   250

#define PIN_LORA_MOSI      6
#define PIN_LORA_MISO      5
#define PIN_LORA_SCK       4
#define PIN_LORA_NSS       7
#define PIN_LORA_RST       2
#define PIN_LORA_DIO0      8
#define LORA_FREQ_HZ       433000000

enum {
    CMD_NONE     = 0x0,
    CMD_MAIN1    = 0x1,
    CMD_MAIN2    = 0x2,
    CMD_OPT1     = 0x3,
    CMD_OPT2     = 0x4,
    CMD_SW_DRONE = 0x5,
    CMD_SW_CAM   = 0x6,
};

#define SERVO_COUNT             6
#define SERVO_PWM_HZ            50
#define SERVO_PWM_BITS          14
#define SERVO_HOLD_US           1100
#define SERVO_RELEASE_US        1900
#define SERVO_DROP_HOLD_MS      350
#define SERVO_TASK_POLL_MS      5

#define PIN_SERVO_1  0
#define PIN_SERVO_2  1
#define PIN_SERVO_3  3
#define PIN_SERVO_4  10
#define PIN_SERVO_5  20
#define PIN_SERVO_6  21

#define DRONE_ENABLE_SERVO_OUTPUT    0
#define DRONE_BENCH_ASSUME_AIRBORNE  0
#define DRONE_BENCH_PRELOAD_SLOT1    0

typedef enum {
    PAYLOAD_NONE = 0,
    PAYLOAD_RED,
    PAYLOAD_GREEN,
    PAYLOAD_BALL,
    PAYLOAD_CUBE,
} payload_type_t;

typedef struct {
    int8_t         gpio;
    payload_type_t payload;
    bool           occupied;
    bool           releasing;
    int64_t        return_at_us;
} servo_slot_t;

static servo_slot_t s_slots[SERVO_COUNT] = {
    { .gpio = PIN_SERVO_1 }, { .gpio = PIN_SERVO_2 },
    { .gpio = PIN_SERVO_3 }, { .gpio = PIN_SERVO_4 },
    { .gpio = PIN_SERVO_5 }, { .gpio = PIN_SERVO_6 },
};

typedef struct {
    uint8_t last_cmd;
    uint8_t latched_cmd;
    uint8_t pantry_land;
    uint8_t ice_land;
    uint8_t oven_land;
    uint32_t rx_main_count;
    uint32_t tx_count;
    uint32_t tx_errors;
    uint32_t dropped_events;
    uint32_t drop_attempts;
    int64_t last_main_rx_us;
    uint8_t active_drone_id;
    bool active;
    bool airborne;
    int64_t last_pantry_rx_us;
    int64_t last_ice_rx_us;
    int64_t last_oven_rx_us;
} drone_state_t;

static drone_state_t s_st = {
    .active_drone_id = DRONE_1,
    .active = (DRONE_UNIT_ID == DRONE_1),
    .airborne = (DRONE_BENCH_ASSUME_AIRBORNE != 0),
};
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t s_cmd_q;
static lora_t *s_radio;

static uint32_t pulse_to_duty(uint16_t pulse_us)
{
    const uint32_t max_duty = (1u << SERVO_PWM_BITS) - 1u;
    return (uint32_t)(((uint64_t)pulse_us * SERVO_PWM_HZ * max_duty + 500000u)
                      / 1000000u);
}

static esp_err_t servo_write_pulse(unsigned i, uint16_t pulse_us)
{
    if (i >= SERVO_COUNT) return ESP_ERR_INVALID_ARG;
    if (!DRONE_ENABLE_SERVO_OUTPUT) return ESP_ERR_INVALID_STATE;
    if (s_slots[i].gpio < 0) return ESP_ERR_INVALID_ARG;
    return analogWrite((uint8_t)s_slots[i].gpio, pulse_to_duty(pulse_us));
}

static void servo_init_all(void)
{
    if (!DRONE_ENABLE_SERVO_OUTPUT) {
        ESP_LOGW(TAG, "Output servo NONAKTIF; atur GPIO dan kalibrasi pulsa terlebih dahulu");
        return;
    }
    ESP_ERROR_CHECK(analogWriteFrequency(SERVO_PWM_HZ));
    ESP_ERROR_CHECK(analogWriteResolution(SERVO_PWM_BITS));

    for (unsigned i = 0; i < SERVO_COUNT; ++i) {
        if (s_slots[i].gpio < 0) {
            ESP_LOGW(TAG, "Servo %u belum memiliki GPIO: dinonaktifkan", i + 1);
            continue;
        }
        esp_err_t err = servo_write_pulse(i, SERVO_HOLD_US);
        if (err != ESP_OK) ESP_LOGE(TAG, "Gagal menginisialisasi servo %u: %s", i + 1, esp_err_to_name(err));
    }
}

static bool slot_matches_cmd(const servo_slot_t *slot, uint8_t cmd)
{
    if (!slot->occupied || slot->releasing) return false;
    if (cmd == CMD_MAIN1) {
        return slot->payload == PAYLOAD_GREEN || slot->payload == PAYLOAD_BALL;
    }
    if (cmd == CMD_MAIN2) {
        return slot->payload == PAYLOAD_RED || slot->payload == PAYLOAD_CUBE;
    }
    return false;
}

static void execute_command(uint8_t cmd)
{
    bool active, airborne;
    int64_t last_main_rx_us;
    portENTER_CRITICAL(&s_mux);
    active = s_st.active;
    airborne = s_st.airborne;
    last_main_rx_us = s_st.last_main_rx_us;
    portEXIT_CRITICAL(&s_mux);
    if (!last_main_rx_us ||
        esp_timer_get_time() - last_main_rx_us > (int64_t)MAIN_STATUS_TIMEOUT_MS * 1000) {
        ESP_LOGW(TAG, "CMD %u diabaikan: paket Main Control sudah terlalu lama tidak diterima", cmd);
        return;
    }

    if (!active) {
        ESP_LOGI(TAG, "CMD %u diabaikan: drone tidak aktif", cmd);
        return;
    }
    if (!airborne) {
        ESP_LOGI(TAG, "CMD %u tidak dijalankan: drone belum dipastikan berada di udara; picking ditangani PRS", cmd);
        return;
    }
    if (cmd != CMD_MAIN1 && cmd != CMD_MAIN2) {
        /* CMD_OPT1 / CMD_OPT2 tidak boleh dianggap sebagai DROP ALL saat di udara. */
        ESP_LOGI(TAG, "CMD %u tidak memiliki aksi dropping saat di udara", cmd);
        return;
    }

    int slot = -1;
    for (unsigned i = 0; i < SERVO_COUNT; ++i) {
        if (slot_matches_cmd(&s_slots[i], cmd)) {
            slot = (int)i;
            break;
        }
    }
    if (slot < 0) {
        ESP_LOGW(TAG, "Tidak ditemukan slot terisi untuk CMD %u; servo tidak digerakkan", cmd);
        return;
    }
    if (!DRONE_ENABLE_SERVO_OUTPUT || s_slots[slot].gpio < 0) {
        ESP_LOGW(TAG, "CMD %u menuju slot %d, tetapi output servo belum dikonfigurasi", cmd, slot + 1);
        return;
    }
    if (s_slots[slot].releasing) return;

    esp_err_t err = servo_write_pulse((unsigned)slot, SERVO_RELEASE_US);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gagal menggerakkan servo %d: %s", slot + 1, esp_err_to_name(err));
        return;
    }

    s_slots[slot].releasing = true;
    s_slots[slot].return_at_us = esp_timer_get_time() + (int64_t)SERVO_DROP_HOLD_MS * 1000;
    portENTER_CRITICAL(&s_mux);
    s_st.drop_attempts++;
    portEXIT_CRITICAL(&s_mux);
    ESP_LOGI(TAG, "Percobaan drop CMD=%u menggunakan slot %d (hasil fisik tidak terdeteksi)", cmd, slot + 1);
}

static void servo_service_returns(void)
{
    const int64_t now = esp_timer_get_time();
    for (unsigned i = 0; i < SERVO_COUNT; ++i) {
        servo_slot_t *s = &s_slots[i];
        if (!s->releasing || now < s->return_at_us) continue;

        esp_err_t err = servo_write_pulse(i, SERVO_HOLD_US);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Servo %u gagal kembali ke posisi awal: %s", i + 1, esp_err_to_name(err));
            s->return_at_us = now + 200000;
            continue;
        }
        s->releasing = false;
        s->occupied = false;
        s->payload = PAYLOAD_NONE;
        ESP_LOGI(TAG, "Slot %u ditandai KOSONG dalam perangkat lunak (belum diverifikasi sensor)", i + 1);
    }
}

static void servo_task(void *arg)
{
    (void)arg;
    uint8_t cmd;
    for (;;) {
        if (xQueueReceive(s_cmd_q, &cmd, pdMS_TO_TICKS(SERVO_TASK_POLL_MS)) == pdTRUE) {
            execute_command(cmd);
        }
        servo_service_returns();
    }
}

static esp_err_t radio_init(void)
{
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
    return lora_begin(&cfg, &s_radio);
}

static bool drone_tx_allowed(void)
{
    bool allowed;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_mux);
    allowed = s_st.active &&
              s_st.active_drone_id == s_drone_unit_id &&
              s_st.last_main_rx_us > 0 &&
              (now - s_st.last_main_rx_us) <= (int64_t)MAIN_STATUS_TIMEOUT_MS * 1000;
    portEXIT_CRITICAL(&s_mux);
    return allowed;
}

static void reply_after_oven(void)
{
    if (!drone_tx_allowed()) return;

    const uint8_t pkt[PKT_LEN] = {
        s_node_id,
        (uint8_t)(s_drone_unit_id << 4),
        0x00
    };
    esp_err_t err = lora_send(s_radio, pkt, PKT_LEN, 0);
    portENTER_CRITICAL(&s_mux);
    if (err == ESP_OK) s_st.tx_count++;
    else               s_st.tx_errors++;
    portEXIT_CRITICAL(&s_mux);
    if (err != ESP_OK) ESP_LOGE(TAG, "Gagal mengirim balasan Node 5: %s", esp_err_to_name(err));
}

static uint8_t next_drone_id(uint8_t current_id)
{
    return (current_id >= DRONE_3) ? DRONE_1 : (uint8_t)(current_id + 1);
}

static void handle_main_packet(uint8_t cmd)
{
    bool enqueue = false;
    bool switched = false;
    bool cam_switch = false;
    bool active_now;
    uint8_t selected_id;

    portENTER_CRITICAL(&s_mux);
    s_st.last_cmd = cmd;
    s_st.rx_main_count++;
    s_st.last_main_rx_us = esp_timer_get_time();

    if (cmd == CMD_NONE) {
        s_st.latched_cmd = CMD_NONE;
    } else if (cmd != s_st.latched_cmd) {
        s_st.latched_cmd = cmd;
        if (cmd == CMD_SW_DRONE) {
            s_st.active_drone_id = next_drone_id(s_st.active_drone_id);
            s_st.active = (s_st.active_drone_id == s_drone_unit_id);
            switched = true;
        } else if (cmd == CMD_SW_CAM) {
            cam_switch = true;
        } else if (cmd == CMD_MAIN1 || cmd == CMD_MAIN2) {
            enqueue = s_st.active;
        }
    }
    selected_id = s_st.active_drone_id;
    active_now = s_st.active;
    portEXIT_CRITICAL(&s_mux);

    if (switched) {
        xQueueReset(s_cmd_q);
        ESP_LOGI(TAG, "Pergantian drone: sekarang Drone %u; unit ini %s",
                 (unsigned)selected_id, active_now ? "AKTIF" : "TIDAK AKTIF");
    }
    if (cam_switch) {
        ESP_LOGI(TAG, "Perintah pergantian kamera diterima; aksi kamera melalui PPM remote");
    }
    if (enqueue && xQueueSend(s_cmd_q, &cmd, 0) != pdTRUE) {
        portENTER_CRITICAL(&s_mux);
        s_st.dropped_events++;
        portEXIT_CRITICAL(&s_mux);
        ESP_LOGW(TAG, "Antrean perintah penuh; CMD %u tidak dijalankan", cmd);
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

    switch (node) {
    case NODE_MAIN:
        handle_main_packet((uint8_t)(b1 >> 4));
        break;
    case NODE_PANTRY:
        portENTER_CRITICAL(&s_mux);
        s_st.pantry_land = (uint8_t)(b2 >> 4);
        s_st.last_pantry_rx_us = esp_timer_get_time();
        portEXIT_CRITICAL(&s_mux);
        break;
    case NODE_ICE:
        portENTER_CRITICAL(&s_mux);
        s_st.ice_land = (uint8_t)(b1 & 0x0F);
        s_st.last_ice_rx_us = esp_timer_get_time();
        portEXIT_CRITICAL(&s_mux);
        break;
    case NODE_OVEN:
        portENTER_CRITICAL(&s_mux);
        s_st.oven_land = (uint8_t)(b1 & 0x0F);
        s_st.last_oven_rx_us = esp_timer_get_time();
        portEXIT_CRITICAL(&s_mux);
        reply_after_oven();
        break;
    default:
        break;
    }
}

static void stats_task(void *arg)
{
    (void)arg;
    for (;;) {
        drone_state_t s;
        portENTER_CRITICAL(&s_mux);
        s = s_st;
        portEXIT_CRITICAL(&s_mux);
        const int64_t now = esp_timer_get_time();
        bool connected = s.last_main_rx_us &&
                         (now - s.last_main_rx_us < (int64_t)MAIN_STATUS_TIMEOUT_MS * 1000);
        bool pantry_fresh = s.last_pantry_rx_us &&
                            (now - s.last_pantry_rx_us < (int64_t)PRS_STATUS_TIMEOUT_MS * 1000);
        bool ice_fresh = s.last_ice_rx_us &&
                         (now - s.last_ice_rx_us < (int64_t)PRS_STATUS_TIMEOUT_MS * 1000);
        bool oven_fresh = s.last_oven_rx_us &&
                          (now - s.last_oven_rx_us < (int64_t)PRS_STATUS_TIMEOUT_MS * 1000);
        ESP_LOGI(TAG,
                 "unit=%u node=%u aktif=%u koneksi=%s CMD=%u N1=%lu TX_N5=%lu gagal=%lu drop=%lu",
                 (unsigned)s_drone_unit_id, (unsigned)s_node_id,
                 (unsigned)s.active_drone_id, connected ? "TERHUBUNG" : "TERPUTUS",
                 (unsigned)s.last_cmd, (unsigned long)s.rx_main_count,
                 (unsigned long)s.tx_count, (unsigned long)s.tx_errors,
                 (unsigned long)s.drop_attempts);
        ESP_LOGI(TAG, "Limit switch PRS: Pantry=%s Ice=%s Oven=%s (tidak menunjukkan ID drone)",
                 pantry_fresh ? (s.pantry_land ? "AKTIF" : "TIDAK AKTIF") : "TIDAK DIKETAHUI",
                 ice_fresh ? (s.ice_land ? "AKTIF" : "TIDAK AKTIF") : "TIDAK DIKETAHUI",
                 oven_fresh ? (s.oven_land ? "AKTIF" : "TIDAK AKTIF") : "TIDAK DIKETAHUI");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    s_cmd_q = xQueueCreate(8, sizeof(uint8_t));
    configASSERT(s_cmd_q);

#if DRONE_BENCH_PRELOAD_SLOT1
    s_slots[0].payload = PAYLOAD_GREEN;
    s_slots[0].occupied = true;
#endif

    servo_init_all();
    ESP_ERROR_CHECK(radio_init());
    ESP_ERROR_CHECK(lora_start_receive(s_radio, on_rx, NULL));

    ESP_LOGI(TAG, "Drone awal dipilih: Drone 1; CMD_SW_DRONE mengubah pilihan berurutan 1-2-3-1");
    ESP_LOGI(TAG, "Paket Node 1 berisi CMD saja; ID drone aktif dikirim oleh Node 5");
    ESP_LOGW(TAG, "Pastikan pergantian lewat LoRa dan PPM remote berjalan sinkron");
    ESP_LOGI(TAG, "CAKSA DRONE unit %u: node ring %u, ukuran paket tetap %u byte",
             (unsigned)s_drone_unit_id, (unsigned)s_node_id, (unsigned)PKT_LEN);

    BaseType_t servo_ok = xTaskCreate(servo_task, "drone_servo", 3072, NULL, 5, NULL);
    BaseType_t stats_ok = xTaskCreate(stats_task, "drone_stats", 3072, NULL, 2, NULL);
    configASSERT(servo_ok == pdPASS && stats_ok == pdPASS);
}

