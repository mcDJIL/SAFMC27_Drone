#include "button.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "button";

#define MAX_LISTENERS       CONFIG_BUTTON_MAX_LISTENERS
#define DEBOUNCE_MS         CONFIG_BUTTON_DEBOUNCE_MS
#define DEFAULT_MAX_PRESS   400
#define DEFAULT_GAP_MS      300

typedef enum { KIND_TAP, KIND_HOLD } kind_t;

struct button_listener {
    bool used;
    kind_t kind;
    gpio_num_t pin;
    bool active_high;

    /* debounce */
    bool raw_last;          /* pembacaan mentah terakhir (true = ditekan) */
    bool stable;            /* status stabil setelah debounce (true = ditekan) */
    uint32_t raw_change_ms; /* waktu pembacaan mentah terakhir berubah */

    /* siklus tekan */
    bool armed;             /* true jika penekanan dimulai setelah listener aktif */
    uint32_t press_start_ms;

    /* tap */
    uint8_t target_taps;
    uint8_t taps;
    uint32_t max_press_ms;
    uint32_t gap_ms;
    uint32_t last_release_ms;

    /* hold */
    uint32_t hold_ms;
    button_hold_mode_t hold_mode;
    bool fired;

    button_tap_cb_t tap_cb;
    button_hold_cb_t hold_cb;
    void *user_data;
};

typedef struct {
    kind_t kind;
    gpio_num_t pin;
    uint32_t value;
    button_tap_cb_t tap_cb;
    button_hold_cb_t hold_cb;
    void *user_data;
} event_t;

enum { ST_IDLE = 0, ST_STARTING, ST_READY };

static struct button_listener s_listeners[MAX_LISTENERS];
static SemaphoreHandle_t s_lock;
static volatile int s_state = ST_IDLE;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void push_event(event_t *ev, int *n, const struct button_listener *l, uint32_t value)
{
    ev[*n] = (event_t){
        .kind = l->kind,
        .pin = l->pin,
        .value = value,
        .tap_cb = l->tap_cb,
        .hold_cb = l->hold_cb,
        .user_data = l->user_data,
    };
    (*n)++;
}

/* Dipanggil tiap tick untuk satu listener. Dijalankan saat s_lock dipegang. */
static void process(struct button_listener *l, uint32_t now, event_t *ev, int *n)
{
    /* --- baca + debounce --- */
    int level = gpio_get_level(l->pin);
    bool raw = l->active_high ? (level != 0) : (level == 0);

    if (raw != l->raw_last) {
        l->raw_last = raw;
        l->raw_change_ms = now;
    }

    bool pressed_edge = false;
    bool released_edge = false;
    if (raw != l->stable && (now - l->raw_change_ms) >= DEBOUNCE_MS) {
        l->stable = raw;
        pressed_edge = raw;
        released_edge = !raw;
    }

    /* --- siklus tekan (waktu diambil dari saat sinyal mulai berubah, bukan setelah debounce) --- */
    bool release_valid = false;
    uint32_t press_ms = 0;

    if (pressed_edge) {
        l->armed = true;
        l->press_start_ms = l->raw_change_ms;
        l->fired = false;
    }
    if (released_edge) {
        release_valid = l->armed;
        l->armed = false;
        press_ms = l->raw_change_ms - l->press_start_ms;
    }

    /* --- logika per jenis --- */
    if (l->kind == KIND_TAP) {
        if (release_valid) {
            if (press_ms <= l->max_press_ms) {
                if (l->taps < UINT8_MAX) {
                    l->taps++;
                }
                l->last_release_ms = l->raw_change_ms;
            } else {
                l->taps = 0; /* tekan terlalu lama -> bukan ketukan, batalkan urutan */
            }
        }
        if (!l->stable && l->taps > 0 && (now - l->last_release_ms) >= l->gap_ms) {
            if (l->taps == l->target_taps) {
                push_event(ev, n, l, l->taps);
            }
            l->taps = 0;
        }
    } else { /* KIND_HOLD */
        if (l->hold_mode == BUTTON_HOLD_ON_THRESHOLD) {
            if (l->stable && l->armed && !l->fired &&
                (now - l->press_start_ms) >= l->hold_ms) {
                l->fired = true;
                push_event(ev, n, l, now - l->press_start_ms);
            }
        } else if (release_valid && press_ms >= l->hold_ms) {
            push_event(ev, n, l, press_ms);
        }
    }
}

static void button_task(void *arg)
{
    (void)arg;
    event_t ev[MAX_LISTENERS];

    TickType_t period = pdMS_TO_TICKS(CONFIG_BUTTON_POLL_INTERVAL_MS);
    if (period == 0) {
        period = 1;
    }
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, period);

        int n = 0;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        uint32_t now = now_ms();
        for (int i = 0; i < MAX_LISTENERS; i++) {
            if (s_listeners[i].used) {
                process(&s_listeners[i], now, ev, &n);
            }
        }
        xSemaphoreGive(s_lock);

        /* callback dipanggil di luar lock agar boleh memanggil button_remove() */
        for (int i = 0; i < n; i++) {
            if (ev[i].kind == KIND_TAP) {
                ev[i].tap_cb(ev[i].pin, (uint8_t)ev[i].value, ev[i].user_data);
            } else {
                ev[i].hold_cb(ev[i].pin, ev[i].value, ev[i].user_data);
            }
        }
    }
}

static esp_err_t ensure_started(void)
{
    for (;;) {
        int st;
        portENTER_CRITICAL(&s_mux);
        st = s_state;
        if (st == ST_IDLE) {
            s_state = ST_STARTING;
        }
        portEXIT_CRITICAL(&s_mux);

        if (st == ST_READY) {
            return ESP_OK;
        }
        if (st == ST_STARTING) {
            vTaskDelay(1); /* task lain sedang inisialisasi */
            continue;
        }

        /* st == ST_IDLE: kita yang menginisialisasi */
        s_lock = xSemaphoreCreateMutex();
        if (s_lock != NULL &&
            xTaskCreate(button_task, "button", CONFIG_BUTTON_TASK_STACK_SIZE, NULL,
                        CONFIG_BUTTON_TASK_PRIORITY, NULL) == pdPASS) {
            s_state = ST_READY;
            return ESP_OK;
        }
        if (s_lock != NULL) {
            vSemaphoreDelete(s_lock);
            s_lock = NULL;
        }
        s_state = ST_IDLE;
        return ESP_ERR_NO_MEM;
    }
}

static struct button_listener *claim_slot(void)
{
    for (int i = 0; i < MAX_LISTENERS; i++) {
        if (!s_listeners[i].used) {
            memset(&s_listeners[i], 0, sizeof(s_listeners[i]));
            return &s_listeners[i];
        }
    }
    return NULL;
}

static esp_err_t setup_pin(struct button_listener *l, gpio_num_t pin,
                           bool active_high, bool no_pull)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = (!no_pull && !active_high) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = (!no_pull && active_high) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config GPIO%d gagal", (int)pin);

    l->pin = pin;
    l->active_high = active_high;

    /* Status awal = kondisi pin saat ini, agar tombol yang sedang tertahan
     * saat didaftarkan tidak langsung memicu event. */
    int level = gpio_get_level(pin);
    bool raw = active_high ? (level != 0) : (level == 0);
    l->raw_last = raw;
    l->stable = raw;
    l->raw_change_ms = now_ms();
    l->armed = false;
    return ESP_OK;
}

esp_err_t button_tap_add(const button_tap_config_t *cfg, button_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(cfg && cfg->callback, ESP_ERR_INVALID_ARG, TAG, "cfg/callback NULL");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(cfg->pin), ESP_ERR_INVALID_ARG, TAG, "pin tidak valid");
    ESP_RETURN_ON_FALSE(cfg->tap_count >= 1, ESP_ERR_INVALID_ARG, TAG, "tap_count harus >= 1");
    ESP_RETURN_ON_ERROR(ensure_started(), TAG, "gagal memulai task");

    xSemaphoreTake(s_lock, portMAX_DELAY);
    struct button_listener *l = claim_slot();
    if (l == NULL) {
        xSemaphoreGive(s_lock);
        ESP_LOGE(TAG, "listener penuh (naikkan CONFIG_BUTTON_MAX_LISTENERS)");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = setup_pin(l, cfg->pin, cfg->active_high, cfg->no_internal_pull);
    if (err != ESP_OK) {
        xSemaphoreGive(s_lock);
        return err;
    }
    l->kind = KIND_TAP;
    l->target_taps = cfg->tap_count;
    l->max_press_ms = cfg->max_press_ms ? cfg->max_press_ms : DEFAULT_MAX_PRESS;
    l->gap_ms = cfg->gap_ms ? cfg->gap_ms : DEFAULT_GAP_MS;
    l->tap_cb = cfg->callback;
    l->user_data = cfg->user_data;
    l->used = true; /* terakhir: baru terlihat oleh task setelah semua field siap */
    xSemaphoreGive(s_lock);

    if (out_handle) {
        *out_handle = l;
    }
    return ESP_OK;
}

esp_err_t button_hold_add(const button_hold_config_t *cfg, button_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(cfg && cfg->callback, ESP_ERR_INVALID_ARG, TAG, "cfg/callback NULL");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(cfg->pin), ESP_ERR_INVALID_ARG, TAG, "pin tidak valid");
    ESP_RETURN_ON_FALSE(cfg->hold_ms > 0, ESP_ERR_INVALID_ARG, TAG, "hold_ms harus > 0");
    ESP_RETURN_ON_ERROR(ensure_started(), TAG, "gagal memulai task");

    xSemaphoreTake(s_lock, portMAX_DELAY);
    struct button_listener *l = claim_slot();
    if (l == NULL) {
        xSemaphoreGive(s_lock);
        ESP_LOGE(TAG, "listener penuh (naikkan CONFIG_BUTTON_MAX_LISTENERS)");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = setup_pin(l, cfg->pin, cfg->active_high, cfg->no_internal_pull);
    if (err != ESP_OK) {
        xSemaphoreGive(s_lock);
        return err;
    }
    l->kind = KIND_HOLD;
    l->hold_ms = cfg->hold_ms;
    l->hold_mode = cfg->mode;
    l->hold_cb = cfg->callback;
    l->user_data = cfg->user_data;
    l->used = true;
    xSemaphoreGive(s_lock);

    if (out_handle) {
        *out_handle = l;
    }
    return ESP_OK;
}


button_handle_t button_on_tap(gpio_num_t pin, uint8_t tap_count,
                              button_tap_cb_t cb, void *user_data,
                              bool active_high)
{
    button_tap_config_t cfg = BUTTON_TAP_CONFIG(pin, tap_count, cb);
    cfg.user_data = user_data;
    cfg.active_high = active_high;

    button_handle_t h = NULL;
    return (button_tap_add(&cfg, &h) == ESP_OK) ? h : NULL;
}

button_handle_t button_on_hold(gpio_num_t pin, uint32_t hold_ms,
                               button_hold_cb_t cb, void *user_data,
                               bool active_high)
{
    button_hold_config_t cfg = BUTTON_HOLD_CONFIG(pin, hold_ms, cb);
    cfg.user_data = user_data;
    cfg.active_high = active_high;

    button_handle_t h = NULL;
    return (button_hold_add(&cfg, &h) == ESP_OK) ? h : NULL;
}


esp_err_t button_remove(button_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL && s_state == ST_READY, ESP_ERR_INVALID_ARG, TAG, "handle tidak valid");
    ESP_RETURN_ON_FALSE(handle >= s_listeners && handle < s_listeners + MAX_LISTENERS,
                        ESP_ERR_INVALID_ARG, TAG, "handle di luar jangkauan");

    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool was_used = handle->used;
    handle->used = false;
    xSemaphoreGive(s_lock);

    return was_used ? ESP_OK : ESP_ERR_INVALID_STATE;
}