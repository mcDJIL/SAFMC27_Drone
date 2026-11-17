#include "easy_gpio.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "soc/soc_caps.h"

static const char *TAG = "easy_gpio_analog";

/* ===================================================================== ADC */

#define ADC_UNITS  SOC_ADC_PERIPH_NUM
#define ADC_CHANS  SOC_ADC_MAX_CHANNEL_NUM

static adc_oneshot_unit_handle_t s_adc_unit[ADC_UNITS];
static adc_cali_handle_t         s_adc_cali[ADC_UNITS][ADC_CHANS];
static bool                      s_adc_chan_ready[ADC_UNITS][ADC_CHANS];
static adc_atten_t               s_adc_atten = ADC_ATTEN_DB_12;
static bool                      s_adc_locked;   /* true setelah channel pertama dikonfigurasi */

esp_err_t analogSetAttenuation(uint8_t atten)
{
    if (s_adc_locked) {
        ESP_LOGE(TAG, "analogSetAttenuation() harus dipanggil sebelum analogRead() pertama");
        return ESP_ERR_INVALID_STATE;
    }

    switch (atten) {
    case ATTEN_0DB:   s_adc_atten = ADC_ATTEN_DB_0;   break;
    case ATTEN_2_5DB: s_adc_atten = ADC_ATTEN_DB_2_5; break;
    case ATTEN_6DB:   s_adc_atten = ADC_ATTEN_DB_6;   break;
    case ATTEN_12DB:  s_adc_atten = ADC_ATTEN_DB_12;  break;
    default:          return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

/* Cari unit/channel dari pin, lalu siapkan unit & channel kalau belum. */
static esp_err_t adc_prepare(uint8_t pin, adc_unit_t *unit, adc_channel_t *chan)
{
    esp_err_t err = adc_oneshot_io_to_channel((int)pin, unit, chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%u bukan pin ADC", pin);
        return err;
    }

    const int u = (int)*unit - (int)ADC_UNIT_1;
    const int c = (int)*chan;
    if (u < 0 || u >= ADC_UNITS || c < 0 || c >= ADC_CHANS) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!s_adc_unit[u]) {
        adc_oneshot_unit_init_cfg_t ucfg = {
            .unit_id  = *unit,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        err = adc_oneshot_new_unit(&ucfg, &s_adc_unit[u]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Gagal membuat unit ADC: %s", esp_err_to_name(err));
            return err;
        }
    }

    if (!s_adc_chan_ready[u][c]) {
        adc_oneshot_chan_cfg_t ccfg = {
            .atten    = s_adc_atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        err = adc_oneshot_config_channel(s_adc_unit[u], *chan, &ccfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Gagal konfigurasi channel ADC: %s", esp_err_to_name(err));
            return err;
        }
        s_adc_chan_ready[u][c] = true;
    }

    s_adc_locked = true;
    return ESP_OK;
}

int analogRead(uint8_t pin)
{
    adc_unit_t    unit;
    adc_channel_t chan;
    int           raw = 0;

    if (adc_prepare(pin, &unit, &chan) != ESP_OK) {
        return -1;
    }
    if (adc_oneshot_read(s_adc_unit[(int)unit - (int)ADC_UNIT_1], chan, &raw) != ESP_OK) {
        return -1;
    }
    return raw;
}

static adc_cali_handle_t adc_get_cali(adc_unit_t unit, adc_channel_t chan)
{
    const int u = (int)unit - (int)ADC_UNIT_1;
    const int c = (int)chan;

    if (s_adc_cali[u][c]) {
        return s_adc_cali[u][c];
    }

    adc_cali_handle_t handle = NULL;
    esp_err_t err = ESP_ERR_NOT_SUPPORTED;

#if defined(ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED)
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id  = unit,
        .chan     = chan,
        .atten    = s_adc_atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_curve_fitting(&cfg, &handle);
#elif defined(ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED)
    adc_cali_line_fitting_config_t cfg = {
        .unit_id  = unit,
        .atten    = s_adc_atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_line_fitting(&cfg, &handle);
#endif

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Kalibrasi ADC tidak tersedia: %s", esp_err_to_name(err));
        return NULL;
    }

    s_adc_cali[u][c] = handle;
    return handle;
}

int analogReadMilliVolts(uint8_t pin)
{
    adc_unit_t    unit;
    adc_channel_t chan;
    int           raw = 0;
    int           mv  = 0;

    if (adc_prepare(pin, &unit, &chan) != ESP_OK) {
        return -1;
    }
    if (adc_oneshot_read(s_adc_unit[(int)unit - (int)ADC_UNIT_1], chan, &raw) != ESP_OK) {
        return -1;
    }

    adc_cali_handle_t cali = adc_get_cali(unit, chan);
    if (!cali) {
        return -1;
    }
    if (adc_cali_raw_to_voltage(cali, raw, &mv) != ESP_OK) {
        return -1;
    }
    return mv;
}

/* ===================================================================== PWM */

#define PWM_MODE      LEDC_LOW_SPEED_MODE
#define PWM_TIMER     LEDC_TIMER_0
#define PWM_CHANNELS  SOC_LEDC_CHANNEL_NUM

typedef struct {
    int      pin;    /* -1 = channel bebas */
    uint32_t duty;
} pwm_slot_t;

static pwm_slot_t s_pwm[PWM_CHANNELS];
static bool       s_pwm_slots_ready;
static bool       s_timer_ready;
static uint32_t   s_pwm_freq = 5000;
static uint8_t    s_pwm_bits = 8;

static void pwm_init_slots(void)
{
    if (!s_pwm_slots_ready) {
        for (int i = 0; i < PWM_CHANNELS; i++) {
            s_pwm[i].pin  = -1;
            s_pwm[i].duty = 0;
        }
        s_pwm_slots_ready = true;
    }
}

static esp_err_t pwm_apply_timer(void)
{
    ledc_timer_config_t t = {
        .speed_mode      = PWM_MODE,
        .duty_resolution = (ledc_timer_bit_t)s_pwm_bits,
        .timer_num       = PWM_TIMER,
        .freq_hz         = s_pwm_freq,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&t);
    if (err == ESP_OK) {
        s_timer_ready = true;
    }
    return err;
}

static void pwm_reapply_duty(void)
{
    for (int i = 0; i < PWM_CHANNELS; i++) {
        if (s_pwm[i].pin >= 0) {
            ledc_set_duty(PWM_MODE, (ledc_channel_t)i, s_pwm[i].duty);
            ledc_update_duty(PWM_MODE, (ledc_channel_t)i);
        }
    }
}

esp_err_t analogWriteFrequency(uint32_t hz)
{
    if (hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint32_t old = s_pwm_freq;
    s_pwm_freq = hz;

    if (s_timer_ready) {
        esp_err_t err = pwm_apply_timer();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Frekuensi %lu Hz tidak valid untuk resolusi %u bit",
                     (unsigned long)hz, s_pwm_bits);
            s_pwm_freq = old;
            pwm_apply_timer();
            return err;
        }
        pwm_reapply_duty();
    }
    return ESP_OK;
}

esp_err_t analogWriteResolution(uint8_t bits)
{
    if (bits < 1 || bits >= LEDC_TIMER_BIT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t  old_bits = s_pwm_bits;
    const uint64_t old_max  = (1ULL << old_bits) - 1;
    const uint64_t new_max  = (1ULL << bits) - 1;

    s_pwm_bits = bits;

    if (s_timer_ready) {
        esp_err_t err = pwm_apply_timer();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Resolusi %u bit tidak valid untuk frekuensi %lu Hz",
                     bits, (unsigned long)s_pwm_freq);
            s_pwm_bits = old_bits;
            pwm_apply_timer();
            return err;
        }
        /* skala ulang duty supaya rasio (persen) tetap sama */
        for (int i = 0; i < PWM_CHANNELS; i++) {
            if (s_pwm[i].pin >= 0) {
                s_pwm[i].duty = (uint32_t)((uint64_t)s_pwm[i].duty * new_max / old_max);
            }
        }
        pwm_reapply_duty();
    }
    return ESP_OK;
}

esp_err_t analogWrite(uint8_t pin, uint32_t value)
{
    const int gpio = (int)pin;
    esp_err_t err;

    if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio)) {
        ESP_LOGE(TAG, "GPIO%d tidak bisa dipakai sebagai output PWM", gpio);
        return ESP_ERR_INVALID_ARG;
    }

    pwm_init_slots();

    if (!s_timer_ready) {
        err = pwm_apply_timer();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Gagal konfigurasi timer LEDC: %s", esp_err_to_name(err));
            return err;
        }
    }

    const uint32_t max = (uint32_t)((1ULL << s_pwm_bits) - 1);
    if (value > max) {
        value = max;
    }

    int slot = -1;
    for (int i = 0; i < PWM_CHANNELS; i++) {
        if (s_pwm[i].pin == gpio) {
            slot = i;
            break;
        }
    }

    if (slot >= 0) {
        s_pwm[slot].duty = value;
        err = ledc_set_duty(PWM_MODE, (ledc_channel_t)slot, value);
        if (err != ESP_OK) {
            return err;
        }
        return ledc_update_duty(PWM_MODE, (ledc_channel_t)slot);
    }

    /* pin baru: ambil channel bebas */
    for (int i = 0; i < PWM_CHANNELS; i++) {
        if (s_pwm[i].pin < 0) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        ESP_LOGE(TAG, "Semua %d channel LEDC sudah terpakai", PWM_CHANNELS);
        return ESP_ERR_NO_MEM;
    }

    ledc_channel_config_t ch = {
        .gpio_num   = gpio,
        .speed_mode = PWM_MODE,
        .channel    = (ledc_channel_t)slot,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = PWM_TIMER,
        .duty       = value,
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ch);
    if (err != ESP_OK) {
        return err;
    }

    s_pwm[slot].pin  = gpio;
    s_pwm[slot].duty = value;
    return ESP_OK;
}

esp_err_t analogWriteStop(uint8_t pin)
{
    const int gpio = (int)pin;

    pwm_init_slots();

    for (int i = 0; i < PWM_CHANNELS; i++) {
        if (s_pwm[i].pin == gpio) {
            ledc_stop(PWM_MODE, (ledc_channel_t)i, 0);
            s_pwm[i].pin  = -1;
            s_pwm[i].duty = 0;
            gpio_reset_pin((gpio_num_t)gpio);
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}