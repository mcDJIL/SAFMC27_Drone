#include "easy_gpio.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "easy_gpio";

/* ---------------------------------------------------------------- Digital */

esp_err_t pinMode(uint8_t pin, uint8_t mode)
{
    /* int (bukan unsigned) supaya makro IDF tidak memicu -Wtype-limits */
    const int gpio = (int)pin;

    if (!GPIO_IS_VALID_GPIO(gpio)) {
        ESP_LOGE(TAG, "GPIO%d tidak valid", gpio);
        return ESP_ERR_INVALID_ARG;
    }

    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << gpio,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    switch (mode) {
    case INPUT:
        break;
    case INPUT_PULLUP:
        cfg.pull_up_en = GPIO_PULLUP_ENABLE;
        break;
    case INPUT_PULLDOWN:
        cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
        break;
    case OUTPUT:
        /* INPUT_OUTPUT supaya digitalRead() & digitalToggle() bisa dipakai */
        cfg.mode = GPIO_MODE_INPUT_OUTPUT;
        break;
    case OUTPUT_OPEN_DRAIN:
        cfg.mode = GPIO_MODE_INPUT_OUTPUT_OD;
        break;
    default:
        ESP_LOGE(TAG, "Mode %u tidak dikenal", mode);
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg.mode != GPIO_MODE_INPUT && !GPIO_IS_VALID_OUTPUT_GPIO(gpio)) {
        ESP_LOGE(TAG, "GPIO%d hanya input, tidak bisa OUTPUT", gpio);
        return ESP_ERR_INVALID_ARG;
    }

    gpio_reset_pin((gpio_num_t)gpio);
    return gpio_config(&cfg);
}

void digitalWrite(uint8_t pin, uint8_t val)
{
    gpio_set_level((gpio_num_t)pin, val ? 1 : 0);
}

int digitalRead(uint8_t pin)
{
    return gpio_get_level((gpio_num_t)pin);
}

void digitalToggle(uint8_t pin)
{
    gpio_set_level((gpio_num_t)pin, !gpio_get_level((gpio_num_t)pin));
}

/* ------------------------------------------------------------------ Waktu */

void delay(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    if (ms > 0 && ticks == 0) {
        ticks = 1;
    }
    vTaskDelay(ticks);
}

void delayMicroseconds(uint32_t us)
{
    esp_rom_delay_us(us);
}

uint32_t millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

uint32_t micros(void)
{
    return (uint32_t)esp_timer_get_time();
}

/* -------------------------------------------------------------- Utilitas */

uint32_t pulseIn(uint8_t pin, uint8_t state, uint32_t timeout_us)
{
    const int level = state ? 1 : 0;
    int64_t t_wait = esp_timer_get_time();

    /* 1) tunggu pulsa yang sedang berjalan selesai */
    while (gpio_get_level((gpio_num_t)pin) == level) {
        if ((uint64_t)(esp_timer_get_time() - t_wait) > timeout_us) {
            return 0;
        }
    }

    /* 2) tunggu pulsa mulai */
    t_wait = esp_timer_get_time();
    while (gpio_get_level((gpio_num_t)pin) != level) {
        if ((uint64_t)(esp_timer_get_time() - t_wait) > timeout_us) {
            return 0;
        }
    }

    /* 3) ukur lebar pulsa */
    const int64_t t_start = esp_timer_get_time();
    while (gpio_get_level((gpio_num_t)pin) == level) {
        if ((uint64_t)(esp_timer_get_time() - t_start) > timeout_us) {
            return 0;
        }
    }

    return (uint32_t)(esp_timer_get_time() - t_start);
}

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val)
{
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t bit = (bitOrder == LSBFIRST) ? ((val >> i) & 1)
                                             : ((val >> (7 - i)) & 1);
        digitalWrite(dataPin, bit);
        digitalWrite(clockPin, HIGH);
        digitalWrite(clockPin, LOW);
    }
}

uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder)
{
    uint8_t value = 0;

    for (uint8_t i = 0; i < 8; i++) {
        digitalWrite(clockPin, HIGH);
        uint8_t bit = (uint8_t)digitalRead(dataPin);
        if (bitOrder == LSBFIRST) {
            value |= bit << i;
        } else {
            value |= bit << (7 - i);
        }
        digitalWrite(clockPin, LOW);
    }
    return value;
}