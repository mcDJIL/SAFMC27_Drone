#include "easy_gpio.h"

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "soc/soc_caps.h"

static const char *TAG = "easy_gpio_irq";

typedef struct {
    easy_isr_t     cb;
    easy_isr_arg_t cb_arg;
    void          *arg;
} isr_entry_t;

static isr_entry_t s_isr[SOC_GPIO_PIN_COUNT];
static bool        s_service_ready;

static void IRAM_ATTR isr_trampoline(void *ctx)
{
    isr_entry_t *e = (isr_entry_t *)ctx;

    if (e->cb) {
        e->cb();
    } else if (e->cb_arg) {
        e->cb_arg(e->arg);
    }
}

static esp_err_t attach_common(uint8_t pin, easy_isr_t cb, easy_isr_arg_t cb_arg,
                               void *arg, uint8_t mode)
{
    const int gpio = (int)pin;
    gpio_int_type_t type;

    if (!GPIO_IS_VALID_GPIO(gpio)) {
        ESP_LOGE(TAG, "GPIO%d tidak valid", gpio);
        return ESP_ERR_INVALID_ARG;
    }
    if (!cb && !cb_arg) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (mode) {
    case RISING:  type = GPIO_INTR_POSEDGE; break;
    case FALLING: type = GPIO_INTR_NEGEDGE; break;
    case CHANGE:  type = GPIO_INTR_ANYEDGE; break;
    default:
        ESP_LOGE(TAG, "Mode interrupt %u tidak dikenal", mode);
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_service_ready) {
        esp_err_t err = gpio_install_isr_service(0);
        /* INVALID_STATE = service sudah dipasang pihak lain, itu tidak masalah */
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            return err;
        }
        s_service_ready = true;
    }

    /* kalau sebelumnya sudah ada handler di pin ini, lepas dulu */
    if (s_isr[gpio].cb || s_isr[gpio].cb_arg) {
        gpio_isr_handler_remove((gpio_num_t)gpio);
    }

    s_isr[gpio].cb     = cb;
    s_isr[gpio].cb_arg = cb_arg;
    s_isr[gpio].arg    = arg;

    esp_err_t err = gpio_set_intr_type((gpio_num_t)gpio, type);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_isr_handler_add((gpio_num_t)gpio, isr_trampoline, &s_isr[gpio]);
    if (err != ESP_OK) {
        return err;
    }
    return gpio_intr_enable((gpio_num_t)gpio);
}

esp_err_t attachInterrupt(uint8_t pin, easy_isr_t callback, uint8_t mode)
{
    return attach_common(pin, callback, NULL, NULL, mode);
}

esp_err_t attachInterruptArg(uint8_t pin, easy_isr_arg_t callback, void *arg, uint8_t mode)
{
    return attach_common(pin, NULL, callback, arg, mode);
}

esp_err_t detachInterrupt(uint8_t pin)
{
    const int gpio = (int)pin;

    if (!GPIO_IS_VALID_GPIO(gpio)) {
        return ESP_ERR_INVALID_ARG;
    }

    gpio_intr_disable((gpio_num_t)gpio);
    gpio_set_intr_type((gpio_num_t)gpio, GPIO_INTR_DISABLE);
    esp_err_t err = gpio_isr_handler_remove((gpio_num_t)gpio);

    s_isr[gpio].cb     = NULL;
    s_isr[gpio].cb_arg = NULL;
    s_isr[gpio].arg    = NULL;
    return err;
}