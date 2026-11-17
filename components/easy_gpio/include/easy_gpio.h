#pragma once

/*
 * easy_gpio - API ala Arduino untuk ESP-IDF (v5.x / v6.x)
 *
 *  Digital   : pinMode, digitalWrite, digitalRead, digitalToggle
 *  Interrupt : attachInterrupt, attachInterruptArg, detachInterrupt
 *  Analog    : analogRead, analogReadMilliVolts, analogSetAttenuation
 *  PWM       : analogWrite, analogWriteFrequency, analogWriteResolution,
 *              analogWriteStop
 *  Waktu     : delay, delayMicroseconds, millis, micros
 *  Utilitas  : pulseIn, shiftIn, shiftOut, map, constrain
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- Digital */

#define LOW   0
#define HIGH  1

#define INPUT              0
#define OUTPUT             1
#define INPUT_PULLUP       2
#define INPUT_PULLDOWN     3
#define OUTPUT_OPEN_DRAIN  4

/* Atur mode pin. Panggil sebelum pin dipakai. */
esp_err_t pinMode(uint8_t pin, uint8_t mode);

void digitalWrite(uint8_t pin, uint8_t val);
int  digitalRead(uint8_t pin);
void digitalToggle(uint8_t pin);

/* -------------------------------------------------------------- Interrupt */

#define RISING   1
#define FALLING  2
#define CHANGE   3

typedef void (*easy_isr_t)(void);
typedef void (*easy_isr_arg_t)(void *arg);

/*
 * Pasang interrupt pada pin. Panggil pinMode() lebih dulu (INPUT / INPUT_PULLUP).
 * Fungsi callback berjalan di konteks ISR: singkat, tanpa delay(), tanpa printf,
 * dan sebaiknya diberi IRAM_ATTR. Variabel yang dibagi dengan task harus volatile.
 */
esp_err_t attachInterrupt(uint8_t pin, easy_isr_t callback, uint8_t mode);
esp_err_t attachInterruptArg(uint8_t pin, easy_isr_arg_t callback, void *arg, uint8_t mode);
esp_err_t detachInterrupt(uint8_t pin);

/* ----------------------------------------------------------------- Analog */

/* Atenuasi ADC (menentukan rentang tegangan input, kira-kira):
 *   ATTEN_0DB   ~ 0 - 0.95 V    ATTEN_2_5DB ~ 0 - 1.25 V
 *   ATTEN_6DB   ~ 0 - 1.75 V    ATTEN_12DB  ~ 0 - 3.1 V (default)
 */
#define ATTEN_0DB    0
#define ATTEN_2_5DB  1
#define ATTEN_6DB    2
#define ATTEN_12DB   3

/* Atur atenuasi untuk semua pin ADC. Harus dipanggil SEBELUM analogRead pertama. */
esp_err_t analogSetAttenuation(uint8_t atten);

/* Baca nilai mentah ADC (12 bit: 0..4095). Return -1 jika gagal / pin bukan pin ADC. */
int analogRead(uint8_t pin);

/* Baca tegangan terkalibrasi dalam mV. Return -1 jika gagal. */
int analogReadMilliVolts(uint8_t pin);

/* -------------------------------------------------------------------- PWM */

/* Frekuensi default 5000 Hz, resolusi default 8 bit (nilai 0..255).
 * Frekuensi & resolusi berlaku untuk SEMUA pin PWM (satu timer dipakai bersama). */
esp_err_t analogWriteFrequency(uint32_t hz);
esp_err_t analogWriteResolution(uint8_t bits);

/* Keluarkan PWM di pin. value: 0 .. (2^resolusi - 1). */
esp_err_t analogWrite(uint8_t pin, uint32_t value);

/* Hentikan PWM di pin dan lepaskan channel-nya. Setelah ini panggil pinMode() lagi. */
esp_err_t analogWriteStop(uint8_t pin);

/* ------------------------------------------------------------------ Waktu */

void     delay(uint32_t ms);
void     delayMicroseconds(uint32_t us);
uint32_t millis(void);
uint32_t micros(void);

/* -------------------------------------------------------------- Utilitas */

#define LSBFIRST 0
#define MSBFIRST 1

/* Ukur lebar pulsa (mikrodetik) pada pin. Return 0 jika timeout.
 * Timeout default Arduino: 1000000 (1 detik). Memakai busy-wait. */
uint32_t pulseIn(uint8_t pin, uint8_t state, uint32_t timeout_us);

void    shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);

#define constrain(amt, low, high) \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

static inline long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    if (in_max == in_min) {
        return out_min;
    }
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#ifdef __cplusplus
}
#endif