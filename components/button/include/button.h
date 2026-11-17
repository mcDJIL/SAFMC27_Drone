/*
 * button — library pembaca tombol untuk ESP32 (ESP-IDF 6.x)
 *
 * Dua fungsi utama:
 *   1. Deteksi jumlah ketuk  -> button_on_tap(pin, jumlah_ketuk, callback, user_data)
 *   2. Deteksi lama ketuk    -> button_on_hold(pin, lama_ms, callback, user_data)
 *
 * Satu pin boleh dipakai oleh banyak listener sekaligus
 * (misal: 2 ketukan, 3 ketukan, dan tahan 2 detik di pin yang sama).
 *
 * Semua callback dipanggil dari satu task internal (bukan dari ISR),
 * jadi aman memanggil ESP_LOGx, xQueueSend, dll. Jaga agar callback tetap singkat.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Handle listener (dipakai untuk button_remove). */
typedef struct button_listener *button_handle_t;

/* ------------------------------------------------------------------ */
/* Callback                                                            */
/* ------------------------------------------------------------------ */

/** Dipanggil saat urutan ketukan selesai dan jumlahnya SAMA dengan yang diminta. */
typedef void (*button_tap_cb_t)(gpio_num_t pin, uint8_t tap_count, void *user_data);

/** Dipanggil saat tombol ditahan sesuai lama yang diminta. duration_ms = lama tekan aktual. */
typedef void (*button_hold_cb_t)(gpio_num_t pin, uint32_t duration_ms, void *user_data);

/* ------------------------------------------------------------------ */
/* Fungsi 1: deteksi jumlah ketuk                                      */
/* ------------------------------------------------------------------ */

typedef struct {
    gpio_num_t pin;            /**< Pin tombol */
    uint8_t tap_count;         /**< Jumlah ketukan agar callback aktif (>= 1) */
    button_tap_cb_t callback;  /**< Wajib diisi */
    void *user_data;           /**< Diteruskan ke callback */
    uint32_t max_press_ms;     /**< Tekanan yang lebih lama dari ini bukan "ketukan". 0 = 400 ms */
    uint32_t gap_ms;           /**< Jeda maksimum antar ketukan; lewat dari ini urutan dianggap selesai. 0 = 300 ms */
    bool active_high;          /**< false (default) = tombol ke GND (aktif LOW), true = tombol ke VCC */
    bool no_internal_pull;     /**< true = jangan aktifkan pull-up/pull-down internal */
} button_tap_config_t;

/** Inisialisasi cepat: sisanya memakai nilai default (aktif LOW + pull-up internal). */
#define BUTTON_TAP_CONFIG(_pin, _count, _cb) \
    { .pin = (_pin), .tap_count = (_count), .callback = (_cb) }

/* ------------------------------------------------------------------ */
/* Fungsi 2: deteksi lama ketuk (tahan)                                */
/* ------------------------------------------------------------------ */

typedef enum {
    BUTTON_HOLD_ON_THRESHOLD = 0, /**< Callback langsung saat durasi tercapai (tombol masih ditekan) */
    BUTTON_HOLD_ON_RELEASE,       /**< Callback saat tombol dilepas, jika durasinya >= hold_ms */
} button_hold_mode_t;

typedef struct {
    gpio_num_t pin;            /**< Pin tombol */
    uint32_t hold_ms;          /**< Lama tekan (ms) agar callback aktif (> 0) */
    button_hold_cb_t callback; /**< Wajib diisi */
    void *user_data;           /**< Diteruskan ke callback */
    button_hold_mode_t mode;   /**< Default: BUTTON_HOLD_ON_THRESHOLD */
    bool active_high;          /**< false (default) = aktif LOW, true = aktif HIGH */
    bool no_internal_pull;     /**< true = jangan aktifkan pull internal */
} button_hold_config_t;

#define BUTTON_HOLD_CONFIG(_pin, _ms, _cb) \
    { .pin = (_pin), .hold_ms = (_ms), .callback = (_cb) }

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/**
 * @brief Aktifkan deteksi jumlah ketuk (versi singkat).
 * @return handle, atau NULL jika gagal (alasan ada di log).
 */
button_handle_t button_on_tap(gpio_num_t pin, uint8_t tap_count,
                              button_tap_cb_t cb, void *user_data,
                              bool active_high);

/**
 * @brief Aktifkan deteksi lama tekan (versi singkat, mode ON_THRESHOLD).
 * @return handle, atau NULL jika gagal.
 */
button_handle_t button_on_hold(gpio_num_t pin, uint32_t hold_ms,
                               button_hold_cb_t cb, void *user_data,
                               bool active_high);

/** Versi lengkap dengan struct konfigurasi. out_handle boleh NULL. */
esp_err_t button_tap_add(const button_tap_config_t *cfg, button_handle_t *out_handle);
esp_err_t button_hold_add(const button_hold_config_t *cfg, button_handle_t *out_handle);

/** Hapus listener. Aman dipanggil dari dalam callback. */
esp_err_t button_remove(button_handle_t handle);

#ifdef __cplusplus
}
#endif