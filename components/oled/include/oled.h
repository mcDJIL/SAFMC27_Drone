/*
 * oled.h - Layar OLED SSD1306 128x64 lewat I2C.
 *
 * Diambil dari project PPM-esp (components/oled.c) dan dipindahkan ke driver
 * I2C baru ESP-IDF (`driver/i2c_master.h`), karena driver lama `driver/i2c.h`
 * sudah EOL di IDF v6.
 *
 * Kursor memakai satuan piksel: x = kolom (0..127), y = halaman (0..7).
 * Satu karakter 6x8 piksel, jadi satu baris muat 21 karakter.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Inisialisasi I2C + panel. Panggil sekali di app_main. */
void oled_init(void);

/** Bersihkan seluruh layar dan kembalikan kursor ke (0,0). */
void oled_clear(void);

/** Pindahkan kursor. y = halaman (0..7), x = kolom piksel (0..127). */
void oled_set_cursor(uint8_t x, uint8_t y);

/** Cetak teks di posisi kursor. Hanya 0-9 A-Z a-z spasi . : - / % yang ada. */
void oled_print(const char *str);

/**
 * Cetak teks berformat di (x,y) dan sisanya diisi spasi sampai ujung baris,
 * supaya tulisan lama tidak tertinggal saat teks baru lebih pendek.
 */
void oled_printf(uint8_t x, uint8_t y, const char *fmt, ...);

/** Gambar bitmap 1-bit, lebar w piksel, tinggi h piksel (kelipatan 8). */
void oled_draw_bitmap(const uint8_t *bitmap, uint8_t w, uint8_t h);

#ifdef __cplusplus
}
#endif
