/*
 * SPDX-FileCopyrightText: 2026 PT6314 ESP-IDF Component contributors
 * SPDX-License-Identifier: MIT
 */

/**
 * @file pt6314.h
 * @brief Native ESP-IDF driver for the Princeton PT6314 character VFD controller.
 *
 * The component implements the PT6314 three-wire serial write interface with
 * ESP-IDF GPIO APIs. It contains no Arduino compatibility code, performs no
 * dynamic allocation, and serializes public operations with a static FreeRTOS
 * mutex. Data bytes supplied to the write APIs are written unmodified to the
 * current PT6314 DDRAM or CGRAM address. Character codes stored in DDRAM then
 * select CGROM or CGRAM glyphs; the driver performs no Unicode or code-page
 * conversion.
 *
 * @note The component exposes one process-wide controller instance and has no
 * deinitialization API. After successful initialization, individual public
 * calls are thread-safe, but a sequence such as set-cursor then write-string is
 * not atomic across the two calls; serialize compound UI operations externally.
 * @note The write-only interface has no ACK or readback. ESP_OK means that the
 * local GPIO transaction completed, not that the external display was observed.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration for one PT6314 controller.
 *
 * The native component intentionally supports one controller instance. All
 * state remains private to the component and no dynamic memory is allocated.
 * The 40-column two-line DDRAM range is usable with the external GRID-driver
 * expansion described by the datasheet; one PT6314 provides 24 GRID outputs.
 *
 * @warning With PT6314 VDD1 at 5 V, the datasheet guarantees logic high only
 * above 3.5 V for SI/STB and 4.0 V for SCK, and specifies input rise/fall times
 * below 15 ns. Direct ESP32-C3 3.3 V signaling is outside the worst-case
 * electrical specification even if a sample works in practice. Use a suitable
 * fast 3.3 V-to-5 V logic buffer unless the module already provides one.
 */
typedef struct {
    gpio_num_t clk_pin;   /**< PT6314 SCK pin; data is latched on its rising edge. */
    gpio_num_t data_pin;  /**< PT6314 SI/SO pin, used as output only. */
    gpio_num_t stb_pin;   /**< PT6314 STB pin, active low for one complete frame. */

    uint8_t columns;      /**< Software cursor bound: 1..80 (one line), 1..40 (two lines). */
    uint8_t lines;        /**< Function Set line mode: 1 or 2; panel wiring and DLS must agree. */
    uint8_t brightness;   /**< Initial level: 0..25=25%, 26..50=50%, 51..75=75%, else 100%. */
} pt6314_config_t;

/**
 * @brief Raw PT6314 serial frame type.
 */
typedef enum {
    PT6314_FRAME_COMMAND = 0xF8, /**< Write the Instruction Register (RS=0, R/W=0). */
    PT6314_FRAME_DATA = 0xFA,    /**< Write the Data Register (RS=1, R/W=0). */
} pt6314_frame_type_t;

/**
 * @brief Configure GPIOs and run the datasheet-aligned PT6314 initialization.
 *
 * This API, and all other APIs in this component, must be called from FreeRTOS
 * task context. Calling pt6314_init() again after successful initialization
 * returns ESP_ERR_INVALID_STATE; a failed initialization may be retried. Call
 * it only after the PT6314 logic supply is stable; the component retains the
 * proven 100 ms board-level startup margin.
 * The configuration is copied before this function returns.
 *
 * @param[in] config GPIO, geometry, and initial-brightness configuration.
 *
 * @retval ESP_OK Initialization completed.
 * @retval ESP_ERR_INVALID_ARG The configuration, pin selection, or geometry is invalid.
 * @retval ESP_ERR_INVALID_STATE Called from an ISR or after successful initialization.
 * @retval ESP_ERR_NO_MEM The static FreeRTOS mutex could not be created.
 * @retval ESP_ERR_TIMEOUT The internal mutex could not be acquired.
 * @return An ESP-IDF GPIO error if a hardware operation fails.
 */
esp_err_t pt6314_init(const pt6314_config_t *config);

/**
 * @brief Fill DDRAM with spaces and return the address counter to zero.
 *
 * The PT6314 also resets the display shift to its original position and forces
 * subsequent DDRAM/CGRAM accesses to increment. Display on/off and brightness
 * are retained.
 *
 * @retval ESP_OK Command transmitted.
 * @retval ESP_ERR_INVALID_STATE Driver not initialized or called from an ISR.
 * @retval ESP_ERR_TIMEOUT The internal mutex could not be acquired.
 * @return An ESP-IDF GPIO error if transmission fails.
 */
esp_err_t pt6314_clear(void);

/**
 * @brief Return the address counter and display shift to the home position.
 *
 * Unlike pt6314_clear(), this command does not change DDRAM contents.
 *
 * @retval ESP_OK Command transmitted.
 * @retval ESP_ERR_INVALID_STATE Driver not initialized or called from an ISR.
 * @retval ESP_ERR_TIMEOUT The internal mutex could not be acquired.
 * @return An ESP-IDF GPIO error if transmission fails.
 */
esp_err_t pt6314_home(void);

/**
 * @brief Enable VFD output without changing DDRAM contents.
 *
 * @return ESP_OK on success, or an ESP-IDF driver/state error.
 */
esp_err_t pt6314_display_on(void);

/**
 * @brief Disable VFD output without changing DDRAM contents.
 *
 * @return ESP_OK on success, or an ESP-IDF driver/state error.
 */
esp_err_t pt6314_display_off(void);

/**
 * @brief Select a zero-based visible DDRAM position.
 *
 * Row zero maps to DDRAM 0x00 and row one maps to DDRAM 0x40. Bounds are
 * checked against the geometry supplied to pt6314_init().
 * The driver does not provide automatic wrapping, clipping, or line filling.
 *
 * @param[in] column Zero-based column index.
 * @param[in] row Zero-based row index (0 or 1 according to configuration).
 *
 * @retval ESP_OK Address command transmitted.
 * @retval ESP_ERR_INVALID_ARG The requested position is outside configured bounds.
 * @return An ESP-IDF driver/state error on failure.
 */
esp_err_t pt6314_set_cursor(uint8_t column, uint8_t row);

/**
 * @brief Set brightness using the original Arduino threshold mapping.
 *
 * 0..25 -> 25%, 26..50 -> 50%, 51..75 -> 75%, and 76..255 -> 100%.
 * A value of zero does not turn the display off; use pt6314_display_off().
 *
 * @param[in] brightness Percentage-style input using the mapping above.
 * @return ESP_OK on success, or an ESP-IDF driver/state error.
 */
esp_err_t pt6314_set_brightness(uint8_t brightness);

/**
 * @brief Write one unmodified character code to the current DDRAM/CGRAM address.
 *
 * The address counter advances according to the active Entry Mode. When written
 * to DDRAM, values 0x00..0x07 select the eight CGRAM glyphs and 0x08..0x0F are
 * aliases because character-code bit 3 is ignored. Codes from 0x10 upward
 * select glyphs from the PT6314 mask-ROM variant.
 *
 * @param[in] value Raw 8-bit character or CGRAM data byte.
 * @return ESP_OK on success, or an ESP-IDF driver/state error.
 */
esp_err_t pt6314_write_byte(uint8_t value);

/**
 * @brief Write unmodified bytes while holding the driver lock for the sequence.
 *
 * A zero length is valid; @p data may be NULL only when @p length is zero.
 * Transmission stops at the first GPIO error, so a failed call may have
 * written a prefix of the buffer. The driver does not stop at the configured
 * visible line width; subsequent hardware addresses follow the active Entry Mode.
 *
 * @param[in] data Source bytes.
 * @param[in] length Number of bytes to transmit.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for an invalid buffer, or an
 *         ESP-IDF driver/state error.
 */
esp_err_t pt6314_write_bytes(const uint8_t *data, size_t length);

/**
 * @brief Write a NUL-terminated byte string without encoding conversion.
 *
 * This function is intended for single-byte character codes. UTF-8 multibyte
 * sequences are sent literally and are not converted to a PT6314 font code.
 * Transmission stops at the first NUL or GPIO error. Use pt6314_write_bytes()
 * for data containing an embedded 0x00 byte.
 *
 * @param[in] text NUL-terminated byte string.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if @p text is NULL, or an
 *         ESP-IDF driver/state error.
 */
esp_err_t pt6314_write_string(const char *text);

/**
 * @brief Write eight rows to one of the eight 5x8 CGRAM character slots.
 *
 * The location is masked with 0x07, matching the Arduino implementation. The
 * PT6314 uses data bits 4..0 for each row and ignores bits 7..5; bytes are sent
 * unmodified for compatibility. Slots are displayed with character codes
 * 0x00..0x07; 0x08..0x0F are aliases because bit 3 is ignored. The current
 * DDRAM address is not restored.
 *
 * @param[in] location Slot number; only the low three bits are used.
 * @param[in] charmap Eight row bytes. Bit 4 is the leftmost visible dot.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if @p charmap is NULL, or an
 *         ESP-IDF driver/state error.
 */
esp_err_t pt6314_create_char(uint8_t location, const uint8_t charmap[8]);

/**
 * @brief Send one raw 16-bit PT6314 frame.
 *
 * Recognized Function Set, Display Control, Entry Mode, and Clear commands
 * update the corresponding cached state after a successful frame. A raw
 * Function Set that changes the line mode can still disagree with the geometry
 * supplied to pt6314_init(), so prefer the dedicated APIs when one is available.
 * The value itself is intentionally not validated against reserved instructions.
 *
 * @param[in] frame_type PT6314_FRAME_COMMAND or PT6314_FRAME_DATA.
 * @param[in] value Raw instruction or data byte.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for an unknown frame type, or
 *         an ESP-IDF driver/state error.
 */
esp_err_t pt6314_send_raw(pt6314_frame_type_t frame_type, uint8_t value);

#ifdef __cplusplus
}
#endif
