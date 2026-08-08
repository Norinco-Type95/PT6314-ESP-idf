/*
 * SPDX-FileCopyrightText: 2026 PT6314 ESP-IDF Component contributors
 * SPDX-License-Identifier: MIT
 */

/**
 * @file pt6314.c
 * @brief ESP-IDF GPIO implementation of the PT6314 serial write protocol.
 *
 * The physical frame and command encodings follow PT6314 datasheet V1.5. This
 * implementation intentionally remains write-only because the PT6314 Busy Flag
 * is specified as always zero and the current application does not read DDRAM
 * or CGRAM. SCK idles high, bytes are shifted MSB first, and STB stays low for
 * a 0xF8/0xFA Start Byte plus one payload byte. Public calls are task-context-only
 * and serialized by a statically allocated mutex; no heap allocation is used.
 */
#include "pt6314.h"

#include <stdbool.h>

#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#if (configSUPPORT_STATIC_ALLOCATION != 1)
#error "The PT6314 component requires FreeRTOS static allocation support"
#endif

#define PT6314_CMD_CLEAR_DISPLAY       UINT8_C(0x01)
#define PT6314_CMD_RETURN_HOME         UINT8_C(0x02)
#define PT6314_CMD_ENTRY_MODE_SET      UINT8_C(0x04)
#define PT6314_CMD_DISPLAY_CONTROL     UINT8_C(0x08)
#define PT6314_CMD_FUNCTION_SET        UINT8_C(0x20)
#define PT6314_CMD_SET_CGRAM_ADDRESS   UINT8_C(0x40)
#define PT6314_CMD_SET_DDRAM_ADDRESS   UINT8_C(0x80)

#define PT6314_ENTRY_INCREMENT         UINT8_C(0x02)
#define PT6314_ENTRY_DISPLAY_SHIFT     UINT8_C(0x01)
#define PT6314_DISPLAY_ON              UINT8_C(0x04)
#define PT6314_FUNCTION_8_BIT          UINT8_C(0x10)
#define PT6314_FUNCTION_2_LINE         UINT8_C(0x08)

#define PT6314_BRIGHTNESS_100          UINT8_C(0x00)
#define PT6314_BRIGHTNESS_75           UINT8_C(0x01)
#define PT6314_BRIGHTNESS_50           UINT8_C(0x02)
#define PT6314_BRIGHTNESS_25           UINT8_C(0x03)
#define PT6314_BRIGHTNESS_MASK         UINT8_C(0x03)

/*
 * PT6314 V1.5 section 11.3 specifies a 500 ns minimum SCK period,
 * 200 ns high/low pulses, 100 ns data setup/hold, 100 ns from STB falling
 * to the first SCK falling edge, 500 ns from the final SCK rising edge to
 * STB rising, and a 500 ns minimum STB-high interval. Two microseconds per
 * software phase preserves generous margin without legacy conservative
 * command-level delays. Combined paths provide longer effective setup/hold.
 */
#define PT6314_DATA_PREPARE_US         UINT32_C(2)
#define PT6314_CLOCK_LOW_US            UINT32_C(2)
#define PT6314_CLOCK_HIGH_HOLD_US      UINT32_C(2)
#define PT6314_STB_TO_CLOCK_US         UINT32_C(2)
#define PT6314_START_DATA_GAP_US       UINT32_C(2)
#define PT6314_CLOCK_TO_STB_US         UINT32_C(2)
#define PT6314_STB_HIGH_US             UINT32_C(2)

#define PT6314_POWER_STABILIZE_MS      UINT32_C(100)

#define PT6314_MAX_COLUMNS_ONE_LINE    UINT8_C(80)
#define PT6314_MAX_COLUMNS_TWO_LINES   UINT8_C(40)
#define PT6314_CGRAM_SLOT_MASK         UINT8_C(0x07)
#define PT6314_CGRAM_BYTES_PER_SLOT    8U

typedef struct {
    pt6314_config_t config;       /**< Private copy of the caller's geometry and pins. */
    uint8_t display_function;     /**< Cached Function Set command, including brightness. */
    uint8_t display_control;      /**< Cached Display Control command. */
    uint8_t display_mode;         /**< Cached Entry Mode command. */
    bool initialized;             /**< True only after the complete startup sequence. */
    StaticSemaphore_t lock_storage; /**< Statically allocated FreeRTOS mutex storage. */
    SemaphoreHandle_t lock;       /**< Serializes frames and compound operations. */
} pt6314_context_t;

static pt6314_context_t s_pt6314;
/* Protects only the one-time publication of the static FreeRTOS mutex. */
static portMUX_TYPE s_pt6314_init_lock = portMUX_INITIALIZER_UNLOCKED;

static esp_err_t pt6314_validate_config(const pt6314_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!GPIO_IS_VALID_OUTPUT_GPIO(config->clk_pin) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->data_pin) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->stb_pin)) {
        return ESP_ERR_INVALID_ARG;
    }

    if ((config->clk_pin == config->data_pin) ||
        (config->clk_pin == config->stb_pin) ||
        (config->data_pin == config->stb_pin)) {
        return ESP_ERR_INVALID_ARG;
    }

    if ((config->lines != 1U) && (config->lines != 2U)) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t maximum_columns = (config->lines == 1U)
                                        ? PT6314_MAX_COLUMNS_ONE_LINE
                                        : PT6314_MAX_COLUMNS_TWO_LINES;
    if ((config->columns == 0U) || (config->columns > maximum_columns)) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t pt6314_require_task_context(void)
{
    return xPortInIsrContext() ? ESP_ERR_INVALID_STATE : ESP_OK;
}

static esp_err_t pt6314_create_lock(void)
{
    /* Publish the one static mutex atomically if init is entered concurrently. */
    portENTER_CRITICAL(&s_pt6314_init_lock);
    if (s_pt6314.lock == NULL) {
        s_pt6314.lock = xSemaphoreCreateMutexStatic(&s_pt6314.lock_storage);
    }
    const bool lock_created = (s_pt6314.lock != NULL);
    portEXIT_CRITICAL(&s_pt6314_init_lock);

    return lock_created ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t pt6314_lock_initialized(void)
{
    esp_err_t error = pt6314_require_task_context();
    if (error != ESP_OK) {
        return error;
    }

    if (!s_pt6314.initialized || (s_pt6314.lock == NULL)) {
        return ESP_ERR_INVALID_STATE;
    }

    /* A mutex, rather than a critical section, preserves FreeRTOS scheduling. */
    if (xSemaphoreTake(s_pt6314.lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static void pt6314_unlock(void)
{
    (void)xSemaphoreGive(s_pt6314.lock);
}

static void pt6314_delay_ms_at_least(uint32_t delay_ms)
{
    /*
     * pdMS_TO_TICKS() may round down (100 Hz tick in the validated project).
     * Sleep for whole ticks, then busy-wait only the remaining microseconds.
     */
    const int64_t deadline_us = esp_timer_get_time() + ((int64_t)delay_ms * 1000);
    const TickType_t delay_ticks = pdMS_TO_TICKS(delay_ms);

    if (delay_ticks > 0) {
        vTaskDelay(delay_ticks);
    }

    const int64_t remaining_us = deadline_us - esp_timer_get_time();
    if (remaining_us > 0) {
        esp_rom_delay_us((uint32_t)remaining_us);
    }
}

static void pt6314_restore_idle_locked(void)
{
    /*
     * Best-effort recovery: terminate a partial frame by raising STB first, then
     * restore idle-high SCK. Ignore recovery errors so the original error wins.
     */
    (void)gpio_set_level(s_pt6314.config.stb_pin, 1U);
    (void)gpio_set_level(s_pt6314.config.clk_pin, 1U);
}

static void pt6314_sync_command_state_locked(uint8_t command)
{
    /*
     * Commit shadow registers only after a complete successful command frame.
     * Address-counter/DDRAM/CGRAM positions are deliberately not shadowed.
     */
    if (command == PT6314_CMD_CLEAR_DISPLAY) {
        /* Clear forces increment and zeroes the current shift; S is retained. */
        s_pt6314.display_mode =
            PT6314_CMD_ENTRY_MODE_SET |
            PT6314_ENTRY_INCREMENT |
            (s_pt6314.display_mode & PT6314_ENTRY_DISPLAY_SHIFT);
    } else if ((command & UINT8_C(0xFC)) == PT6314_CMD_ENTRY_MODE_SET) {
        s_pt6314.display_mode = command;
    } else if ((command & UINT8_C(0xF8)) == PT6314_CMD_DISPLAY_CONTROL) {
        s_pt6314.display_control = command;
    } else if ((command & UINT8_C(0xE0)) == PT6314_CMD_FUNCTION_SET) {
        s_pt6314.display_function = command;
    }
}

static esp_err_t pt6314_send_byte_locked(uint8_t value)
{
    /*
     * Caller holds the mutex, STB is low, and SCK starts high. DATA changes
     * while SCK is high and is held through the low phase and rising sample.
     */
    for (int bit = 7; bit >= 0; --bit) {
        esp_err_t error = gpio_set_level(
            s_pt6314.config.data_pin,
            (uint32_t)((value >> bit) & UINT8_C(0x01)));
        if (error != ESP_OK) {
            return error;
        }
        esp_rom_delay_us(PT6314_DATA_PREPARE_US);

        error = gpio_set_level(s_pt6314.config.clk_pin, 0U);
        if (error != ESP_OK) {
            return error;
        }
        esp_rom_delay_us(PT6314_CLOCK_LOW_US);

        error = gpio_set_level(s_pt6314.config.clk_pin, 1U);
        if (error != ESP_OK) {
            return error;
        }
        /* PT6314 latches SI on this rising edge. */
        esp_rom_delay_us(PT6314_CLOCK_HIGH_HOLD_US);
    }

    return ESP_OK;
}

static esp_err_t pt6314_send_frame_locked(pt6314_frame_type_t frame_type, uint8_t value)
{
    /* STB stays low for one Start Byte plus one Instruction/Data byte. */
    esp_err_t error = gpio_set_level(s_pt6314.config.stb_pin, 0U);
    if (error != ESP_OK) {
        pt6314_restore_idle_locked();
        return error;
    }
    esp_rom_delay_us(PT6314_STB_TO_CLOCK_US);

    error = pt6314_send_byte_locked((uint8_t)frame_type);
    if (error != ESP_OK) {
        pt6314_restore_idle_locked();
        return error;
    }
    /* Extra SCK-high guard between bytes; this is not command processing time. */
    esp_rom_delay_us(PT6314_START_DATA_GAP_US);

    error = pt6314_send_byte_locked(value);
    if (error != ESP_OK) {
        pt6314_restore_idle_locked();
        return error;
    }

    esp_rom_delay_us(PT6314_CLOCK_TO_STB_US);
    error = gpio_set_level(s_pt6314.config.stb_pin, 1U);
    if (error != ESP_OK) {
        pt6314_restore_idle_locked();
        return error;
    }

    /* tWSTB: keep STB high between complete Start+Instruction/Data frames. */
    esp_rom_delay_us(PT6314_STB_HIGH_US);

    if (frame_type == PT6314_FRAME_COMMAND) {
        pt6314_sync_command_state_locked(value);
    }

    return ESP_OK;
}

static uint8_t pt6314_brightness_bits(uint8_t brightness)
{
    /* Function Set BR1:BR0 encodes 00/01/10/11 as 100/75/50/25 percent. */
    if (brightness <= 25U) {
        return PT6314_BRIGHTNESS_25;
    }
    if (brightness <= 50U) {
        return PT6314_BRIGHTNESS_50;
    }
    if (brightness <= 75U) {
        return PT6314_BRIGHTNESS_75;
    }
    return PT6314_BRIGHTNESS_100;
}

esp_err_t pt6314_init(const pt6314_config_t *config)
{
    esp_err_t error = pt6314_require_task_context();
    if (error != ESP_OK) {
        return error;
    }

    error = pt6314_validate_config(config);
    if (error != ESP_OK) {
        return error;
    }

    error = pt6314_create_lock();
    if (error != ESP_OK) {
        return error;
    }

    if (xSemaphoreTake(s_pt6314.lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_pt6314.initialized) {
        pt6314_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    s_pt6314.config = *config;
    s_pt6314.display_function = PT6314_CMD_FUNCTION_SET |
                                 PT6314_FUNCTION_8_BIT |
                                 pt6314_brightness_bits(config->brightness);
    if (config->lines > 1U) {
        s_pt6314.display_function |= PT6314_FUNCTION_2_LINE;
    }
    s_pt6314.display_control = PT6314_CMD_DISPLAY_CONTROL | PT6314_DISPLAY_ON;
    s_pt6314.display_mode = PT6314_CMD_ENTRY_MODE_SET | PT6314_ENTRY_INCREMENT;

    /* All three signals are write-only in the current driver. */
    gpio_config_t io_config = {0};
    io_config.pin_bit_mask = (UINT64_C(1) << config->clk_pin) |
                             (UINT64_C(1) << config->data_pin) |
                             (UINT64_C(1) << config->stb_pin);
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;

    /* Preload the idle levels before enabling output to avoid a false frame. */
    error = gpio_set_level(config->stb_pin, 1U);
    if (error != ESP_OK) {
        goto init_failed;
    }
    error = gpio_set_level(config->clk_pin, 1U);
    if (error != ESP_OK) {
        goto init_failed;
    }
    error = gpio_set_level(config->data_pin, 0U);
    if (error != ESP_OK) {
        goto init_failed;
    }

    error = gpio_config(&io_config);
    if (error != ESP_OK) {
        goto init_failed;
    }

    error = gpio_set_level(s_pt6314.config.stb_pin, 1U);
    if (error != ESP_OK) {
        goto init_failed;
    }
    error = gpio_set_level(s_pt6314.config.clk_pin, 1U);
    if (error != ESP_OK) {
        goto init_failed;
    }
    error = gpio_set_level(s_pt6314.config.data_pin, 0U);
    if (error != ESP_OK) {
        goto init_failed;
    }

    /* Retained board-level power margin; bit and command timing never uses ms waits. */
    pt6314_delay_ms_at_least(PT6314_POWER_STABILIZE_MS);

    /*
     * Function Set is the first instruction as required by section 6.6.
     * Keep the original library's proven three-send initialization margin,
     * but use only the serial-interface guard intervals between sends.
     */
    for (int attempt = 0; attempt < 3; ++attempt) {
        error = pt6314_send_frame_locked(PT6314_FRAME_COMMAND, s_pt6314.display_function);
        if (error != ESP_OK) {
            goto init_failed;
        }
    }

    error = pt6314_send_frame_locked(PT6314_FRAME_COMMAND, PT6314_CMD_CLEAR_DISPLAY);
    if (error != ESP_OK) {
        goto init_failed;
    }
    error = pt6314_send_frame_locked(PT6314_FRAME_COMMAND, s_pt6314.display_control);
    if (error != ESP_OK) {
        goto init_failed;
    }

    error = pt6314_send_frame_locked(PT6314_FRAME_COMMAND, s_pt6314.display_mode);
    if (error != ESP_OK) {
        goto init_failed;
    }

    s_pt6314.initialized = true;
    pt6314_unlock();
    return ESP_OK;

init_failed:
    pt6314_restore_idle_locked();
    s_pt6314.initialized = false;
    pt6314_unlock();
    return error;
}

esp_err_t pt6314_clear(void)
{
    esp_err_t error = pt6314_lock_initialized();
    if (error != ESP_OK) {
        return error;
    }

    error = pt6314_send_frame_locked(PT6314_FRAME_COMMAND, PT6314_CMD_CLEAR_DISPLAY);

    pt6314_unlock();
    return error;
}

esp_err_t pt6314_home(void)
{
    esp_err_t error = pt6314_lock_initialized();
    if (error != ESP_OK) {
        return error;
    }

    error = pt6314_send_frame_locked(PT6314_FRAME_COMMAND, PT6314_CMD_RETURN_HOME);

    pt6314_unlock();
    return error;
}

esp_err_t pt6314_display_on(void)
{
    esp_err_t error = pt6314_lock_initialized();
    if (error != ESP_OK) {
        return error;
    }

    const uint8_t updated_control = s_pt6314.display_control | PT6314_DISPLAY_ON;
    error = pt6314_send_frame_locked(PT6314_FRAME_COMMAND, updated_control);

    pt6314_unlock();
    return error;
}

esp_err_t pt6314_display_off(void)
{
    esp_err_t error = pt6314_lock_initialized();
    if (error != ESP_OK) {
        return error;
    }

    const uint8_t updated_control =
        s_pt6314.display_control & (uint8_t)~PT6314_DISPLAY_ON;
    error = pt6314_send_frame_locked(PT6314_FRAME_COMMAND, updated_control);

    pt6314_unlock();
    return error;
}

esp_err_t pt6314_set_cursor(uint8_t column, uint8_t row)
{
    esp_err_t error = pt6314_lock_initialized();
    if (error != ESP_OK) {
        return error;
    }

    if ((row >= s_pt6314.config.lines) || (column >= s_pt6314.config.columns)) {
        pt6314_unlock();
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t address = (uint8_t)(PT6314_CMD_SET_DDRAM_ADDRESS +
                                      ((row == 0U) ? column : (UINT8_C(0x40) + column)));
    error = pt6314_send_frame_locked(PT6314_FRAME_COMMAND, address);

    pt6314_unlock();
    return error;
}

esp_err_t pt6314_set_brightness(uint8_t brightness)
{
    esp_err_t error = pt6314_lock_initialized();
    if (error != ESP_OK) {
        return error;
    }

    const uint8_t updated_function =
        (uint8_t)((s_pt6314.display_function & (uint8_t)~PT6314_BRIGHTNESS_MASK) |
                  pt6314_brightness_bits(brightness));
    error = pt6314_send_frame_locked(PT6314_FRAME_COMMAND, updated_function);

    pt6314_unlock();
    return error;
}

esp_err_t pt6314_write_byte(uint8_t value)
{
    esp_err_t error = pt6314_lock_initialized();
    if (error != ESP_OK) {
        return error;
    }

    error = pt6314_send_frame_locked(PT6314_FRAME_DATA, value);

    pt6314_unlock();
    return error;
}

esp_err_t pt6314_write_bytes(const uint8_t *data, size_t length)
{
    if ((data == NULL) && (length > 0U)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t error = pt6314_lock_initialized();
    if (error != ESP_OK) {
        return error;
    }

    for (size_t index = 0; index < length; ++index) {
        error = pt6314_send_frame_locked(PT6314_FRAME_DATA, data[index]);
        if (error != ESP_OK) {
            break;
        }
    }

    pt6314_unlock();
    return error;
}

esp_err_t pt6314_write_string(const char *text)
{
    if (text == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t error = pt6314_lock_initialized();
    if (error != ESP_OK) {
        return error;
    }

    while (*text != '\0') {
        error = pt6314_send_frame_locked(PT6314_FRAME_DATA, (uint8_t)(unsigned char)*text);
        if (error != ESP_OK) {
            break;
        }
        ++text;
    }

    pt6314_unlock();
    return error;
}

esp_err_t pt6314_create_char(uint8_t location, const uint8_t charmap[8])
{
    if (charmap == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t error = pt6314_lock_initialized();
    if (error != ESP_OK) {
        return error;
    }

    location &= PT6314_CGRAM_SLOT_MASK;
    const uint8_t address =
        (uint8_t)(PT6314_CMD_SET_CGRAM_ADDRESS | (uint8_t)(location << 3));

    error = pt6314_send_frame_locked(PT6314_FRAME_COMMAND, address);
    for (size_t index = 0;
         (index < PT6314_CGRAM_BYTES_PER_SLOT) && (error == ESP_OK);
         ++index) {
        error = pt6314_send_frame_locked(PT6314_FRAME_DATA, charmap[index]);
    }

    pt6314_unlock();
    return error;
}

esp_err_t pt6314_send_raw(pt6314_frame_type_t frame_type, uint8_t value)
{
    if ((frame_type != PT6314_FRAME_COMMAND) && (frame_type != PT6314_FRAME_DATA)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t error = pt6314_lock_initialized();
    if (error != ESP_OK) {
        return error;
    }

    error = pt6314_send_frame_locked(frame_type, value);

    pt6314_unlock();
    return error;
}
