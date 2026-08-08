# SPDX-FileCopyrightText: 2026 PT6314 ESP-IDF Component contributors
# SPDX-License-Identifier: MIT

# Native ESP-IDF component registration. Public consumers only need pt6314.h;
# timing, FreeRTOS locking, and ESP-ROM delay dependencies remain private.
idf_component_register(
    SRCS "pt6314.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio
    PRIV_REQUIRES esp_rom esp_timer freertos
)
