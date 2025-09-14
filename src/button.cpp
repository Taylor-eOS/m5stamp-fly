/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#include "button.hpp"
#include "OneButton.h"
#include "led.hpp"
#include "buzzer.h"

#define PIN_INPUT 0

OneButton button(PIN_INPUT, true);

uint8_t is_long_press = 0;
void task_button_update(void *pvParameters);
void LongPressStop(void *oneButton);
void DuringLongPress(void *oneButton);
void Click(void *oneButton);

bool init_button(void) {
    USBSerial.printf("Event called\n");
    button.attachClick(Click, &button);
    button.setLongPressIntervalMs(3000);
    xTaskCreatePinnedToCore(task_button_update, "TaskButtonUpdate", 1024 * 4, NULL, 0, NULL, tskNO_AFFINITY);
    return true;
}

void task_button_update(void *pvParameters) {
    for (;;) {
        button.tick();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void Click(void *oneButton) {
    USBSerial.printf("Button pressed\n");
    buzzer_sound(4000, 800);
    esp_restart();
}

void LongPressStop(void *oneButton) {
    USBSerial.printf("Button released\n");
    is_long_press = 0;
    esp_restart();
}

void DuringLongPress(void *oneButton) {
    USBSerial.printf("Button long press\n");
    if (!is_long_press) {
        is_long_press = 1;
    }
}

