// SPDX-License-Identifier: LGPL-2.1-or-later

extern "C" {
#include "bus/mini_i2c.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_log.h>
#include <esp_console.h>
#include <nvs_flash.h>

#include <string.h>
#include <stdio.h>
}

#include "misc/misc.hpp"
#include "misc/battery.hpp"
#include "wifi/http.hpp"
#include "wifi/wifi.hpp"
#include "wifi/cam_control.hpp"
#include "gyro/gyro.hpp"
#include "logger/logger.hpp"
#include "filters/gyro_ring.hpp"
#include "storage/settings.hpp"
#include "storage/storage_fat.hpp"

#include "global_context.hpp"
#include "bus/aux_i2c.hpp"
#include "display.hpp"

static const char *TAG = "main";

static void nvs_init() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

extern void test();

void test_task(void *params) {
    // mini_i2c_set_timing(200000);
    // mini_i2c_double_stop_timing();
    // mini_i2c_double_stop_timing();
    while (1) {
        proc_aux_i2c();
        vTaskDelay(1);
    }
}

void app_main_cpp(void) {
    nvs_init();

    gctx.settings_manager = new SettingsManager();

#if EXPERIMENTAL_BATTERY
    xTaskCreate(battery_task, "battery_task", 3084, NULL, configMAX_PRIORITIES - 4, NULL);
#endif

    wifi_init();
    display_setup();

    ESP_ERROR_CHECK(storage_fat_init());

    gctx.logger_control.mutex = xSemaphoreCreateMutex();
    gctx.logger_control.accel_raw_mtx = xSemaphoreCreateMutex();
    gctx.gyro_ring = new GyroRing();
    gctx.gyro_ring->Init(3072, kBlockSize, 1800);

    xTaskCreate(logger_task, "logger", 3084, NULL, configMAX_PRIORITIES - 3, NULL);

    int sda_pin = gctx.settings_manager->Get("sda_pin");
    int scl_pin = gctx.settings_manager->Get("scl_pin");
    if (sda_pin >= 0 && scl_pin >= 0) {
        ESP_ERROR_CHECK(mini_i2c_init(gctx.settings_manager->Get("sda_pin"),
                                      gctx.settings_manager->Get("scl_pin"), 400000));
        gctx.aux_i2c_queue = xQueueCreate(1, sizeof(aux_i2c_msg_t));
        gyro_probe_and_start_task();
    } else {
        ESP_LOGW(TAG, "Please assign i2c gpio pins!");
    }

    if (gctx.settings_manager->Get("led_type") < 0.5) {
        xTaskCreate(led_task, "led-task", 4096, NULL, configMAX_PRIORITIES - 4, NULL);
    } else {
        xTaskCreate(led_strip_task, "led-task", 4096, NULL, configMAX_PRIORITIES - 4, NULL);
    }
    xTaskCreate(button_task, "button-task", 4096, NULL, configMAX_PRIORITIES - 4, NULL);
    xTaskCreate(cam_control_task, "cam-task", 4096, NULL, configMAX_PRIORITIES - 4, NULL);
    xTaskCreate(display_task, "display-task", 4096, NULL, configMAX_PRIORITIES - 4, NULL);

    http_init();
}

extern "C" {
void app_main(void) { app_main_cpp(); }
}