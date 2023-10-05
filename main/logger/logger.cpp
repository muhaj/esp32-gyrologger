// SPDX-License-Identifier: LGPL-2.1-or-later

#include "logger.hpp"
extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

#include <esp_log.h>
#include <esp_console.h>
#include <argtable3/argtable3.h>
}

#include <cstring>

#include <fstream>

#include "filters/gyro_ring.hpp"
#include "storage/storage_fat.hpp"

#include "global_context.hpp"

#define TAG "logger"

#include "storage/filenames.hpp"

std::string get_camera_filename();

static esp_err_t set_gyro_filename_from_camera(char *buf) {
    std::string camera_filename = get_camera_filename();

    if(camera_filename.empty()) {
        ESP_LOGE(TAG, "Couldn't get camera filename");
        return ESP_FAIL;
    }

    // Add a prefix or suffix if needed. For example, adding "_gyro" before the file extension.
    std::size_t last_dot = camera_filename.find_last_of(".");
    if(last_dot == std::string::npos) {
        ESP_LOGE(TAG, "Unexpected camera filename format");
        return ESP_FAIL;
    }
    
    std::string gyro_filename = camera_filename.substr(0, last_dot) + "_gyro" + camera_filename.substr(last_dot);
    
    strncpy(buf, gyro_filename.c_str(), sizeof(buf) - 1);  // copy the new filename to the buffer
    buf[sizeof(buf) - 1] = '\0';  // ensure null termination

    return ESP_OK;
}

static esp_err_t delete_oldest() {
    DIR *dp;
    struct dirent *ep;
    dp = opendir("/sdcard");
    std::string file_to_delete{};
    int min_idx = std::numeric_limits<int>::max();
    if (dp != NULL) {
        while ((ep = readdir(dp))) {
            std::string filename = ep->d_name;
            if (!validate_file_name(filename)) {
                unlink(("/sdcard/" + filename).c_str());
                continue;
            }
            int idx = ((filename[1] - 'A') * 26 + (filename[2] - 'A')) * 100000 +
                      std::stoi(filename.substr(3, 5));
            if (idx < min_idx) {
                min_idx = idx;
                file_to_delete = "/sdcard/" + filename;
            }
        }
        (void)closedir(dp);
    } else {
        ESP_LOGE(TAG, "Couldn't open the directory");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Deleting %s", file_to_delete.c_str());
    unlink(file_to_delete.c_str());
    return ESP_OK;
}

extern "C" {
int esp_vfs_fsync(int fd);
}

static char file_name_buf[30];
void logger_task(void *params_pvoid) {
    std::ofstream csv_writer; // Declare outside the loop for efficiency
    TickType_t prev_dump = xTaskGetTickCount();

    for (int i = 0;; ++i) {
        if (gctx.terminate_for_update) {
            ESP_LOGI(TAG, "Terminating for SW update");
            vTaskDelete(nullptr);
        }

        GyroRing::WorkResult work_result = gctx.gyro_ring->Work();
        if (!work_result.quats) {
            vTaskDelay(1);
            continue;
        }

        if (xSemaphoreTake(gctx.logger_control.mutex, portMAX_DELAY)) {
            gctx.logger_control.last_block_time_us = esp_timer_get_time();

            if (gctx.logger_control.active) {
                gctx.logger_control.busy = true;

                if (!gctx.logger_control.file_name) {
                    if (gctx.video_filename.empty()) {
                        find_good_filename(file_name_buf);
                    } else {
                        strncpy(file_name_buf, gctx.video_filename.c_str(), sizeof(file_name_buf) - 1);
                        file_name_buf[sizeof(file_name_buf) - 1] = '\0';  // Null-terminate in case of overflow
                    }

                    gctx.logger_control.file_name = file_name_buf;

                    ESP_LOGI(TAG, "Using filename: %s", file_name_buf);
                }

                // CSV writing logic
                csv_writer.open(std::string("/sdcard/") + gctx.logger_control.file_name, std::ios::app);


                // Write the header row only if it's a new file
                if (csv_writer.tellp() == 0) {
                    csv_writer << "time,gx,gy,gz,ax,ay,az" << std::endl;
                }

                for (auto& quat : std::begin(work_result.quats), std::end(work_result.quats)) {
                    float roll, pitch, yaw;
                    quat.toEulerAngles(roll, pitch, yaw);

                    csv_writer << xTaskGetTickCount() * portTICK_PERIOD_MS << "," << roll << "," << pitch << "," << yaw << "," << work_result.accels[0] << "," << work_result.accels[1] << "," << work_result.accels[2] << std::endl;
                }

                csv_writer.close();
            } else {
                gctx.logger_control.busy = false;
                gctx.logger_control.file_name = NULL;
            }
        }

        auto [free, total] = get_free_space_kb();
        if (free < 90 && free > 0) {
            delete_oldest();
        }
    }
}
