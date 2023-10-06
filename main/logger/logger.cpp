// SPDX-License-Identifier: LGPL-2.1-or-later

#include "logger.hpp"
#include "wifi/cam_control.hpp"
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

extern std::string video_filename;

// Function to get camera timestamp from cam_control.cpp
extern int get_camera_timestamp();

// Global variables from decoding logic
std::vector<uint8_t> input;
bool fail = false;
static int ztime = 0;
static int pos = 0;
static quat::quat prev_quat(quat::base_type{1}, {}, {}, {});

static esp_err_t find_good_filename(char *buf) {
    DIR *dp;
    struct dirent *ep;
    dp = opendir("/spiflash");
    int max_idx = 0;
    if (dp != NULL) {
        while ((ep = readdir(dp))) {
            std::string filename = ep->d_name;
            int idx = filename_to_index(filename);
            if (idx < 0) {
                unlink(("/spiflash/" + filename).c_str());
                continue;
            }
            max_idx = std::max(idx, max_idx);
        }
        (void)closedir(dp);
    } else {
        ESP_LOGE(TAG, "Couldn't open the directory");
        return ESP_FAIL;
    }
    static constexpr char templ[] = "/spiflash/L%c%c%05d.bin";
    int epoch = gctx.settings_manager->Get("file_epoch");
    if (epoch < max_idx / 100000) {
        epoch = max_idx / 100000;
    }

    if (epoch != max_idx / 100000) {
        ESP_LOGW(TAG, "%d != %d", epoch, max_idx / 100000);
        index_to_filename(100000 * epoch + 1, buf);
    } else {
        index_to_filename(100000 * epoch + (max_idx % 100000) + 1, buf);
    }
    return ESP_OK;
}

static esp_err_t delete_oldest() {
    DIR *dp;
    struct dirent *ep;
    dp = opendir("/spiflash");
    std::string file_to_delete{};
    int min_idx = std::numeric_limits<int>::max();
    if (dp != NULL) {
        while ((ep = readdir(dp))) {
            std::string filename = ep->d_name;
            if (!validate_file_name(filename)) {
                unlink(("/spiflash/" + filename).c_str());
                continue;
            }
            int idx = ((filename[1] - 'A') * 26 + (filename[2] - 'A')) * 100000 +
                      std::stoi(filename.substr(3, 5));
            if (idx < min_idx) {
                min_idx = idx;
                file_to_delete = "/spiflash/" + filename;
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
    FILE *f = NULL;

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
                gctx.logger_control.total_samples_written += kBlockSize;
                xSemaphoreGive(gctx.logger_control.mutex);

                if (!gctx.logger_control.file_name) {
                    std::string sdCardFileName = "/sdcard/" + video_filename;  // Use video_filename from cam_control.hpp
                    gctx.logger_control.file_name = sdCardFileName.c_str();
                }

                if (!f) {
                    ESP_LOGI(TAG, "Opening file %s", gctx.logger_control.file_name);
                    f = fopen(gctx.logger_control.file_name, "ab");  // Open file on SD card
                    
                    // Writing headers to the file
                    fprintf(f, "GYROFLOW IMU LOG\n");
                    fprintf(f, "version,1.1\n");
                    fprintf(f, "id,esplog\n");
                    fprintf(f, "camera_timestamp,%d\n", get_camera_timestamp());  // Fetching camera timestamp
                    fprintf(f, "orientation,xyz\n");
                    fprintf(f, "tscale,0.00180\n");
                    fprintf(f, "gscale,0.00053263221\n");
                    fprintf(f, "ascale,0.00048828125\n");
                    fprintf(f, "t,gx,gy,gz,ax,ay,az\n");
                }

                int i = 0;
                for (auto& q : work_result.quats) {
                    int i_lim = std::min(i++ / 55, work_result.accels_len / 3 - 1);
                    quat::vec rv = (q.conj() * prev_quat).axis_angle();
                    prev_quat = q;
                    if (ztime != 0) {
                        double scale = sample_rate * gscale;
                        fprintf(f, "%d,%d,%d,%d,%d,%d,%d\n",
                                ztime,
                                (int)(double(rv.x) * scale),
                                (int)(double(rv.y) * scale),
                                (int)(double(rv.z) * scale),
                                (int)(work_result.accels[0 + 3 * i_lim]),
                                (int)(work_result.accels[1 + 3 * i_lim]),
                                (int)(work_result.accels[2 + 3 * i_lim]));
                    }
                    ztime++;
                }

            } else {
                if (f) {
                    fflush(f);
                    fclose(f);
                    f = NULL;
                }
                gctx.logger_control.busy = false;
                gctx.logger_control.file_name = NULL;
                xSemaphoreGive(gctx.logger_control.mutex);
            }
        }

        static int flush_gate = 0;
        if (f && (++flush_gate) % 20 == 0) {
            if (xSemaphoreTake(gctx.logger_control.mutex, portMAX_DELAY)) {
                gctx.logger_control.total_bytes_written = ftell(f);
                int log_duration_ms =
                    xTaskGetTickCount() * portTICK_PERIOD_MS - gctx.logger_control.log_start_ts_ms;
                gctx.logger_control.avg_logging_rate_bytes_min =
                    gctx.logger_control.total_bytes_written * 1000LL * 60LL / log_duration_ms;
                xSemaphoreGive(gctx.logger_control.mutex);
            }
            esp_vfs_fsync(fileno(f));
        }
        auto [free, total] = get_free_space_kb();
        if (free < 90 && free > 0) {
            delete_oldest();
        }
    }
}

// Function definition to fetch camera timestamp
int get_camera_timestamp() {
    extern int camera_timestamp_value;  // From cam_control.cpp
    return camera_timestamp_value;
}
