#pragma once

#include "compression/lib/fixquat.hpp"
#include "global_context.hpp"

#include "pt_filter.hpp"
#include "storage/settings.hpp"

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <esp_check.h>
#include <nvs_flash.h>
#include <nvs.h>
}

#include <nvs_handle.hpp>

#include <variant>
#include <vector>

static constexpr int loglevel = 0;

static constexpr float kGyroToRads = 1.0 / 32.8 * 3.141592 / 180.0;
static constexpr float kAccelToG = 16.0 / 32767;
struct raw_sample {
    int16_t gx, gy, gz;
    int16_t ax, ay, az;
    int flags;
};

struct sample {
    uint32_t duration_ns;
    std::variant<raw_sample, quat::quat> sample;
};

struct acc_sample {
    int gyro_ref;
    int16_t acc[3];
};

static constexpr int kFlagHaveAccel = 1;
static constexpr char kLogTag[] = "gyro_ring";

class DurationSmoother {
   public:
    DurationSmoother(int thresh) : thresh(thresh) {}
    uint32_t Smooth(uint32_t d) {
        sum += d;
        count += 1;
        if (count > thresh) {
            dur_avg = sum / count;
            sum = 0;
            count = 0;
            gctx.logger_control.avg_sample_interval_ns = dur_avg;
        }
        return dur_avg;
    }

   private:
    uint64_t sum{};
    int count{};
    int thresh{};
    uint32_t dur_avg{};
};

class Calibrator {
   public:
    Calibrator() {
        esp_err_t err;
        calib_handle = nvs::open_nvs_handle("storage", NVS_READWRITE, &err);
        ESP_ERROR_CHECK(err);
        bool fail = false;
        err = calib_handle->get_item("ofs_x", g_ofs_x);
        if (err != ESP_OK) fail = true;
        err = calib_handle->get_item("ofs_y", g_ofs_y);
        if (err != ESP_OK) fail = true;
        err = calib_handle->get_item("ofs_z", g_ofs_z);
        if (err != ESP_OK) fail = true;

        a_ofs_x = gctx.settings_manager->Get("accel_ofs_x") / kAccelToG;
        a_ofs_y = gctx.settings_manager->Get("accel_ofs_y") / kAccelToG;
        a_ofs_z = gctx.settings_manager->Get("accel_ofs_z") / kAccelToG;

        ESP_LOGI(kLogTag, "Loaded accel offsets %d %d %d", a_ofs_x, a_ofs_y, a_ofs_z);

        if (fail) {
            ESP_LOGE(kLogTag, "Failed load gyro calibration");
            g_ofs_x = g_ofs_y = g_ofs_z = 0;
            StoreCalibration();
        } else {
            ESP_LOGI(kLogTag, "Gyro calibration loaded");
        }
    }
    void ProcessSample(sample &s) {
        auto &rs = std::get<raw_sample>(s.sample);
        RunGyroCalibration(s);
        RunAccelCalibration(s);

        rs.gx = std::max((int)std::numeric_limits<int16_t>::min(),
                         std::min((int)std::numeric_limits<int16_t>::max(), rs.gx + g_ofs_x));
        rs.gy = std::max((int)std::numeric_limits<int16_t>::min(),
                         std::min((int)std::numeric_limits<int16_t>::max(), rs.gy + g_ofs_y));
        rs.gz = std::max((int)std::numeric_limits<int16_t>::min(),
                         std::min((int)std::numeric_limits<int16_t>::max(), rs.gz + g_ofs_z));

        rs.ax += a_ofs_x;
        rs.ay += a_ofs_y;
        rs.az += a_ofs_z;
    }

   private:
    int g_ofs_x{}, g_ofs_y{}, g_ofs_z{};
    int a_ofs_x{}, a_ofs_y{}, a_ofs_z{};

    int gyr_samples{};
    int64_t sum_gx{}, sum_gy{}, sum_gz{};

    std::shared_ptr<nvs::NVSHandle> calib_handle;

    static constexpr int kGyroCalibrationSamples = 1000;
    void RunGyroCalibration(const sample &s) {
        if (gctx.logger_control.calibration_pending) {
            gctx.logger_control.calibration_pending = false;
            sum_gx = sum_gy = sum_gz = 0;
            gyr_samples = kGyroCalibrationSamples + 1;
            ESP_LOGI(kLogTag, "Gyro calibration started");
        }
        if (gyr_samples == 1) {
            g_ofs_x = -sum_gx / kGyroCalibrationSamples;
            g_ofs_y = -sum_gy / kGyroCalibrationSamples;
            g_ofs_z = -sum_gz / kGyroCalibrationSamples;
            gyr_samples = 0;
            ESP_LOGI(kLogTag, "Gyro calibration complete %d %d %d", g_ofs_x, g_ofs_y, g_ofs_z);
            StoreCalibration();
        } else if (gyr_samples) {
            auto &rs = std::get<raw_sample>(s.sample);
            sum_gx += rs.gx;
            sum_gy += rs.gy;
            sum_gz += rs.gz;
            gyr_samples -= 1;
        }
    }

    void RunAccelCalibration(const sample &s) {
        static int64_t prev_accel_export_time = esp_timer_get_time();
        if (esp_timer_get_time() - prev_accel_export_time > 50000) {
            prev_accel_export_time = esp_timer_get_time();
            auto &rs = std::get<raw_sample>(s.sample);
            quat::base_type ascale{kAccelToG / 16.0};
            quat::vec accel = quat::vec{ascale * rs.ax, ascale * rs.ay, ascale * rs.az};
            static constexpr double lpf_k = 0.1;
            if (xSemaphoreTake(gctx.logger_control.accel_raw_mtx, portMAX_DELAY)) {
                gctx.logger_control.accel_raw[0] = lpf_k * static_cast<double>(accel.x) * 16.0 +
                                                   (1 - lpf_k) * gctx.logger_control.accel_raw[0];
                gctx.logger_control.accel_raw[1] = lpf_k * static_cast<double>(accel.y) * 16.0 +
                                                   (1 - lpf_k) * gctx.logger_control.accel_raw[1];
                gctx.logger_control.accel_raw[2] = lpf_k * static_cast<double>(accel.z) * 16.0 +
                                                   (1 - lpf_k) * gctx.logger_control.accel_raw[2];

                static constexpr double gyr_lpf_k = 0.2;
                gctx.logger_control.gyro_raw[0] = gyr_lpf_k * static_cast<double>(rs.gx) * kGyroToRads + (1 - gyr_lpf_k) * gctx.logger_control.gyro_raw[0];
                gctx.logger_control.gyro_raw[1] = gyr_lpf_k * static_cast<double>(rs.gy) * kGyroToRads + (1 - gyr_lpf_k) * gctx.logger_control.gyro_raw[1];
                gctx.logger_control.gyro_raw[2] = gyr_lpf_k * static_cast<double>(rs.gz) * kGyroToRads + (1 - gyr_lpf_k) * gctx.logger_control.gyro_raw[2];
                xSemaphoreGive(gctx.logger_control.accel_raw_mtx);
            }
        }
    }

    void StoreCalibration() {
        esp_err_t err;
        bool fail = false;

        ESP_LOGI(kLogTag, "Storing gyro calibration into nvm");

        err = calib_handle->set_item("ofs_x", g_ofs_x);
        if (err != ESP_OK) fail = true;
        err = calib_handle->set_item("ofs_y", g_ofs_y);
        if (err != ESP_OK) fail = true;
        err = calib_handle->set_item("ofs_z", g_ofs_z);
        if (err != ESP_OK) fail = true;

        if (fail) {
            ESP_LOGE(kLogTag, "Failed to store gyro calibration into nvm");
        }
    }
};

class GyroRing {
   public:
    struct WorkResult {
        quat::quat *quats;
        int16_t *accels;
        int accels_len;
    };

    GyroRing() {
        gctx.filter_settings.pt_order = gctx.settings_manager->Get("pt_count");
        gctx.filter_settings.pt_cutoff = gctx.settings_manager->Get("pt_cutoff");
        gctx.filter_settings.accel_pt_order = gctx.settings_manager->Get("acc_pt_count");
        gctx.filter_settings.accel_pt_cutoff = gctx.settings_manager->Get("acc_pt_cutoff");
    }

    void Init(int capacity, int chunk_size, uint32_t desired_interval) {
        this->chunk_size_ = chunk_size;
        this->desired_interval_ = desired_interval;
        ring_.resize(capacity);
        acc_ring_.resize(20);
        quats_chunk_.reserve(chunk_size);
        inv_desired_interval_ = quat::base_type{1.0 / desired_interval_};
        ESP_LOGI(kLogTag, "inv_interval %f", (double)inv_desired_interval_);
    }

    void Push(uint32_t dur_ns, int16_t gx, int16_t gy, int16_t gz, int16_t ax, int16_t ay,
              int16_t az, int flags) {
        static int shadow_wptr{};

        auto &s = ring_[shadow_wptr];
        s.duration_ns = dur_smoother.Smooth(dur_ns);
        raw_sample rs = {
            .gx = gx, .gy = gy, .gz = gz, .ax = ax, .ay = ay, .az = az, .flags = flags};
        s.sample = rs;

        shadow_wptr = (shadow_wptr + 1) % ring_.size();

        taskENTER_CRITICAL_ISR(&ptr_mux_);
        wptr_ = shadow_wptr;
        taskEXIT_CRITICAL_ISR(&ptr_mux_);
    }

    WorkResult Work() {
        taskENTER_CRITICAL(&ptr_mux_);
        int cached_wptr = wptr_;
        taskEXIT_CRITICAL(&ptr_mux_);

        // ESP_LOGI(kLogTag, "rptr: %d, wptr: %d, count: %d", rptr_, cached_wptr,
        //  ((cached_wptr + ring_.size()) - rptr_) % ring_.size());
        static int accel_cnt = 0;

        if (quats_chunk_.size() >= chunk_size_) {
            quats_chunk_.clear();
            accel_chunk_.clear();
            accel_cnt = 0;
        }

        // Integrate gyro and LPF
        while (rptr_ != cached_wptr) {
            auto &s = ring_[rptr_];
            auto &rs = std::get<raw_sample>(s.sample);

            calib_.ProcessSample(s);

            float gscale = kGyroToRads * s.duration_ns / 1e9;
            auto gyro = quat::vec{quat::base_type{gscale * rs.gx}, quat::base_type{gscale * rs.gy},
                                  quat::base_type{gscale * rs.gz}};

            if (gctx.filter_settings.pt_order) {
                if (!pts_[0]) {
                    pts_[0] = new PtFilter(gctx.filter_settings.pt_order,
                                           gctx.filter_settings.pt_cutoff, gctx.gyro_sr);
                    pts_[1] = new PtFilter(gctx.filter_settings.pt_order,
                                           gctx.filter_settings.pt_cutoff, gctx.gyro_sr);
                    pts_[2] = new PtFilter(gctx.filter_settings.pt_order,
                                           gctx.filter_settings.pt_cutoff, gctx.gyro_sr);
                }

                gyro.x = pts_[0]->apply(gyro.x);
                gyro.y = pts_[1]->apply(gyro.y);
                gyro.z = pts_[2]->apply(gyro.z);
            }

            if (fpm::abs(gyro.x) > quat::base_type{2} || fpm::abs(gyro.y) > quat::base_type{2} ||
                fpm::abs(gyro.z) > quat::base_type{2}) {
                ESP_LOGW("ring", "Filter unstable?");
                gyro = {};
            }

            quat::quat pred = quat_rptr_ * quat::quat{gyro};

            static quat::vec f_accel;
            if (rs.flags & kFlagHaveAccel) {
                quat::base_type ascale{kAccelToG / 256};
                quat::vec accel = quat::vec{ascale * rs.ax, ascale * rs.ay, ascale * rs.az};
                accel = pred.rotate_point(accel);
                if (!pts_accel_[0]) {
                    pts_accel_[0] =
                        new PtFilter(gctx.filter_settings.accel_pt_order,
                                     gctx.filter_settings.accel_pt_cutoff, gctx.accel_sr);
                    pts_accel_[1] =
                        new PtFilter(gctx.filter_settings.accel_pt_order,
                                     gctx.filter_settings.accel_pt_cutoff, gctx.accel_sr);
                    pts_accel_[2] =
                        new PtFilter(gctx.filter_settings.accel_pt_order,
                                     gctx.filter_settings.accel_pt_cutoff, gctx.accel_sr);
                }

                f_accel.x = pts_accel_[0]->apply(accel.x);
                f_accel.y = pts_accel_[1]->apply(accel.y);
                f_accel.z = pts_accel_[2]->apply(accel.z);
                f_accel = pred.conj().rotate_point(f_accel);
            }
            {
                static int accel_div{};
                if (!accel_div) {
                    accel_div = gctx.gyro_sr / 10.0;
                }

                if (++accel_cnt % accel_div == 0) {
                    const int scale = 256 * 32768 / 16;
                    acc_ring_[acc_wptr_].gyro_ref = rptr_;
                    acc_ring_[acc_wptr_].acc[0]= (int16_t)(-((float)f_accel.x) * scale);
                    acc_ring_[acc_wptr_].acc[1]= (int16_t)(-((float)f_accel.y) * scale);
                    acc_ring_[acc_wptr_].acc[2]= (int16_t)(-((float)f_accel.z) * scale);
                    acc_wptr_ = (acc_wptr_ + 1) % acc_ring_.size();
                    // printf("%f %f %f %d %d\n", ((float)f_accel.x) * 256, ((float)f_accel.y) *
                    // 256,
                    //    ((float)f_accel.z) * 256, accel_chunk_.size() / 3,
                    //    (int)(int16_t)(((float)f_accel.x) * scale));
                }
            }

            quat_rptr_ = pred;

            MaybeNormalize(quat_rptr_);

            s.sample = quat_rptr_;

            rptr_ = (rptr_ + 1) % ring_.size();
        }

        // See how many samples we have buffered
        int samples_buffered = ((cached_wptr + ring_.size()) - sptr_) % ring_.size();

        if (samples_buffered < kResamlpingLag) {
            return {.quats = nullptr};
        }

        // ESP_LOGI(kLogTag, "%d %u", samples_buffered, ring_[sptr_].duration_ns);

        // Quaternion interpolation
        samples_buffered -= kResamlpingLag;

        if (quats_chunk_.size() >= chunk_size_) {
            quats_chunk_.clear();
            accel_chunk_.clear();
            accel_cnt = 0;
        }

        while (samples_buffered) {
            {  // advance sptr while we can
                while (samples_buffered) {
                    int next_sptr = (sptr_ + 1) % ring_.size();
                    uint32_t next_sptr_ts = sptr_ts_ + ring_[next_sptr].duration_ns;
                    if (next_sptr_ts < interp_ts_) {
                        if (sptr_ == acc_ring_[acc_rptr_].gyro_ref) {
                            accel_chunk_.push_back(acc_ring_[acc_rptr_].acc[0]);
                            accel_chunk_.push_back(acc_ring_[acc_rptr_].acc[1]);
                            accel_chunk_.push_back(acc_ring_[acc_rptr_].acc[2]);
                            acc_rptr_ = (acc_rptr_ + 1) % acc_ring_.size();
                        }
                        sptr_ = next_sptr;
                        sptr_ts_ = next_sptr_ts;
                        samples_buffered -= 1;
                    } else {
                        break;
                    }
                }
                if (!samples_buffered) break;
            }
            {  // wrap the timestamps if needed
                static constexpr uint32_t kTsWrapInterval = 200000000;
                if (interp_ts_ >= 2 * kTsWrapInterval) {
                    interp_ts_ -= kTsWrapInterval;
                    sptr_ts_ -= kTsWrapInterval;
                    if (loglevel >= 3) {
                        ESP_LOGI(kLogTag, "wrap");
                    }
                }
            }
            {  // Interpolate
                int next_sptr = (sptr_ + 1) % ring_.size();
                quat::base_type k2 = quat::base_type{static_cast<float>(interp_ts_ - sptr_ts_) /
                                                     ring_[next_sptr].duration_ns},
                                k1 = quat::base_type{1} - k2;
                quat::quat q = std::get<quat::quat>(ring_[sptr_].sample) * k1 +
                               std::get<quat::quat>(ring_[next_sptr].sample) * k2;

                int cached_sptr = (sptr_ + 1) % ring_.size();
                int cached_sptr_ts = sptr_ts_ + ring_[cached_sptr].duration_ns;

                interp_ts_ += desired_interval_ * 1000;

                quats_chunk_.push_back(q);
                if (quats_chunk_.size() >= chunk_size_) {
                    if (loglevel >= 2) {
                        ESP_LOGI(kLogTag, "produced chunk size %d", chunk_size_);
                    }
                    return {.quats = quats_chunk_.data(),
                            .accels = accel_chunk_.data(),
                            .accels_len = (int)accel_chunk_.size()};
                }
            }
        }
        return {.quats = nullptr};
    }

    uint32_t GetInterval() {
        return desired_interval_;
    }

   private:
    void MaybeNormalize(quat::quat &q) {
        static uint8_t x = 0;
        if ((++x % 16) == 0) q = q.normalized();
    }

   private:
    static constexpr int kResamlpingLag = 32;

    int chunk_size_;
    std::vector<sample> ring_;
    std::vector<acc_sample> acc_ring_;
    DurationSmoother dur_smoother{8000};
    Calibrator calib_{};

    uint32_t desired_interval_;
    quat::base_type inv_desired_interval_;
    uint32_t interp_ts_{};
    uint32_t sptr_ts_{};

    portMUX_TYPE ptr_mux_ = portMUX_INITIALIZER_UNLOCKED;
    volatile int wptr_{};
    int rptr_{}, sptr_{};
    quat::quat quat_rptr_{};

    int acc_rptr_{}, acc_wptr_{};

    PtFilter *pts_[3] = {};
    PtFilter *pts_accel_[3] = {};

    std::vector<quat::quat> quats_chunk_;
    std::vector<int16_t> accel_chunk_;
};