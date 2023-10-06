#include "lib/compression.hpp"
#include <emscripten.h>
#include <sys/types.h>
#include <fstream>
#include <iostream>
#include "wifi/cam_control.hpp"

std::vector<uint8_t> input;
bool fail = false;
static constexpr int kMaxOutputSize = 1024 * 1024;
static constexpr int kBlockSize = 256;
static constexpr double sample_rate = 1.0 / 0.00125;
static constexpr double gscale = 1 / 0.00053263221;

static int ztime = 0;
static int pos = 0;
static quat::quat prev_quat(quat::base_type{1}, {}, {}, {});
static Coder decoder(kBlockSize, 22);

extern "C" {
EMSCRIPTEN_KEEPALIVE
uint8_t* allocate_input(int size) {
    input.resize(size);
    ztime = 0;
    pos = 0;
    prev_quat = {quat::base_type{1}, {}, {}, {}};
    decoder = Coder{kBlockSize, 22};
    return input.data();
}

EMSCRIPTEN_KEEPALIVE
int decode(const char* filename) {  // Receive filename as a parameter
    // Open file for writing
    std::ofstream outFile;
    outFile.open(filename, std::ios::out | std::ios::app);  // Use the provided filename
    if(!outFile) {
        std::cerr << "File opening failed" << std::endl;
        return -1;  // or other error handling
    }

    while (pos < input.size()) {
        auto [decoded_bytes, dquats, scale] =
            decoder.decode_block(input.data() + pos, input.size() - pos);
        if (fail || !decoded_bytes) break;
        pos += decoded_bytes;

        int accel_count = input[pos++];
        int16_t* accel_data = (int16_t*)(input.data() + pos);
        pos += 6 * accel_count;

        int ascale = (256 * 32768) / (10000 * 16);

        int i = 0;
        for (auto& q : dquats) {
            int i_lim = std::min(i++ / 55, accel_count - 1);
            quat::vec rv = (q.conj() * prev_quat).axis_angle();
            prev_quat = q;
            if (ztime != 0) {
                double scale = sample_rate * gscale;
                double ascale = 10000;
                outFile << ztime << ","
                        << (int)(double(rv.x) * scale) << ","
                        << (int)(double(rv.y) * scale) << ","
                        << (int)(double(rv.z) * scale) << ","
                        << (int)(accel_data[0 + 3 * i_lim]) << ","
                        << (int)(accel_data[1 + 3 * i_lim]) << ","
                        << (int)(accel_data[2 + 3 * i_lim]) << std::endl;
            }
            ztime++;
        }
    }
    
    // Close file when done
    outFile.close();

    return 0;
}

void _Exit(int) { fail = true; }
int __stdio_close(int) { return {}; }
int __stdio_write(int, int, int) { return {}; }
off_t __lseek(int, off_t, int) { return {}; }
}
