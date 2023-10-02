#include "camera_timestamp_dma.hpp"
extern "C" {
#include "esp_dma.h"
#include <esp_log.h>
#include <driver/uart.h>
}

#include <cstring>
#include <cstdlib>

// Constants and macros
#define TAG "camera_timestamp_dma"
#define DMA_CHANNEL_USED DMA_CHANNEL_0

// The source address of the timestamp data
static uint8_t* timestamp_source_addr;

// The destination address of the timestamp data
static uint8_t timestamp_destination_addr[sizeof(uint32_t)];

// The size of the timestamp data
static size_t timestamp_size;

// Function to initialize the DMA
static void initDma() {
    dma_config_t dma_config = {
        .channel = DMA_CHANNEL_USED,
        .gpio_intr_type = GPIO_INTR_DISABLE,
        .flags = DMA_FLAG_RW_8BIT,
    };

    esp_err_t err = dma_init(&dma_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing DMA: %d", err);
        return;
    }
}

// Initialize the source of the timestamp data
static void initTimestampSourceAddress() {
    timestamp_source_addr = uart_rx_buffer(UART_NUM_2);
}

// Initialize the destination of the timestamp data
static void initTimestampDestinationAddress() {
    timestamp_destination_addr = new uint8_t[sizeof(uint32_t)];
}

// Initialize the timestamp data size
static void initTimestampSize() {
    timestamp_size = 4;
}

// Set the DMA source address
static void setSourceAddress() {
    esp_dma_set_source_addr(DMA_CHANNEL_USED, timestamp_source_addr);
}

// Set the DMA destination address
static void setDestinationAddress() {
    esp_dma_set_destination_addr(DMA_CHANNEL_USED, timestamp_destination_addr);
}

// Set the DMA transfer size
static void setTransferSize() {
    esp_dma_set_transfer_size(DMA_CHANNEL_USED, timestamp_size);
}

// Start the DMA transfer
static void startDmaTransfer() {
    esp_dma_start(DMA_CHANNEL_USED);
}

void initCameraTimestampDma() {
    initDma();
    initTimestampSourceAddress();
    initTimestampDestinationAddress();
    initTimestampSize();
}

// Fetch the timestamp from the camera using DMA
void fetchCameraTimestamp() {
    esp_dma_set_source_addr(DMA_CHANNEL_USED, timestamp_source_addr);
    esp_dma_set_destination_addr(DMA_CHANNEL_USED, timestamp_destination_addr);
    esp_dma_set_transfer_size(DMA_CHANNEL_USED, timestamp_size);
    esp_dma_start(DMA_CHANNEL_USED);
}

// Get the fetched timestamp
uint32_t getCameraTimestamp() {
    if (timestamp_destination_addr) {
        return *(reinterpret_cast<uint32_t*>(timestamp_destination_addr));
    } else {
        // Return some default or error value if the timestamp hasn't been fetched yet
        return 0xFFFFFFFF;
    }
}
