#pragma once

#include <stdint.h>

// Initialize DMA for fetching the camera timestamp
void initCameraTimestampDma();

// Fetch the timestamp from the camera using DMA
void fetchCameraTimestamp();

// Get the timestamp that was fetched
uint32_t getFetchedCameraTimestamp();

