#include "WS2812B_RMT.h"

void CWs2812bRmt::LedTaskCore0(void *pParameters) {                  // Background FreeRTOS task running on Core 0 to handle asynchronous hardware transmission
    CWs2812bRmt *pInstance = static_cast<CWs2812bRmt*>(pParameters); // Cast generic parameters pointer back into the usable class instance context
    uint8_t *pu8LocalGrb = (uint8_t *)malloc(pInstance->m_szTotalBytes); // Dynamically allocate local task memory to securely duplicate incoming frame bytes

    if (!pu8LocalGrb) {                                               // Validate allocation; if system memory is full, safely terminate this isolated task
        pInstance->m_hLedTask = NULL;                                 // Clear handle so the destructor does not delete an already deleted task
        vTaskDelete(NULL);
        return;
    }

    rmt_transmit_config_t stTransmitConfig = {.loop_count = 0, .flags = { .eot_level = 0 }}; // Set RMT hardware transmission configuration structure (no loops, default standard end-of-transmission level)

    while (true) { // Continuous execution loop for the background worker thread
        if (xQueueReceive(pInstance->m_hLedQueue, pu8LocalGrb, portMAX_DELAY) == pdTRUE) { // Block indefinitely until a raw native GRB frame arrives via the thread-safe queue
            if (pInstance->m_bStopRequested) {                        // Destructor asked us to exit; release our own memory and report back
                free(pu8LocalGrb);
                xTaskNotifyGive(pInstance->m_hStopWaiter);            // Wake the destructor; pInstance must not be touched after this line
                vTaskDelete(NULL);
            }
            rmt_transmit(pInstance->m_hTxChan, pInstance->m_hLedEncoder, pu8LocalGrb, pInstance->m_szTotalBytes, &stTransmitConfig); // Hand over memory buffer directly to the high-speed RMT peripheral using DMA/Hardware acceleration
            rmt_tx_wait_all_done(pInstance->m_hTxChan, -1);          // Block task execution until all data bits are completely flushed out onto the physical wire
            delayMicroseconds(300); // Delay for 300 microseconds to fulfill WS2812B protocol latch/reset timing requirements
        }
    }
}

CWs2812bRmt::CWs2812bRmt(int iLedPin, size_t szNumLeds) :            // Class constructor implementation matching initialization values
    m_iLedPin(iLedPin),                                              // Bind the hardware output data GPIO pin
    m_szNumLeds(szNumLeds),                                          // Store total amount of LEDs in configuration
    m_szTotalBytes(szNumLeds * 3),                                   // Compute target buffer size (3 bytes per pixel)
    m_u8Brightness(255),                                             // Set initialization brightness to full (100%)
    m_hTxChan(NULL),                                                 // Initialize peripheral channel handle as empty
    m_hLedEncoder(NULL),                                             // Initialize timing bit encoder handle as empty
    m_hLedQueue(NULL),                                               // Initialize FreeRTOS queue structure as empty
    m_hLedTask(NULL),                                                // Initialize worker task handler reference as empty
    m_hStopWaiter(NULL),                                             // No destructor is waiting for the worker yet
    m_bStopRequested(false) {                                        // Worker runs until the destructor requests a stop
        m_pu8FrameBuffer = (uint8_t *)malloc(m_szTotalBytes);        // Dynamically request block memory size for the core color frame buffer
        if (m_pu8FrameBuffer) {                                      // If allocation succeeds, explicitly wipe the memory buffer to clear LEDs on bootup
            memset(m_pu8FrameBuffer, 0, m_szTotalBytes);
        }
}

CWs2812bRmt::~CWs2812bRmt() {                                        // Destructor implementation to cleanly dismantle all active processes and allocations
    if (m_hLedTask) {                                                // If the worker task is running, ask it to stop so it can free its own frame copy
        m_hStopWaiter = xTaskGetCurrentTaskHandle();                 // Tell the worker which task to notify once it has cleaned up
        m_bStopRequested = true;
        xQueueOverwrite(m_hLedQueue, m_pu8FrameBuffer);              // Wake the worker; it finishes any frame in progress, then sees the stop flag
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);                     // Wait until the worker has released its memory and is about to delete itself
    }
    if (m_hLedQueue) vQueueDelete(m_hLedQueue);                      // If the inter-core communication queue structure exists, destroy and free it
    if (m_pu8FrameBuffer) free(m_pu8FrameBuffer);                    // Deallocate and release the main color frame buffer memory block back to the system heap
    if (m_hTxChan) {                                                 // If the active high-speed hardware RMT transmitter channel handle is configured
        rmt_disable(m_hTxChan);                                      // Formally stop signals on the designated RMT hardware module
        rmt_del_channel(m_hTxChan);                                  // Release and delete the hardware channel assignment back to the ESP32 resource pool
    }
    if (m_hLedEncoder) {                                             // If a custom bit configuration encoder module exists, delete it to clear hardware registers
        rmt_del_encoder(m_hLedEncoder);
    }
}

bool CWs2812bRmt::bInit() {                                          // Complete initialization function for RMT driver mapping and core setup
    if (!m_pu8FrameBuffer) return false;                             // Early exit safety check; if frame memory was never successfully allocated, initialization fails

    rmt_tx_channel_config_t stTxChanConfig = {                       // Define configuration values for the hardware RMT TX channel architecture
        .gpio_num = (gpio_num_t)m_iLedPin,                           // Assign the physical board data output pin
        .clk_src = RMT_CLK_SRC_DEFAULT,                              // Use standard default high-frequency system clock source
        .resolution_hz = 80000000,                                   // Establish clock base rate at exactly 80 MHz (12.5ns ticks)
        .mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL,          // Use exactly one RAM block per channel (64 on ESP32, 48 on S3/C3/C6) so every TX channel stays available
        .trans_queue_depth = 4,                                      // Set non-zero transfer queue depth to support driver operations
        .flags = { .invert_out = false, .with_dma = false }          // Keep digital output normal and bypass DMA module
    };

    if (rmt_new_tx_channel(&stTxChanConfig, &m_hTxChan) != ESP_OK) return false; // Register configuration setup to build a new hardware TX transmitter module channel

    rmt_bytes_encoder_config_t stBytesEncoderConfig = {              // Map sub-microsecond pulse-width timings for WS2812B logic high and logic low data states
        .bit0 = {
            .duration0 = c_u32Ws2812bT0hTicks, .level0 = 1,          // High state time duration for encoding a '0' bit
            .duration1 = c_u32Ws2812bT0lTicks, .level1 = 0           // Low state time duration for encoding a '0' bit
        },
        .bit1 = {
            .duration0 = c_u32Ws2812bT1hTicks, .level0 = 1,          // High state time duration for encoding a '1' bit
            .duration1 = c_u32Ws2812bT1lTicks, .level1 = 0           // Low state time duration for encoding a '1' bit
        },
        .flags = { .msb_first = 1 }                                  // Read and transmit array byte streams starting with the Most Significant Bit
    };

    if (rmt_new_bytes_encoder(&stBytesEncoderConfig, &m_hLedEncoder) != ESP_OK) return false; // Instantiate and finalize the configured bytes encoder within the peripheral structure
    if (rmt_enable(m_hTxChan) != ESP_OK) return false;               // Enable the newly configured RMT transmitter module to accept incoming buffer triggers

    m_hLedQueue = xQueueCreate(1, m_szTotalBytes);                   // Form a thread-safe message queue with a storage slot capacity of 1 single data frame buffer
    if (!m_hLedQueue) return false;                                  // If the OS fails to provide structure space for this queue configuration, abort initialization

    char acTaskName[configMAX_TASK_NAME_LEN];                        // Buffer for a unique task name per strip
    snprintf(acTaskName, sizeof(acTaskName), "LedTaskGpio%d", m_iLedPin); // Name the task after its GPIO pin, e.g. "LedTaskGpio18"

    BaseType_t xReturned = xTaskCreatePinnedToCore(                  // Deploy the asynchronous background driver task anchored exclusively inside CPU Core 0
        LedTaskCore0,                                                // Pass function pointer reference of the execution loop
        acTaskName,                                                  // Unique name tag string assigned to identify this specific thread (FreeRTOS copies it)
        4096,                                                        // Set stack depth allocation size to prevent data overflows
        this,                                                        // Hand over the execution instance address reference as parameter bridge
        2,                                                           // Assign a thread priority rating of level 2 inside the OS scheduler
        &m_hLedTask,                                                 // Bind the tracking variable pointer handle location
        0                                                            // Enforce hardware processing mapping directly onto Core 0
    );

    return (xReturned == pdPASS);                                    // Return verification status signaling whether FreeRTOS task creation was successful
}

uint8_t* CWs2812bRmt::pu8GetBuffer() {                               // Return raw data address pointer location of main array memory
    return m_pu8FrameBuffer;
}

size_t CWs2812bRmt::szGetBufferSize() const {                        // Return calculated volume of complete byte storage capacity
    return m_szTotalBytes;
}

void CWs2812bRmt::vSetBrightness(uint8_t u8Brightness) {             // Set master variable boundary used to adjust color scale outputs
    m_u8Brightness = u8Brightness;  
}

void CWs2812bRmt::vSetPixel(size_t szIndex, uint8_t u8R, uint8_t u8G, uint8_t u8B) { // Maps specific pixel index array properties into the dedicated GRB protocol pattern
    if (szIndex >= m_szNumLeds || !m_pu8FrameBuffer) return;         // Array safety ceiling check; discard instruction if out of bounds or buffer is unallocated

    if (m_u8Brightness < 255) {                                      // If brightness scaling is active (less than max value 255), apply bit-shifted math adjustments
        u8R = (uint16_t)(u8R * m_u8Brightness) >> 8;                 // Scale red color intensity
        u8G = (uint16_t)(u8G * m_u8Brightness) >> 8;                 // Scale green color intensity
        u8B = (uint16_t)(u8B * m_u8Brightness) >> 8;                 // Scale blue color intensity
    }

    m_pu8FrameBuffer[szIndex * 3 + 0] = u8G;                         // Force storage alignment matching native WS2812B hardware specifications (GRB sequence layout); assign green data value first
    m_pu8FrameBuffer[szIndex * 3 + 1] = u8R;                         // Assign red data value second
    m_pu8FrameBuffer[szIndex * 3 + 2] = u8B;                         // Assign blue data value third
}

void CWs2812bRmt::vShow() {                                          // Trigger asynchronous updates by pushing local updates over to the core processor queue
    if (m_hLedQueue != NULL && m_pu8FrameBuffer != NULL) {           // Verify valid operating structures; perform a non-blocking queue overwrite injection
        xQueueOverwrite(m_hLedQueue, m_pu8FrameBuffer);
    }
}

void CWs2812bRmt::vClear() {                                         // Wipe out memory values to pull lines low and prepare all dark state combinations
    if (m_pu8FrameBuffer) {
        memset(m_pu8FrameBuffer, 0, m_szTotalBytes);
    }
}

void CWs2812bRmt::vGetWheelColor(uint8_t u8WheelPos, uint8_t &u8R, uint8_t &u8G, uint8_t &u8B) { // Cycle transitions to extract custom shifting color wheel spectrum components
    u8WheelPos = 255 - u8WheelPos;
    if (u8WheelPos < 85) {
       u8R = 255 - u8WheelPos * 3; u8G = 0; u8B = u8WheelPos * 3;
    } else if (u8WheelPos < 170) {
       u8WheelPos -= 85;
       u8R = 0; u8G = u8WheelPos * 3; u8B = 255 - u8WheelPos * 3;
    } else {
       u8WheelPos -= 170;
       u8R = u8WheelPos * 3; u8G = 255 - u8WheelPos * 3; u8B = 0;
    }
}

