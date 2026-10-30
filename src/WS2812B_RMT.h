#ifndef WS2812B_RMT_H
#define WS2812B_RMT_H

#include <Arduino.h>                             // Include the standard Arduino framework definitions
#include "driver/rmt_tx.h"                       // Include the ESP32 RMT (Remote Control) transmitter driver API
#include "driver/rmt_encoder.h"                  // Include the ESP32 RMT encoder driver API to format data bits
#include "soc/soc_caps.h"                        // Include chip capabilities (RMT memory size per channel)

#define c_u32Ws2812bT0hTicks   28                // Define RMT tick duration for bit 0 High state (~350ns at 80MHz)
#define c_u32Ws2812bT0lTicks   64                // Define RMT tick duration for bit 0 Low state (~800ns at 80MHz)
#define c_u32Ws2812bT1hTicks   56                // Define RMT tick duration for bit 1 High state (~700ns at 80MHz)
#define c_u32Ws2812bT1lTicks   48                // Define RMT tick duration for bit 1 Low state (~600ns at 80MHz)

class CWs2812bRmt {
private:
    int m_iLedPin;                               // GPIO pin number assigned for the WS2812B data line
    size_t m_szNumLeds;                          // Total number of addressable LEDs in the strip
    size_t m_szTotalBytes;                       // Total size of the frame buffer in bytes (3 bytes per LED: G, R, B)
    uint8_t m_u8Brightness;                      // Global brightness level scaled from 0 (off) to 255 (max)
    bool m_bGammaEnabled;                        // When true, vSetPixel() applies gamma correction after brightness scaling

    rmt_channel_handle_t m_hTxChan;              // Handle for the allocated ESP32 RMT TX channel
    rmt_encoder_handle_t m_hLedEncoder;          // Handle for the custom RMT bytes encoder

    QueueHandle_t m_hLedQueue;                   // FreeRTOS queue handle used to pass frame buffers to Core 0
    TaskHandle_t m_hLedTask;                     // FreeRTOS task handle for the background LED driver worker
    TaskHandle_t m_hStopWaiter;                  // Task waiting in the destructor for the worker to finish shutting down
    volatile bool m_bStopRequested;              // Set by the destructor to ask the worker task to exit on its next frame

    uint8_t *m_pu8FrameBuffer;                   // Dynamic memory pointer holding the current local frame buffer

    static void LedTaskCore0(void *pParameters); // Static FreeRTOS task handler executed strictly on CPU Core 0

public:
    CWs2812bRmt(int iLedPin, size_t szNumLeds);  // Constructor to set up configurations and allocate frame buffer memory
    ~CWs2812bRmt();                              // Destructor to safely clean up hardware channels, tasks, and memory
    CWs2812bRmt(const CWs2812bRmt&) = delete;    // Forbid copying; a copy would share and double-free buffers and handles
    CWs2812bRmt& operator=(const CWs2812bRmt&) = delete; // Forbid copy assignment for the same reason

    bool bInit();                                // Initializes the RMT peripherals, creates the queue, and spawns the core task
    uint8_t* pu8GetBuffer();                     // Returns a raw pointer to the internal frame buffer
    size_t szGetBufferSize() const;              // Returns the total capacity of the frame buffer in bytes
    void vSetBrightness(uint8_t u8Brightness);   // Sets the master scaling value for pixel brightness
    void vSetGammaCorrection(bool bEnable);      // Enables or disables gamma correction (gamma 2.8) in vSetPixel()
    static uint8_t u8Gamma(uint8_t u8Value);     // Returns the gamma-corrected value of one color channel (for direct buffer access)
    void vSetPixel(size_t szIndex, uint8_t u8R, uint8_t u8G, uint8_t u8B); // Sets specific RGB colors for a pixel index with integrated brightness scaling
    void vShow(); // Pushes the frame buffer into the queue to trigger hardware transmission
    void vClear(); // Clears the internal frame buffer by resetting all elements to zero
    void vGetWheelColor(uint8_t u8WheelPos, uint8_t &u8R, uint8_t &u8G, uint8_t &u8B); // Generates smooth rainbow colors based on a byte-sized wheel position
};

#endif // WS2812B_RMT_H
