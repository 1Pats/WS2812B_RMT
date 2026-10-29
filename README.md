# WS2812B_RMT

Lightweight, non-blocking WS2812B LED strip driver for dual-core ESP32.

Pixel data is encoded by the ESP32 **RMT** peripheral, and transmission runs in a
background FreeRTOS task pinned to **Core 0**. `vShow()` only copies the frame into a
queue and returns at once, so your sketch on Core 1 is never blocked by the LED update.

## ✅ Requirements

- ESP32 board (dual-core recommended)
- Arduino-ESP32 core **3.x** (ESP-IDF 5 RMT driver, `driver/rmt_tx.h`)

## 🔌 Wiring

| Strip | ESP32                                   |
|-------|-----------------------------------------|
| DIN   | Any output-capable GPIO (e.g. GPIO 18)  |
| GND   | GND (common ground with the strip PSU)  |
| 5V    | External 5 V supply sized for the strip |

## 🚀 Quick Start

```cpp
#include <WS2812B_RMT.h>

CWs2812bRmt leds(18, 30);   // GPIO, number of LEDs

void setup() {
    if (!leds.bInit()) while (true);
    leds.vSetBrightness(64);
}

void loop() {
    leds.vSetPixel(0, 255, 0, 0);
    leds.vShow();
    delay(20);
}
```

See `examples/WS2812B_RMT_Demo` for a full rainbow demo.

## 🌈 Multiple Strips

Create one object per strip. Each one gets its own RMT channel and its own Core 0 task.

```cpp
CWs2812bRmt strip1(18, 30);
CWs2812bRmt strip2(19, 60);
```

- Objects cannot be copied. Pass them to functions by reference (`CWs2812bRmt &s`).
- The strip count is limited by the RMT TX channels: ESP32 = 8, ESP32-S3 = 4, ESP32-C3/C6 = 2.
  `bInit()` returns `false` when no channel is left.
- RAM per strip: about 3 × (LEDs × 3) bytes of buffers plus a 4 KB task stack.

## 📚 API

| Method | Description |
|--------|-------------|
| `CWs2812bRmt(int iLedPin, size_t szNumLeds)` | Create a driver and allocate the frame buffer |
| `bool bInit()` | Set up RMT channel, encoder, queue and Core 0 task; returns `false` on failure |
| `void vSetPixel(size_t szIndex, uint8_t u8R, uint8_t u8G, uint8_t u8B)` | Set one pixel (brightness is applied here) |
| `void vSetBrightness(uint8_t u8Brightness)` | Set brightness 0–255; affects pixels set **after** the call |
| `void vClear()` | Set all pixels in the buffer to black |
| `void vShow()` | Send the current frame to the strip (non-blocking) |
| `void vGetWheelColor(uint8_t u8WheelPos, uint8_t &u8R, uint8_t &u8G, uint8_t &u8B)` | Rainbow color for a wheel position 0–255 |
| `uint8_t* pu8GetBuffer()` | Raw frame buffer, 3 bytes per LED in **GRB** order |
| `size_t szGetBufferSize() const` | Frame buffer size in bytes |

## 🛠️ Direct Buffer Access

To keep the library lightweight, the number of functions is limited. To manipulate the strip
directly, use `pu8GetBuffer()`, which returns a pointer to the LED color data.

- Bytes are stored in **GRB** order, 3 bytes per LED.
- LED index `i` (0-based, same as `vSetPixel`) starts at byte offset `i * 3`.
- Brightness is **not** applied to bytes written directly.
- Call `vShow()` afterwards to send the changes to the strip.

```cpp
uint8_t *pu8Buf = leds.pu8GetBuffer();
size_t i = 5;              // sixth LED
pu8Buf[i * 3 + 0] = 255;   // G
pu8Buf[i * 3 + 1] = 0;     // R
pu8Buf[i * 3 + 2] = 0;     // B
leds.vShow();
```

## 🎯 Design Principles

- KISS (Keep It Simple and Small)
- Minimal memory footprint
- Non-blocking: LED transmission never stalls the main loop

## 📄 License

MIT License — see [LICENSE](LICENSE).
