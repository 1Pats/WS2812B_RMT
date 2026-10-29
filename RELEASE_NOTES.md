# Release Notes — WS2812B_RMT

## V1.0.0

Initial release.

- WS2812B driver based on the ESP32 RMT TX peripheral (80 MHz, 12.5 ns resolution)
- Non-blocking `vShow()`: frames are queued and transmitted by a FreeRTOS task on Core 0
- Global brightness scaling via `vSetBrightness()`
- Rainbow helper `vGetWheelColor()`
- Example: `WS2812B_RMT_Demo`
- Multiple strips supported: copying is disabled, RMT memory uses one block per channel on every chip, and each task is named after its GPIO (`LedTaskGpio<pin>`)
- Fixed: destructor no longer deletes an already deleted task when the task buffer allocation fails
- Fixed: destructor now asks the Core 0 task to stop, so the task frees its own frame buffer and is never killed mid-transmission
- Fixed: example moved to `examples/WS2812B_RMT_Demo/` so the Arduino IDE lists it

## Notes

- Requires Arduino-ESP32 core 3.x.
- Brightness is applied when a pixel is set; changing it does not rescale pixels already in the buffer.
