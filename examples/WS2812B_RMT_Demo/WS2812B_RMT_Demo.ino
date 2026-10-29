#include <WS2812B_RMT.h>

#define LED_PIN   18  // your GPIO
#define NUM_LEDS  30  // your LED count

CWs2812bRmt leds(LED_PIN, NUM_LEDS);

void setup() {
    Serial.begin(115200);
    
    if (leds.bInit()) Serial.println("WS2812B RMT Initialized!");
    else {
        Serial.println("WS2812B RMT Initialization Error!");
        while(true);
    }
    
    leds.vSetBrightness(64); // Brightness: ~25% (0-255)
}

void loop() {
    static uint8_t j = 0;
    uint8_t r, g, b;

    for (size_t i = 0; i < NUM_LEDS; i++) {
        leds.vGetWheelColor(((i * 256 / NUM_LEDS) + j) & 255, r, g, b);
        leds.vSetPixel(i, r, g, b);
    }
    
    leds.vShow();
    j++;
    delay(20);
}
