#include <FastLED.h>

#define PIN_LED_ESP 21

CRGB leds[1];

void setup(){
    FastLED.addLeds<WS2812, PIN_LED_ESP, GRB>(leds, 1);
    FastLED.setBrightness(50);
}

void loop(){
    leds[0] = CRGB::Red;
    FastLED.show();
    delay(500);
    leds[0] = CRGB::Blue;
    FastLED.show();
    delay(500);
}

