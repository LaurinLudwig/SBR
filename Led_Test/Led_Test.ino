#include <Adafruit_NeoPixel.h>

#define PIN        13 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 1 // Popular NeoPixel ring size

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 200 // Time (in milliseconds) to pause between pixels

void setup() {
  delay(10);
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

void loop() {
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();   // Send the updated pixel colors to the hardware.

  delay(DELAYVAL); // Pause before next pass through loop

  pixels.setPixelColor(0, pixels.Color(0, 100, 0));
  pixels.show();   // Send the updated pixel colors to the hardware.

  delay(DELAYVAL);
}
