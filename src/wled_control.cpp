// #include <Adafruit_NeoPixel.h>

// // Define the pin where the built-in RGB LED is connected
// #define LED_PIN 48

// // Define the number of LEDs in the strip (usually 1 for built-in LED)
// #define NUM_LEDS 1

// // Create an instance of the Adafruit_NeoPixel class
// Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// void setup() {
// delay(500);
// Serial.begin(115200);
// delay(500);
// Serial.println("\n\n================================");
// Serial.print("Chip Model: ");
// Serial.println(ESP.getChipModel());
// Serial.print("Chip version: ");
// Serial.println(ESP.getChipRevision());
// Serial.print("Numer of cores: ");
// Serial.println(ESP.getChipCores());
// Serial.print("Flash Chip Size: ");
// Serial.println(ESP.getFlashChipSize());
// Serial.print("Flash Chip Speed: ");
// Serial.println(ESP.getFlashChipSpeed());

// Serial.println("================================");

// // Initialize the NeoPixel library
// strip.begin();
// strip.show(); // Initialize all pixels to 'off'
// }

// int i=0;
// // the loop function runs over and over again forever
// void loop() {
// delay(1000);
// Serial.print("Loop: ");
// Serial.println(i++);

// // Cycle through some colors

// // Red
// strip.setPixelColor(0, strip.Color(25, 0, 0)); // Red
// strip.show();
// delay(1000);

// // Green
// strip.setPixelColor(0, strip.Color(0, 25, 0)); // Green
// strip.show();
// delay(1000);

// // Blue
// strip.setPixelColor(0, strip.Color(0, 0, 25)); // Blue
// strip.show();
// delay(1000);

// // Yellow
// strip.setPixelColor(0, strip.Color(25, 25, 0)); // Yellow
// strip.show();
// delay(1000);

// // Cyan
// strip.setPixelColor(0, strip.Color(0, 25, 25)); // Cyan
// strip.show();
// delay(1000);

// // Magenta
// strip.setPixelColor(0, strip.Color(25, 0, 25)); // Magenta
// strip.show();
// delay(1000);

// // White
// strip.setPixelColor(0, strip.Color(25, 25, 25)); // White
// strip.show();
// delay(1000);

// // Turn off
// strip.setPixelColor(0, strip.Color(0, 0, 0)); // Off
// strip.show();
// delay(1000);
// }