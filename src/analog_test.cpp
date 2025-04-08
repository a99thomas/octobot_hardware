// #include <Arduino.h>

// #define NUM_MOTORS 9

// uint8_t read_pins[NUM_MOTORS] = {36, 39, 34, 35, 32, 33, 25, 26, 27};

// float mapFloat(int x, int in_min, int in_max, float out_min, float out_max) {
//     return (float)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

// void setup() {
//     Serial.begin(115200);
//     for (int i = 0; i < NUM_MOTORS; i++) {
//         pinMode(read_pins[i], INPUT);
//     }
// }

// void loop() {
//     for (int i = 0; i < NUM_MOTORS; i++) {
//         int raw_value = analogRead(read_pins[i]);
//         float mapped_value = mapFloat(raw_value, 255, 3805, 0.0, 360.0);
//         Serial.print("Motor ");
//         Serial.print(i);
//         Serial.print(" (Pin ");
//         Serial.print(read_pins[i]);
//         Serial.print("): Raw = ");
//         Serial.print(raw_value);
//         Serial.print(", Mapped = ");
//         Serial.println(mapped_value);
//     }
//     delay(500);  // Adjust delay as needed
// }
