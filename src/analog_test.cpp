// #include "Arduino.h"


// int min1 = 300;
// int max1 = 3000;
// float min_input = 255.0;
// float max_input = 3805.0;
// float min_output = 0.0;
// float max_output = 360.0;
// // int input = 300;
// uint8_t read_pin = 34; 

// // Setup
// void setup() {
//   Serial.begin(115200);
//   pinMode(read_pin, INPUT);
  
// }

// // Main loop
// void loop() {
//   // Read positions from analog pins
//     int raw_value = analogRead(read_pin);

//     // Map the analog input (0-4095 range on ESP32) to a float range (0-360 degrees)
//     float input = (float(raw_value) - min_input) * (max_output - min_output) / (max_input - min_input) + min_output;
  
//     // if (input < min1 && input > 200){
//     //     min1 = input;
//     // }
//     // if (input > max1){
//     //     max1 = input;
//     // }
//     // Serial.print("Min:" + String(min1));
//     // Serial.print("Max: " + String(max1));
//     Serial.println("Analog Read Value: " + String(input));


//   delay(100); // Small delay to allow PID control to stabilize
// }

