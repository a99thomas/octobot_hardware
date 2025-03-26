// #include <Arduino.h>
// #include <Preferences.h>

// Preferences preferences;  // Create NVS object
// int savedPosition = 0;

// void setup() {
//     Serial.begin(115200);
//     // Open NVS storage

//     preferences.begin("storage", false);
//     // Save a variable
//     // int lastPosition = 1234;  // Example position
//     preferences.putInt("encoderPos", savedPosition);
//     Serial.println("Saved Position: " + String(savedPosition));

//     // Retrieve the stored variable
//     savedPosition = preferences.getInt("encoderPos", 0);
//     Serial.println("Recovered Position: " + String(savedPosition));

//     preferences.end();  // Close storage
// }

// void loop(){}

////////// NVME Run code.... DO NOT RUN BECAUSE YOU CAN DEGRADE FLASH QUICKLY!!!!!!
////////// JUST AN EXAMPLE FOR HOW TO DO IT


// #include <Arduino.h>
// #include <Preferences.h>

// Preferences preferences;  // Create NVS object
// int savedPosition = 0;


// void setup() {
//     Serial.begin(115200);
//       // Open NVS storage

//     preferences.begin("storage", false);
//     // Save a variable
//     // int lastPosition = 1234;  // Example position
//     // preferences.putInt("encoderPos", lastPosition);
//     // Serial.println("Saved Position: " + String(lastPosition));

//     // Retrieve the stored variable
//     savedPosition = preferences.getInt("encoderPos", 0);
//     Serial.println("Recovered Position: " + String(savedPosition));

//     preferences.end();  // Close storage
// }

// void loop() {
//     // Nothing here
//     savedPosition += 1;
//     // Serial.println(savedPosition);
//     unsigned long start_time = micros();
//     preferences.begin("storage", false);
//     preferences.putInt("encoderPos", savedPosition);
//     Serial.print("Yo");
//     Serial.println(String(preferences.getInt("encoderPos", 32)));
//     preferences.end();
//     unsigned long dt = micros()-start_time;
//     Serial.println(dt);
//     delay(100);
// }

