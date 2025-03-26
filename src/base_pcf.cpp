// #include "Arduino.h"
// #include "PCF8575.h"

// // Set i2c address
// PCF8575 pcf8575(0x20);

// void setup()
// {
// 	Serial.begin(115200);

// 	// Set pinMode to OUTPUT
// 	pcf8575.pinMode(P0, OUTPUT);

// 	pcf8575.begin();
//     Serial.begin(115200);
// }

// void loop()
// {
//     Serial.println(millis());
// 	pcf8575.digitalWrite(P0, HIGH);
//     Serial.println(millis());
// 	delay(1000);
//     Serial.println(millis());
// 	pcf8575.digitalWrite(P0, LOW);
//     Serial.println(millis());
// 	delay(1000);
// }