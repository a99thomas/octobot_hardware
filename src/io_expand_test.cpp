
// #include "Arduino.h"
// #include "PCF8575.h"

// #define LED_PIN 7          // GPIO pin where the LED is connected
// #define PWM_CHANNEL 0      // LEDC channel (0-15)
// #define PWM_FREQUENCY 5000 // Frequency in Hz
// #define PWM_RESOLUTION 8   // 8-bit resolution (0-255)

// uint8_t pwm_pins[] = {19, 18, 17, 16, 4, 2, 13, 12, 23};  
// int num_pins = sizeof(pwm_pins)/ sizeof(pwm_pins[0]);


// uint8_t read_pins[] = {36, 39, 34, 35, 32, 33, 25, 26, 27};  
// int num_read_pins = sizeof(read_pins)/ sizeof(read_pins[0]);



// // Set i2c address
// PCF8575 pcf8575(0x20);


// void setup(){
// 	Serial.begin(115200);
//   Serial.println(num_pins);

// 	// Set pinMode to OUTPUT
// 	pcf8575.pinMode(P0, OUTPUT);
// 	pcf8575.pinMode(P1, OUTPUT);
//   // ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
//   // ledcAttachPin(LED_PIN, PWM_CHANNEL);

// 	Serial.println(pcf8575.begin());

//   for (int i = 0; i < num_pins; i++) {
//     Serial.print(i);
//     Serial.println(pwm_pins[i]);
//       ledcSetup(i, PWM_FREQUENCY, PWM_RESOLUTION);
//       ledcAttachPin(pwm_pins[i], i);
//       Serial.flush();

//   }
//   Serial.println("READ");
//   for (int i = 0; i < num_read_pins; i++) {
//     Serial.print(i);
//     Serial.println(read_pins[i]);
//     pinMode(read_pins[i], INPUT);
//   }
//   delay(1000);
// }

// void loop()
// {
//   for (int pin = 0; pin < num_read_pins; pin++){
//     Serial.print("PIn: ");
//     Serial.print(read_pins[pin]);
//     Serial.print("  ");
//     Serial.println(analogRead(read_pins[pin]));
//     Serial.flush();
//   }

// 	Serial.print("ONE");
//   pcf8575.digitalWrite(P0, HIGH);
//   pcf8575.digitalWrite(P1, LOW);

//   for (int duty = 0; duty <= 255; duty += 5) {
//     for(int channel=0; channel<=num_pins; channel++){
//       ledcWrite(channel, duty);
//       // Serial.print("Channel ");
//       // Serial.print(channel);
//       // Serial.print(", ");
//       // Serial.println(duty);
//     }
//     delay(20);  // Small delay for smooth dimming
// }

// pcf8575.digitalWrite(P0, LOW);
// pcf8575.digitalWrite(P1, HIGH);
// // Gradually decrease brightness
// for (int duty = 255; duty >= 0; duty -= 5) {
//   for(int channel=0; channel<=num_pins; channel++){
//     ledcWrite(channel, duty);
//   }
//     delay(20);
// }



// //   for (int duty = 0; duty <= 255; duty += 5) {
// //     for(int channel=0; channel<=num_pins; channel++){
// //       ledcWrite(channel, duty);
// //     }
// //     delay(20);  // Small delay for smooth dimming
// // }

// // // Gradually decrease brightness
// // for (int duty = 255; duty >= 0; duty -= 5) {
// //   for(int channel=0; channel<=num_pins; channel++){
// //     ledcWrite(channel, duty);
// //   }
// //     delay(20);
// // }
// }