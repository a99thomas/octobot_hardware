// #include <Arduino.h>
// #include <PCF8575.h>
// #include <PID_v1.h>
// #include <Preferences.h>
// const int pwmChannel1 = 0;  // PWM channel for D2
// const int pwmChannel2 = 1;  // PWM channel for D4
// const int freq = 5000;      // Frequency in Hz
// const int resolution = 8;   // 8-bit resolution (0-255)

// void setup() {
//     // Configure PWM on pin D2
//     ledcSetup(pwmChannel1, freq, resolution);
//     ledcAttachPin(2, pwmChannel1); // D2

//     // Configure PWM on pin D4
//     ledcSetup(pwmChannel2, freq, resolution);
//     ledcAttachPin(4, pwmChannel2); // D4

//     // Start with complementary signals
//     ledcWrite(pwmChannel1, 255); // D2 at 100%
//     ledcWrite(pwmChannel2, 0);   // D4 at 0%
// }

// void loop() {
//     // Toggle complementary PWM every 500ms
//     delay(500);
//     ledcWrite(pwmChannel1, 0);   // D2 at 0%
//     ledcWrite(pwmChannel2, 255); // D4 at 100%

//     delay(500);
//     ledcWrite(pwmChannel1, 255); // D2 at 100%
//     ledcWrite(pwmChannel2, 0);   // D4 at 0%
// }
