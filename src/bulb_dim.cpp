// #include <Arduino.h>
// // #include <
// #define LED_PIN 7          // GPIO pin where the LED is connected
// #define PWM_CHANNEL 0      // LEDC channel (0-15)
// #define PWM_FREQUENCY 5000 // Frequency in Hz
// #define PWM_RESOLUTION 8   // 8-bit resolution (0-255)

// void setup() {
//     // Configure PWM on the selected channel and pin
//     ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
//     ledcAttachPin(LED_PIN, PWM_CHANNEL);
// }

// void loop() {
//     // Gradually increase brightness
//     for (int duty = 0; duty <= 255; duty += 5) {
//         ledcWrite(PWM_CHANNEL, duty);
//         delay(20);  // Small delay for smooth dimming
//     }

//     // Gradually decrease brightness
//     for (int duty = 255; duty >= 0; duty -= 5) {
//         ledcWrite(PWM_CHANNEL, duty);
//         delay(20);
//     }
// }
