// #include "Arduino.h"
// #include "PCF8575.h"
// #include <PID_v1.h>
// #include <Preferences.h>

// #define PWM_FREQUENCY 5000 // Frequency in Hz
// #define PWM_RESOLUTION 8   // 8-bit resolution (0-255)

// #define NUM_MOTORS 9       // Number of motors

// // Set i2c address
// PCF8575 pcf8575(0x20);

// // Declare variables
// uint8_t pwm_pins[] = {19, 18, 17, 16, 2, 4, 13, 12, 23};  
// int num_pins = sizeof(pwm_pins)/ sizeof(pwm_pins[0]);

// uint8_t read_pins[] = {36, 39, 34, 35, 32, 33, 25, 26, 27};  
// int num_read_pins = sizeof(read_pins)/ sizeof(read_pins[0]);

// // PID variables
// double input[NUM_MOTORS], output[NUM_MOTORS], setpoint[NUM_MOTORS];
// PID* pid[NUM_MOTORS];
// // Non-volatile memory
// Preferences preferences;

// // Define motor direction pins (PCF8575 pins)
// uint8_t motor_direction_pins[] = {P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15, 14, 15};

// // Setup
// void setup() {
//   Serial.begin(115200);

//   preferences.begin("storage", false);

//   // Initialize PCF8575
//   pcf8575.begin();

//   // Set motor direction pins to OUTPUT
//   for (int i = 0; i < NUM_MOTORS*2; i++) {
//     pcf8575.pinMode(motor_direction_pins[i], OUTPUT);
//   }
//   pinMode(14, OUTPUT);
//   pinMode(15, OUTPUT);

//   // Initialize PWM pins
//   for (int i = 0; i < num_pins; i++) {
//     ledcSetup(i, PWM_FREQUENCY, PWM_RESOLUTION);
//     ledcAttachPin(pwm_pins[i], i);
//   }

//   // Initialize position read pins
//   for (int i = 0; i < num_read_pins; i++) {
//     pinMode(read_pins[i], INPUT);
//   }

//   // Set up PID controllers for each motor
//   for (int i = 0; i < NUM_MOTORS; i++) {
//     setpoint[i] = 180;  // Example target position (could be input by the user)
//     pid[i] = new PID(&input[i], &output[i], &setpoint[i], 2.0, 5.0, 1.0, DIRECT);  // Initialize each PID object
//     pid[i]->SetMode(AUTOMATIC);
//   }

//   // Load previous positions from non-volatile memory
//   preferences.begin("motor_positions", false);
//   for (int i = 0; i < NUM_MOTORS; i++) {
//     String motorKey = "motor" + String(i); // Create key as String
//     int last_position = preferences.getInt(motorKey.c_str(), 0); // Use .c_str() to convert to const char*
//     input[i] = last_position;
//   }
//   preferences.end();
  
// }

// // Main loop
// void loop() {
//   // Read positions from analog pins
//   for (int i = 0; i < NUM_MOTORS; i++) {
//     input[i] = analogRead(read_pins[i]); // Read position (0-1023)
//     Serial.print("Sensor ");
//     Serial.print(i);
//     Serial.print(": ");
//     Serial.println(input[i]);
//   }

//   // delay(1000);
//   // Compute motor control output using PID
//   for (int i = 0; i < NUM_MOTORS; i++) {
//     pid[i]->Compute();

//     // Set PWM value (speed control)
//     ledcWrite(i, output[i]);
//     // Set motor direction
//     if (setpoint[i] > input[i]) {

//       if (i < 8){ 
//         pcf8575.digitalWrite(motor_direction_pins[i*2], LOW); // Forward direction
//         pcf8575.digitalWrite(motor_direction_pins[i*2+1], HIGH);
//         } 
//       else{
//         digitalWrite(motor_direction_pins[i*2], LOW);
//         digitalWrite(motor_direction_pins[i*2+1], HIGH);
//         }
//       }


//     else {
//       if (i < 8){ 
//         pcf8575.digitalWrite(motor_direction_pins[i*2], LOW); // Forward direction
//         pcf8575.digitalWrite(motor_direction_pins[i*2+1], HIGH);
//         } 
//       else{
//         digitalWrite(motor_direction_pins[i*2], LOW);
//         digitalWrite(motor_direction_pins[i*2+1], HIGH);
//         }
//       }
//   }

//   // Save current position every 8 seconds
//   static unsigned long lastSaveTime = 0;
//   if (millis() - lastSaveTime >= 8000) {
//     lastSaveTime = millis();
    
//     preferences.begin("motor_positions", false);
//     for (int i = 0; i < NUM_MOTORS; i++) {
//         String motorKey = "motor" + String(i); // Create key as String
//         preferences.putInt(motorKey.c_str(), (int)input[i]);

//     }
//     preferences.end();
//   }

//   delay(10); // Small delay to allow PID control to stabilize
// }
