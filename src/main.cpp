#include <Arduino.h>
#include <PCF8575.h>
#include <PID_v1.h>
#include <Preferences.h>

// Define number of motors (for this example we use 9)
#define NUM_MOTORS 9
#define C_SPOOL .026*3.1416 //Spool circumference (m)

// Create a PCF8575 instance (I2C address 0x20)
PCF8575 pcf8575(0x20);

// Define PWM and encoder read pins for each motor
uint8_t pwm_pins[NUM_MOTORS] = {19, 18, 17, 16, 4, 2, 13, 12, 23};  
uint8_t read_pins[NUM_MOTORS] = {36, 39, 34, 35, 32, 33, 25, 26, 27};

//Define direction motor needs to go to tightent (0 is default, 1 is reverse)
uint8_t motor_directions[NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

double min_input = 255.0;
double max_input = 3805.0;
double offset_angle[NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
double raw_angle[NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
double filtered_pos[NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
double unfiltered_pos[NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
// MOTOR ORDER 0, 4, 5; 2, 8, 6; 1, 7, 3
// This correlates with top, left, and right for each section

// Define motor direction pins (two per motor) via PCF8575
// For a 9-motor system we need 18 direction outputs.
// Here we assume that the first 16 outputs are named P0..P15,
// and then we use two additional pins (14, 15) for the last motor.
uint8_t motor_direction_pins[NUM_MOTORS * 2] = {
  P1, P0,  // Motor 0
  P2, P3,  // Motor 1
  P5, P4,  // Motor 2
  P6, P7,  // Motor 3
  P9, P8,  // Motor 4
  P10, P11, // Motor 5
  P13, P12, // Motor 6
  P14, P15, // Motor 7
  15, 14    // Motor 8 (using additional pins)
};

// PID variables for each motor
double input[NUM_MOTORS], output[NUM_MOTORS], setpoint[NUM_MOTORS];
PID* pid[NUM_MOTORS];

// For encoder unwrapping: store last raw angle and rotation count per motor
double last_angle[NUM_MOTORS];
int rotations[NUM_MOTORS];

// Create Preferences instance (for non-volatile storage if needed)
Preferences preferences;
static unsigned long lastSaveTime = 0;

// For ESP32 LEDC PWM channels (one per motor)
uint8_t pwm_channels[NUM_MOTORS] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
const int pwmFreq = 500;
const int pwmResolution = 8;

// PID tuning parameters (adjust these per your system)
const double Kp = 5.0, Ki = 0.0005, Kd = 1.0;

double readAngle(int motor);
void updateRotation(int motor);
double getAbsolutePosition(int motor);
void driveMotor(int motor, double controlSignal);
double doubleMap(double input, double in1, double in2, double out1, double out2);
void nvmUpdate(int rotations);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting Multi-Motor PID Control");

  // Initialize PCF8575 (make sure your I2C pins are set correctly in Wire.begin() if needed)
  pcf8575.begin();

  // Initialize PWM pins via LEDC for each motor
  for (int i = 0; i < NUM_MOTORS; i++) {
    ledcSetup(pwm_channels[i], pwmFreq, pwmResolution);
    ledcAttachPin(pwm_pins[i], pwm_channels[i]);
  }
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  
  // Initialize PID controllers and set initial setpoints to the current measured positions.
  for (int i = 0; i < NUM_MOTORS; i++) {
    // Read the initial raw angle for motor i (from its analog encoder output)
    input[i] = readAngle(i);
    setpoint[i] = input[i];  // start at current position
    pid[i] = new PID(&input[i], &output[i], &setpoint[i], Kp, Ki, Kd, DIRECT);
    pid[i]->SetMode(AUTOMATIC);
    pid[i]->SetOutputLimits(-254,254);
  }
  
  // Initialize unwrapping variables for each motor
  for (int i = 0; i < NUM_MOTORS; i++) {
    offset_angle[i] = readAngle(i);
    last_angle[i] = offset_angle[i];
    filtered_pos[i] = last_angle[i];
    rotations[i] = 0;
  }
  
  // Set PCF8575 direction pins as outputs
  for (int i = 0; i < NUM_MOTORS * 2; i++) {
    pcf8575.pinMode(motor_direction_pins[i], OUTPUT);
  }

  // WHEN ALL HARDWARE IS PERFECTLY SETUP YOU CAN IMPLEMENT THIS... 
  // NVM IS ONLY RELIABLE UP TO 100,000 ITERATIONS SO LIMIT THIS USAGE
  // Load previous positions from non-volatile memory
//   preferences.begin("motor_positions", false);
//   for (int i = 0; i < NUM_MOTORS; i++) {
//     last_angle[i] = readAngle(i);
//     preferences.begin("motor_positions", false);
//     String key = "rot" + String(i);
//     rotations[i] = preferences.getInt(key.c_str(), 0);
//     preferences.end();
//     Serial.println("Motor " + String(i) + " loaded rotations: " + String(rotations[i]));
//   }
    delay(5000);
    Serial.println("Setup Complete");
  
}

void loop() {
  // Check for serial commands in the format: <motorIndex> <setpoint>, <motorIndex> <setpoint>
  // Example: "3 720, 4 540" sets motor 3's setpoint to 720° and motor 4's setpoint to 540°".
  if (Serial.available() > 0) {  // Only read if there is actual data
    String input = Serial.readStringUntil('\n');  // Read the entire input line
    input.trim();  // Remove any trailing newline or spaces

    int lastIndex = 0;
    while (lastIndex < input.length()) {
        int commaIndex = input.indexOf(',', lastIndex);
        if (commaIndex == -1) commaIndex = input.length();

        String command = input.substring(lastIndex, commaIndex);
        command.trim();

        int sepIndex = command.indexOf(' ');
        if (sepIndex != -1) {
            int motorIndex = command.substring(0, sepIndex).toInt();
            double sp = command.substring(sepIndex + 1).toFloat();

            if (motorIndex >= 0 && motorIndex < NUM_MOTORS) {
              if (motor_directions[motorIndex] == 0){
                setpoint[motorIndex] = sp * C_SPOOL / 360.0;
              }
              else {
                setpoint[motorIndex] = -sp * C_SPOOL / 360.0;
              }
                Serial.print("Motor ");
                Serial.print(motorIndex);
                Serial.print(" new setpoint: ");
                Serial.println(sp);
            }
            if (motorIndex == 10){
              rotations[NUM_MOTORS] = {0};
              for (int i = 0; i < NUM_MOTORS; i++) {
                offset_angle[i] = raw_angle[i];
              }
            }
        }

        lastIndex = commaIndex + 1;  // Move to next command
    }
}

  
  // Update each motor’s PID loop
  for (int i = 0; i < NUM_MOTORS; i++) {
    // Update the unwrapped angle measurement
    updateRotation(i);
    
    // Calculate the absolute position (angle plus rotations)
    unfiltered_pos[i] = getAbsolutePosition(i);
    filtered_pos[i] = filtered_pos[i]*0.9 + unfiltered_pos[i]*0.1;
    input[i] = filtered_pos[i];
    
    // Compute the PID control output for this motor
    pid[i]->Compute();
    
    // Drive the motor based on PID output
    driveMotor(i, output[i]);
    
    // Optionally print debug information
    // Serial.print("Motor ");
    Serial.print(i);
    Serial.print("P:");
    Serial.print(int(input[i]));
    Serial.print(" S");
    Serial.print(int(setpoint[i]));
    Serial.print(" O");
    Serial.print(int(output[i]));
    Serial.print(" ");

  }
  Serial.print("       \r"); // Return to the beginning of the line
    //   // UNCOMMENT WHEN READY TO USE NVM
//   if (millis() >= lastSaveTime + 10000){
//     nvmUpdate();
//     lastSaveTime = millis();
//   }

  delay(10);
}

// Read the raw angle (0-360°) from the encoder for motor 'motor'
double readAngle(int motor) {
  double angle = 0;
  int raw = analogRead(read_pins[motor]);
  angle = doubleMap(double(raw), min_input, max_input, 0.0, 360.0);
  return angle;
}

// Update the rotation count (unwrap the encoder reading) for motor 'motor'
void updateRotation(int motor) {
  raw_angle[motor] = readAngle(motor);
  double diff = raw_angle[motor] - last_angle[motor];
  // When the change is large (more than half a rotation) we assume a wrap-around.
  if (diff > 180) {
    rotations[motor]--;
  } else if (diff < -180) {
    rotations[motor]++;
  }
  last_angle[motor] = raw_angle[motor];
}

// Get the absolute (unwrapped) position for motor 'motor'
double getAbsolutePosition(int motor) {
  return rotations[motor] * 360.0 + raw_angle[motor];
}

// Drive the motor for index 'motor' using the PID control output.
// The controlSignal is used to determine both the speed (PWM value) and direction.
void driveMotor(int motor, double controlSignal) {
  // Calculate the PWM duty-cycle (absolute value, constrained to 0-255)
  int pwmValue = abs(controlSignal);
  pwmValue = constrain(pwmValue, 0, 255);
  
  // Determine the two PCF8575 pins for motor 'motor'
  // We assume motor i uses motor_direction_pins[2*i] and motor_direction_pins[2*i + 1]
  uint8_t dirPin1 = motor_direction_pins[2 * motor];
  uint8_t dirPin2 = motor_direction_pins[2 * motor + 1];
  
  // Set direction: positive controlSignal drives one way, negative the opposite.
    if (motor == 8){
        if (controlSignal >= 0) {
            digitalWrite(dirPin1, HIGH);
            digitalWrite(dirPin2, LOW);
        } 
        else {
            digitalWrite(dirPin1, LOW);
            digitalWrite(dirPin2, HIGH);
            }
        }
    else{
        if (controlSignal >= 0) {
            pcf8575.digitalWrite(dirPin1, HIGH);
            pcf8575.digitalWrite(dirPin2, LOW);
            } 
        else {
            pcf8575.digitalWrite(dirPin1, LOW);
            pcf8575.digitalWrite(dirPin2, HIGH);
            }
    }
  
  // Output the PWM signal to the motor driver via the corresponding LEDC channel.
  ledcWrite(pwm_channels[motor], pwmValue);
}

double doubleMap(double input, double in1, double in2, double out1, double out2){
    return input/(in2-in1)*(out2-out1)+out1;
}

// Save the rotations to nvm
void nvmUpdate() {
    preferences.begin("motor_positions", false);
    for (int i = 0; i < NUM_MOTORS; i++) {
      String key = "rot" + String(i);
      preferences.putInt(key.c_str(), rotations[i]);
    }
    preferences.end();
    Serial.println("NVM Updated with current rotations.");
  }
  