#include <Arduino.h>
#include <PCF8575.h>
#include <PID_v1.h>
#include <Preferences.h>

// Define number of motors (for this example we use 9)
#define NUM_MOTORS 9

// Create a PCF8575 instance (I2C address 0x20)
PCF8575 pcf8575(0x20);

// Define PWM and encoder read pins for each motor
uint8_t pwm_pins[NUM_MOTORS] = {19, 18, 17, 16, 2, 4, 13, 12, 23};  
uint8_t read_pins[NUM_MOTORS] = {36, 39, 34, 35, 32, 33, 25, 26, 27};

double min_input = 255.0;
double max_input = 3805.0;

// Define motor direction pins (two per motor) via PCF8575
// For a 9-motor system we need 18 direction outputs.
// Here we assume that the first 16 outputs are named P0..P15,
// and then we use two additional pins (14, 15) for the last motor.
uint8_t motor_direction_pins[NUM_MOTORS * 2] = {
  P0, P1,  // Motor 0
  P2, P3,  // Motor 1
  P4, P5,  // Motor 2
  P6, P7,  // Motor 3
  P8, P9,  // Motor 4
  P10, P11, // Motor 5
  P12, P13, // Motor 6
  P14, P15, // Motor 7
  14, 15    // Motor 8 (using additional pins)
};

// PID variables for each motor
double input[NUM_MOTORS], output[NUM_MOTORS], setpoint[NUM_MOTORS];
PID* pid[NUM_MOTORS];

// For encoder unwrapping: store last raw angle and rotation count per motor
double last_angle[NUM_MOTORS];
long rotations[NUM_MOTORS];

// Create Preferences instance (for non-volatile storage if needed)
Preferences preferences;

// For ESP32 LEDC PWM channels (one per motor)
uint8_t pwm_channels[NUM_MOTORS] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
const int pwmFreq = 500;
const int pwmResolution = 8;

// PID tuning parameters (adjust these per your system)
const double Kp = 2.0, Ki = 0.0, Kd = 1.0;

double readAngle(int motor);
void updateRotation(int motor);
double getAbsolutePosition(int motor);
void driveMotor(int motor, double controlSignal);
double doubleMap(double input, double in1, double in2, double out1, double out2);

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
  
  // Initialize PID controllers and set initial setpoints to the current measured positions.
  for (int i = 0; i < NUM_MOTORS; i++) {
    // Read the initial raw angle for motor i (from its analog encoder output)
    input[i] = readAngle(i);
    setpoint[i] = input[i];  // start at current position
    pid[i] = new PID(&input[i], &output[i], &setpoint[i], Kp, Ki, Kd, DIRECT);
    pid[i]->SetMode(AUTOMATIC);
    pid[i]->SetOutputLimits(-255,255);
  }
  
  // Initialize unwrapping variables for each motor
  for (int i = 0; i < NUM_MOTORS; i++) {
    last_angle[i] = readAngle(i);
    rotations[i] = 0;
  }
  
  // Set PCF8575 direction pins as outputs
  for (int i = 0; i < NUM_MOTORS * 2; i++) {
    pcf8575.pinMode(motor_direction_pins[i], OUTPUT);
  }
}

void loop() {
  // Check for serial commands in the format: <motorIndex> <setpoint>
  // Example: "3 720" sets motor 3's setpoint to 720°.
  if (Serial.available() > 0) {  // Only read if there is actual data
    int motorIndex = Serial.parseInt();
    double sp = Serial.parseFloat();
    if (motorIndex >= 0 && motorIndex < NUM_MOTORS) {
        setpoint[motorIndex] = sp;
        Serial.print("Motor ");
        Serial.print(motorIndex);
        Serial.print(" new setpoint: ");
        Serial.println(sp);
    }
}

  
  // Update each motor’s PID loop
  for (int i = 0; i < NUM_MOTORS; i++) {
    // Update the unwrapped angle measurement
    updateRotation(i);
    
    // Calculate the absolute position (angle plus rotations)
    input[i] = getAbsolutePosition(i);
    
    // Compute the PID control output for this motor
    pid[i]->Compute();
    
    // Drive the motor based on PID output
    driveMotor(i, output[i]);
    
    // Optionally print debug information
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(" | Pos: ");
    Serial.print(input[i]);
    Serial.print(" | Set: ");
    Serial.print(setpoint[i]);
    Serial.print(" | Out: ");
    Serial.println(output[i]);
  }
  
  delay(10);
}

// Read the raw angle (0-360°) from the encoder for motor 'motor'
// This example assumes an analog output from the encoder connected to an ADC pin.
// Adjust the conversion if your encoder output range or ADC resolution is different.
double readAngle(int motor) {
  int raw = analogRead(read_pins[motor]);
  double angle = doubleMap(double(raw), min_input, max_input, 0.0, 360.0);
  return angle;
}

// Update the rotation count (unwrap the encoder reading) for motor 'motor'
void updateRotation(int motor) {
  double currentAngle = readAngle(motor);
  double diff = currentAngle - last_angle[motor];
  // When the change is large (more than half a rotation) we assume a wrap-around.
  if (diff > 180) {
    rotations[motor]--;
  } else if (diff < -180) {
    rotations[motor]++;
  }
  last_angle[motor] = currentAngle;
}

// Get the absolute (unwrapped) position for motor 'motor'
double getAbsolutePosition(int motor) {
  return rotations[motor] * 360.0 + readAngle(motor);
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
  if (controlSignal >= 0) {
    pcf8575.digitalWrite(dirPin1, HIGH);
    pcf8575.digitalWrite(dirPin2, LOW);
  } else {
    pcf8575.digitalWrite(dirPin1, LOW);
    pcf8575.digitalWrite(dirPin2, HIGH);
  }
  
  // Output the PWM signal to the motor driver via the corresponding LEDC channel.
  ledcWrite(pwm_channels[motor], pwmValue);
}

double doubleMap(double input, double in1, double in2, double out1, double out2){
    return input/(in2-in1)*(out2-out1)+out1;
}