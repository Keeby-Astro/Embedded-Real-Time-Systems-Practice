#include <quaternionFilters.h>  // Include quaternion filter library for IMU processing
#include <MPU9250.h>            // Include MPU9250 IMU library
#include <PWMServo.h>           // Include PWMServo library for controlling brushless motor
#include "project.h"            // Include custom header file for enumerations and configurations
#include "PeriodicAction.h"     // Include library for scheduling periodic actions
#include <ImuUtils.h>           // Include IMU utility functions

#define switch_pin 0            // Pin connected to the control switch
#define m1_clockwise 1          // Motor 1 clockwise control pin
#define m1_c_clockwise 2        // Motor 1 counter-clockwise control pin
#define m1_pulsewidth 3         // Motor 1 PWM speed control pin
#define m2_clockwise 22         // Motor 2 clockwise control pin
#define m2_c_clockwise 23       // Motor 2 counter-clockwise control pin
#define m2_pulsewidth 21        // Motor 2 PWM speed control pin
#define m3_clockwise 7          // Motor 3 clockwise control pin
#define m3_c_clockwise 8        // Motor 3 counter-clockwise control pin
#define m3_pulsewidth 9         // Motor 3 PWM speed control pin
#define SDA_0 18                // I2C Data pin for IMU
#define SCL_0 19                // I2C Clock pin for IMU

PWMServo fan;                  // Create PWMServo object to control the central fan
const int CENTRAL_FAN_PWM = 36; // Pin for controlling the central fan

/*
 * Initializes the Brushless lift motor (fan) with a low throttle command.
 */
void fan_setup() {
  fan.attach(CENTRAL_FAN_PWM); // Attach the fan to the specified PWM pin
  delay(100);
  fan.write(20);               // Send a low throttle signal for initialization
  delay(3000);                 // Wait for motor to stabilize
}

/*
 * Sets up all pins as inputs/outputs, initializes IMU, and starts serial communication.
 */
void setup() {
  fan_setup();                 // Set up the brushless motor power
  pinMode(switch_pin, INPUT);  // Set the switch pin as input

  // Initialize motor 1 pins
  pinMode(m1_clockwise, OUTPUT);
  pinMode(m1_c_clockwise, OUTPUT);
  pinMode(m1_pulsewidth, OUTPUT);
  digitalWrite(m1_clockwise, LOW);
  digitalWrite(m1_c_clockwise, LOW);
  digitalWrite(m1_pulsewidth, LOW);

  // Initialize motor 2 pins
  pinMode(m2_clockwise, OUTPUT);
  pinMode(m2_c_clockwise, OUTPUT);
  pinMode(m2_pulsewidth, OUTPUT);
  digitalWrite(m2_clockwise, LOW);
  digitalWrite(m2_c_clockwise, LOW);
  digitalWrite(m2_pulsewidth, LOW);

  // Initialize motor 3 pins
  pinMode(m3_clockwise, OUTPUT);
  pinMode(m3_c_clockwise, OUTPUT);
  pinMode(m3_pulsewidth, OUTPUT);
  digitalWrite(m3_clockwise, LOW);
  digitalWrite(m3_c_clockwise, LOW);
  digitalWrite(m3_pulsewidth, LOW);

  Serial.begin(115200);        // Begin serial communication at 115200 baud
  Serial.setTimeout(100000000); // Set a long timeout for serial communication
  imu_setup();                 // Initialize the IMU sensor
  delay(1000);                 // Delay for setup stability
}

/*
 * Ensures that a given value is within a specified range.
 */
float bound(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;          // Enforce minimum value
  } else if (value > max_value) {
    return max_value;          // Enforce maximum value
  } else {
    return value;              // Return the original value if within bounds
  }
}

/*
 * Configures the motors' directions and speeds based on input power values.
 */
void set_motors(float val[3]) {
  const float negative_gain[3] = {1.1, 1.1, 1.2}; // Gain for negative values
  const float positive_gain[3] = {1.0, 1.0, 1.1}; // Gain for positive values

  for (int i = 0; i <= 2; ++i) {
    int motorClockwise = 0;        // Variable for clockwise control pin
    int motor_C_Clockwise = 0;     // Variable for counter-clockwise control pin
    int motorPulse = 0;            // Variable for PWM pin

    // Assign motor pins based on the index
    if (i == 0) { // Motor 1 (left motor)
      motorClockwise = m1_clockwise;
      motor_C_Clockwise = m1_c_clockwise;
      motorPulse = m1_pulsewidth;
    } else if (i == 1) { // Motor 2 (right motor)
      motorClockwise = m2_clockwise;
      motor_C_Clockwise = m2_c_clockwise;
      motorPulse = m2_pulsewidth;
    } else if (i == 2) { // Motor 3 (back motor)
      motorClockwise = m3_clockwise;
      motor_C_Clockwise = m3_c_clockwise;
      motorPulse = m3_pulsewidth;
    }

    // Configure motor direction and power
    if (val[i] > 0) { // Positive power for clockwise spin
      val[i] *= positive_gain[i];
      digitalWrite(motorClockwise, LOW);
      digitalWrite(motor_C_Clockwise, HIGH);
    } else if (val[i] < 0) { // Negative power for counter-clockwise spin
      val[i] *= negative_gain[i];
      digitalWrite(motorClockwise, HIGH);
      digitalWrite(motor_C_Clockwise, LOW);
    } else { // Stop the motor
      digitalWrite(motorClockwise, LOW);
      digitalWrite(motor_C_Clockwise, LOW);
    }

    // Write the power to the motor PWM pin (bounded within range)
    analogWrite(motorPulse, abs(bound(val[i], -255, 255)));
    Serial.printf("Power %f Motor %d \n", val[i], motorPulse);
  }
}

/*
 * Converts an angle from degrees to radians.
 */
float angle_radians(int deg) {
  return (((float)deg) * (PI / 180.0)); // Convert degrees to radians
}

/*
 * Calculates motor forces based on desired force and torque inputs.
 */
void set_hovercraft_forces(float fx, float fy, float torque) {
  float F1, F2, F3;
  float R = 5.5; // Effective radius of the hovercraft in inches

  // Calculate motor forces using force and torque equations
  F1 = ((1 / (2 * cos(angle_radians(30)))) * fx) +
       ((-1 / (2 + (2 * sin(angle_radians(30))))) * fy) +
       ((-1 / (2 * R * (1 + (sin(angle_radians(30)))))) * torque);

  F2 = ((1 / (2 * cos(angle_radians(30)))) * fx) +
       ((1 / (2 + (2 * sin(angle_radians(30))))) * fy) +
       ((1 / (2 * R * (1 + (sin(angle_radians(30)))))) * torque);

  F3 = ((-1 / (1 + (1 * sin(angle_radians(30))))) * fy) +
       ((sin(angle_radians(30)) / (R * (1 + (sin(angle_radians(30)))))) * torque);

  float forces[] = {F1, F2, F3}; // Array of calculated forces
  set_motors(forces);            // Set the motor power based on forces
}

/*
 * Implements the finite state machine (FSM) for hovercraft control.
 */
void fsm_step() {
  // FSM implementation with states and transitions
}

/*
 * Uses the IMU to counter rotation around the Z-axis.
 */
void pd_step() {
  float Kd = 0.2;              // Proportional-derivative gain value
  float fx = 0;
  float fy = 0;
  float torque = Kd * (-IMU.gz); // Calculate torque to counter rotation
  set_hovercraft_forces(fx, fy, torque);
}

/*
 * Reports the current rotation rate around the Z-axis.
 */
void report_step() {
  Serial.printf("Rotation Rate: %f \n", IMU.gz);
}

/*
 * Main loop: Repeatedly calls scheduled tasks.
 */
void loop() {
  static PeriodicAction fsm_task(50, fsm_step);      // FSM step every 50ms
  static PeriodicAction imu_task(5, imu_update);    // Update IMU every 5ms
  static PeriodicAction pd_task(10, pd_step);       // Perform PD control every 10ms
  static PeriodicAction report_task(1000, report_step); // Report rotation every 1s

  fsm_task.step();
}
