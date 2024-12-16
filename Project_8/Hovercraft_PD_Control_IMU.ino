#include <quaternionFilters.h>  // Include quaternion filters for IMU processing
#include <MPU9250.h>            // Include MPU9250 IMU library
#include <PWMServo.h>           // Include PWMServo library for controlling brushless motor
#include "project.h"            // Include custom project definitions
#include "PeriodicAction.h"     // Include library for scheduling periodic actions
#include <ImuUtils.h>           // Include utility functions for IMU

// Define pin aliases for motors and I2C communication
#define switch_pin 0            // Pin connected to the control switch
#define m1_clockwise 23         // Motor 1 clockwise control pin
#define m1_c_clockwise 22       // Motor 1 counter-clockwise control pin
#define m1_pulsewidth 21        // Motor 1 PWM speed control pin
#define m2_clockwise 1          // Motor 2 clockwise control pin
#define m2_c_clockwise 2        // Motor 2 counter-clockwise control pin
#define m2_pulsewidth 3         // Motor 2 PWM speed control pin
#define m3_clockwise 8          // Motor 3 clockwise control pin
#define m3_c_clockwise 7        // Motor 3 counter-clockwise control pin
#define m3_pulsewidth 9         // Motor 3 PWM speed control pin
#define SDA_0 18                // I2C Data pin for IMU
#define SCL_0 19                // I2C Clock pin for IMU

PWMServo fan;                  // Create PWMServo object to control the central fan
const int CENTRAL_FAN_PWM = 36; // Pin for controlling the central fan

/*
 * Function: fan_setup
 * Initializes the brushless lift motor with a low throttle command for safety.
 */
void fan_setup() {
  fan.attach(CENTRAL_FAN_PWM);  // Attach the fan to the specified PWM pin
  delay(100);
  fan.write(20);                // Write low throttle signal for initialization
  delay(3000);                  // Wait for motor to stabilize
}

/*
 * Function: setup
 * Initializes all pins as input/output, starts the IMU, and begins the serial monitor.
 */
void setup() {
  fan_setup();                  // Set up the central lift motor
  pinMode(switch_pin, INPUT);   // Set the switch pin as input

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

  Serial.begin(115200);         // Begin serial communication at 115200 baud
  Serial.setTimeout(100000000); // Set a long timeout for serial communication
  imu_setup();                  // Initialize the IMU sensor
  delay(1000);                  // Delay for setup stabilization
}

/*
 * Function: bound
 * Ensures a value is within a specified range.
 */
float bound(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;           // Enforce minimum bound
  } else if (value > max_value) {
    return max_value;           // Enforce maximum bound
  } else {
    return value;               // Return the value if within bounds
  }
}

/*
 * Function: set_motors
 * Configures motor directions and speeds based on input power values.
 */
void set_motors(float val[3]) {
  const float negative_gain[3] = {2.2, 2.2, 2.2}; // Gain for negative values
  const float positive_gain[3] = {1.2, 1.2, 1.1}; // Gain for positive values

  for (int i = 0; i <= 2; ++i) {
    int motorClockwise = 0;     // Variable for clockwise control pin
    int motor_C_Clockwise = 0;  // Variable for counter-clockwise control pin
    int motorPulse = 0;         // Variable for PWM pin

    // Assign motor pins based on the index
    if (i == 0) {
      motorClockwise = m1_clockwise;
      motor_C_Clockwise = m1_c_clockwise;
      motorPulse = m1_pulsewidth;
    } else if (i == 1) {
      motorClockwise = m2_clockwise;
      motor_C_Clockwise = m2_c_clockwise;
      motorPulse = m2_pulsewidth;
    } else if (i == 2) {
      motorClockwise = m3_clockwise;
      motor_C_Clockwise = m3_c_clockwise;
      motorPulse = m3_pulsewidth;
    }

    // Set motor direction and apply power
    if (val[i] > 0) { // Clockwise
      val[i] *= positive_gain[i];
      digitalWrite(motorClockwise, LOW);
      digitalWrite(motor_C_Clockwise, HIGH);
    } else if (val[i] < 0) { // Counter-clockwise
      val[i] *= negative_gain[i];
      digitalWrite(motorClockwise, HIGH);
      digitalWrite(motor_C_Clockwise, LOW);
    } else { // Stop motor
      digitalWrite(motorClockwise, LOW);
      digitalWrite(motor_C_Clockwise, LOW);
    }

    analogWrite(motorPulse, abs(bound(val[i], -255, 255))); // Apply power with bounds
  }
}

/*
 * Function: angle_radians
 * Converts an angle from degrees to radians.
 */
float angle_radians(int deg) {
  return (((float)deg) * (PI / 180.0));
}

/*
 * Function: set_hovercraft_forces
 * Calculates and sets motor forces based on input forces and torque.
 */
void set_hovercraft_forces(float fx, float fy, float torque) {
  float F1, F2, F3;
  float R = 5.5; // Radius of hovercraft in inches

  // Force calculations based on geometry and physics
  F1 = ((1 / (2 * cos(angle_radians(30)))) * fx) +
       ((-1 / (2 + (2 * sin(angle_radians(30))))) * fy) +
       ((-1 / (2 * R * (1 + sin(angle_radians(30))))) * torque);

  F2 = ((1 / (2 * cos(angle_radians(30)))) * fx) +
       ((1 / (2 + (2 * sin(angle_radians(30))))) * fy) +
       ((1 / (2 * R * (1 + sin(angle_radians(30))))) * torque);

  F3 = ((-1 / (1 + sin(angle_radians(30)))) * fy) +
       ((sin(angle_radians(30)) / (R * (1 + sin(angle_radians(30))))) * torque);

  float forces[] = {F1, F2, F3};
  set_motors(forces);           // Apply the calculated forces to motors
}

/*
 * Function: compute_error
 * Computes the angular error between the current heading and the goal.
 */
float compute_error(float theta, float heading_goal) {
  float diff = heading_goal - theta;
  if (diff >= 0 && diff <= 180) {
    return diff;
  } else {
    return diff - 360;
  }
}

/*
 * Function: db_clip
 * Applies a deadband and saturation to the error.
 */
float db_clip(float error, float deadband, float saturation) {
  if (error <= deadband && error >= -deadband) {
    return 0.0; // Error within the deadband
  } else if (abs(error) >= saturation) {
    return saturation - deadband; // Saturated value
  } else {
    return (error / saturation) * (saturation - deadband); // Scaled error
  }
}

/*
 * Function: pd_step
 * Implements proportional-derivative control to counter IMU-measured rotation.
 */
void pd_step() {
  float Kd = 4; // Gain value
  float fx = 0;
  float fy = 0;
  float theta = 0;
  float rotation = 0;
  float heading_error = compute_error(theta, rotation);
  float modified_error = db_clip(heading_error, 15, 45);
  float torque = Kd * modified_error; // Compute corrective torque
  set_hovercraft_forces(fx, fy, torque);
}

/*
 * Function: report_step
 * Prints the current rotation rate about the Z-axis.
 */
void report_step() {
  Serial.printf("Rotation Rate: %f \n", IMU.gz);
}

/*
 * Main loop: Executes periodic tasks for the hovercraft.
 */
void loop() {
  static PeriodicAction fsm_task(50, fsm_step);      // FSM step every 50ms
  static PeriodicAction imu_task(5, imu_update);    // IMU update every 5ms
  static PeriodicAction pd_task(10, pd_step);       // PD control every 10ms
  static PeriodicAction report_task(1000, report_step); // Reporting every 1s

  imu_task.step();
  pd_task.step();
  fsm_task.step();
  report_task.step();
}
