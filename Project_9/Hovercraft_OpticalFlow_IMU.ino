#include <quaternionFilters.h>  // Library for quaternion filtering for IMU
#include <MPU9250.h>            // Library for MPU9250 IMU
#include <PWMServo.h>           // Library for controlling the central fan (brushless motor)
#include "project.h"            // Project-specific enumerations and configurations
#include "PeriodicAction.h"     // Library for scheduling periodic actions
#include <ImuUtils.h>           // Utility functions for IMU processing
#include <OpticalFlowCamera.h>  // Library for interfacing with optical flow cameras

// Define pin mappings for motors, cameras, and other peripherals
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
#define SDA_0 18                // I2C Data pin
#define SCL_0 19                // I2C Clock pin
#define pin_MISO 12             // MISO pin for SPI communication
#define pin_SCL 13              // SCL pin for SPI communication
#define pin_MOSI 11             // MOSI pin for SPI communication
#define cam_1 15                // Camera 1 control pin
#define cam_2 16                // Camera 2 control pin
#define cam_3 17                // Camera 3 control pin
#define RESET 14                // Reset pin for cameras

// Global variables
static boolean motor_on = false;         // Flag to track if motors are on
const uint8_t CAMERA_SELECT[3] = {cam_1, cam_2, cam_3}; // Array to hold camera selection pins
OpticalFlowCamera cams(RESET);           // OpticalFlowCamera object
int32_t adx[3] = {0, 0, 0};              // Array to hold cumulative x-slip values for each camera
int32_t ady[3] = {0, 0, 0};              // Array to hold cumulative y-slip values for each camera
PWMServo fan;                            // PWMServo object to control the central lift fan
const int CENTRAL_FAN_PWM = 36;          // Pin for controlling the central fan

/*
 * Function: fan_setup
 * Input: none
 * Output: none
 * Initializes the central lift fan with low throttle for safety.
 */
void fan_setup() {
  fan.attach(CENTRAL_FAN_PWM);           // Attach the fan to the specified PWM pin
  delay(100);
  fan.write(20);                         // Set fan to low throttle
  delay(3000);                           // Wait for fan to stabilize
}

/*
 * Function: camera_setup
 * Input: none
 * Output: none
 * Initializes all optical flow cameras. If a camera fails to initialize,
 * the program enters a loop until all cameras are functional.
 */
void camera_setup() {
  int ret_1 = -1, ret_2 = -1, ret_3 = -1;
  do {                                   // Retry initializing cameras until all succeed
    ret_1 = cams.addCamera(CAMERA_SELECT[0]);
    ret_2 = cams.addCamera(CAMERA_SELECT[1]);
    ret_3 = cams.addCamera(CAMERA_SELECT[2]);
    if (ret_1 != 0) Serial.printf("Camera 1 Down \n");
    if (ret_2 != 0) Serial.printf("Camera 2 Down \n");
    if (ret_3 != 0) Serial.printf("Camera 3 Down \n");
  } while (ret_1 != 0 || ret_2 != 0 || ret_3 != 0);
}

/*
 * Function: setup
 * Input: none
 * Output: none
 * Sets up all motors, cameras, IMU, and peripherals.
 */
void setup() {
  fan_setup();                           // Initialize the lift fan
  camera_setup();                        // Initialize cameras
  pinMode(switch_pin, INPUT);            // Set control switch as input

  // Initialize motor pins
  pinMode(m1_clockwise, OUTPUT);
  pinMode(m1_c_clockwise, OUTPUT);
  pinMode(m1_pulsewidth, OUTPUT);
  pinMode(m2_clockwise, OUTPUT);
  pinMode(m2_c_clockwise, OUTPUT);
  pinMode(m2_pulsewidth, OUTPUT);
  pinMode(m3_clockwise, OUTPUT);
  pinMode(m3_c_clockwise, OUTPUT);
  pinMode(m3_pulsewidth, OUTPUT);

  // Set all motor outputs to LOW
  digitalWrite(m1_clockwise, LOW);
  digitalWrite(m1_c_clockwise, LOW);
  digitalWrite(m1_pulsewidth, LOW);
  digitalWrite(m2_clockwise, LOW);
  digitalWrite(m2_c_clockwise, LOW);
  digitalWrite(m2_pulsewidth, LOW);
  digitalWrite(m3_clockwise, LOW);
  digitalWrite(m3_c_clockwise, LOW);
  digitalWrite(m3_pulsewidth, LOW);

  Serial.begin(115200);                  // Start serial communication
  Serial.setTimeout(100000000);          // Set a long timeout for serial communication
  imu_setup();                           // Initialize the IMU
  IMU.magbias[0] = 145.327698;           // Apply IMU calibration biases
  IMU.magbias[1] = -69.055908;
  IMU.magbias[2] = 395.881012;
  delay(1000);                           // Delay for setup stabilization
}

/*
 * Function: bound
 * Input: value (float), min_value (float), max_value (float)
 * Output: float - Returns the bounded value between min and max.
 * Ensures the value remains within specified bounds.
 */
float bound(float value, float min_value, float max_value) {
  if (value < min_value) return min_value; // If value is less than minimum, return min_value
  if (value > max_value) return max_value; // If value is greater than maximum, return max_value
  return value;                            // Otherwise, return the original value
}

/*
 * Function: set_motors
 * Input: val[3] - Array of desired motor power values for left, right, and back motors
 * Output: none
 * Configures motor directions and sets motor power based on input values.
 */
void set_motors(float val[3]) {
  const float negative_gain[3] = {2.2, 2.2, 2.2}; // Gain for negative motor values
  const float positive_gain[3] = {1.2, 1.2, 1.1}; // Gain for positive motor values

  for (int i = 0; i <= 2; ++i) {
    // Determine motor pins based on motor index
    int motorClockwise = 0, motor_C_Clockwise = 0, motorPulse = 0;
    if (i == 0) { // Motor 1 (left)
      motorClockwise = m1_clockwise;
      motor_C_Clockwise = m1_c_clockwise;
      motorPulse = m1_pulsewidth;
    } else if (i == 1) { // Motor 2 (right)
      motorClockwise = m2_clockwise;
      motor_C_Clockwise = m2_c_clockwise;
      motorPulse = m2_pulsewidth;
    } else if (i == 2) { // Motor 3 (back)
      motorClockwise = m3_clockwise;
      motor_C_Clockwise = m3_c_clockwise;
      motorPulse = m3_pulsewidth;
    }

    // Determine motor direction and apply gain
    if (val[i] > 0) { // Positive power -> clockwise
      val[i] = val[i] * positive_gain[i];
      digitalWrite(motorClockwise, LOW);
      digitalWrite(motor_C_Clockwise, HIGH);
    } else if (val[i] < 0) { // Negative power -> counterclockwise
      val[i] = val[i] * negative_gain[i];
      digitalWrite(motorClockwise, HIGH);
      digitalWrite(motor_C_Clockwise, LOW);
    } else { // Zero power -> stop motor
      digitalWrite(motorClockwise, LOW);
      digitalWrite(motor_C_Clockwise, LOW);
    }

    // Set motor power using PWM and enforce bounds
    analogWrite(motorPulse, abs(bound(val[i], -255, 255)));
  }
}

/*
 * Function: angle_radians
 * Input: deg (int) - Angle in degrees
 * Output: float - Converted angle in radians
 * Converts an angle from degrees to radians.
 */
float angle_radians(int deg) {
  return (((float)deg) * (PI / 180.00)); // Convert degrees to radians
}

/*
 * Function: set_hovercraft_forces
 * Input: fx (float) - Force in the x direction
 *        fy (float) - Force in the y direction
 *        torque (float) - Torque to apply
 * Output: none
 * Computes forces for each motor and applies them to set hovercraft motion.
 */
void set_hovercraft_forces(float fx, float fy, float torque) {
  float F1, F2, F3;
  float R = 5.5; // Effective radius of the hovercraft in inches

  // Calculate forces for each motor based on fx, fy, and torque
  F1 = ((1 / (2 * cos(angle_radians(30)))) * fx) +
       ((-1 / (2 + (2 * sin(angle_radians(30))))) * fy) +
       ((-1 / (2 * R * (1 + sin(angle_radians(30))))) * torque);

  F2 = ((1 / (2 * cos(angle_radians(30)))) * fx) +
       ((1 / (2 + (2 * sin(angle_radians(30))))) * fy) +
       ((1 / (2 * R * (1 + sin(angle_radians(30))))) * torque);

  F3 = ((-1 / (1 + sin(angle_radians(30)))) * fy) +
       ((sin(angle_radians(30)) / (R * (1 + sin(angle_radians(30))))) * torque);

  // Apply calculated forces to the motors
  float forces[] = {F1, F2, F3};
  set_motors(forces);
}

/*
 * Function: compute_error
 * Input: theta (float) - Current orientation angle from IMU
 *        heading_goal (float) - Desired orientation direction
 * Output: float - Angle difference between current position and goal (-180 to 180)
 * Computes the error angle required for the hovercraft to reach the desired heading.
 */
float compute_error(float theta, float heading_goal) {
  float diff = heading_goal - theta;
  if (diff >= 0 && diff <= 180) {
    return diff;
  } else {
    return diff; // Adjusted for range but not exceeding limits
  }
}

/*
 * Function: db_clip
 * Input: error (float) - Difference between current and desired heading
 *        deadband (float) - Acceptable range where no motor adjustment occurs
 *        saturation (float) - Maximum allowed angle
 * Output: float - Scaled error after applying deadband and saturation
 * Clips the error within the bounds of deadband and saturation, scaling it accordingly.
 */
float db_clip(float error, float deadband, float saturation) {
  float value = 0;
  if (error <= deadband && error >= -deadband) { // Error within deadband
    return value;
  } else if (abs(error) >= saturation) { // Error exceeds saturation
    value = saturation - deadband;
  } else if (abs(error) >= deadband && abs(error) <= saturation) { // Error within range
    value = (error / saturation) * (saturation - deadband);
  } else { // Default case (should not execute)
    value = 0.0;
  }
  return value;
}

/*
 * Function: pd_step
 * Input: none
 * Output: none
 * Computes the heading error using the IMU and applies proportional-derivative control
 * to counter unwanted rotations.
 */
void pd_step() {
  float Kd = 4.5; // PD control gain
  float fx = 0, fy = 0, rotation = 0;
  float heading_error = compute_error(IMU.yaw, rotation); // Compute heading error
  float modified_error = db_clip(heading_error, 15, 300); // Clip error within deadband and saturation
  float torque = Kd * modified_error; // Calculate torque for correction

  if (motor_on) { // If motors are on, apply torque
    set_hovercraft_forces(fx, fy, torque);
  } else { // Otherwise, stop hovercraft forces
    set_hovercraft_forces(0, 0, 0);
  }
}

/*
 * Function: report_step
 * Input: none
 * Output: none
 * Reports cumulative x and y slip values for each camera. Allows resetting the values via a serial command.
 */
void report_step() {
  // Print cumulative slip values for all cameras
  Serial.printf("Camera 1: adx = %d ady = %d, Camera 2: adx = %d ady = %d, Camera 3: adx = %d ady = %d \n",
                adx[0], ady[0], adx[1], ady[1], adx[2], ady[2]);

  if (Serial.available()) {
    char found = Serial.read(); // Check for input in serial monitor
    if (found == 'c') { // Reset cumulative values if 'c' is received
      for (int i = 0; i < 3; ++i) {
        adx[i] = 0;
        ady[i] = 0;
      }
    }
  }
}

/*
 * Function: camera_step
 * Input: none
 * Output: none
 * Reads slip values from all cameras, updates cumulative slip values,
 * and handles error cases.
 */
void camera_step() {
  int8_t dx, dy;                         // Variables to hold slip values
  uint8_t quality;                       // Variable to hold quality of the optical flow
  for (int index = 0; index < 3; ++index) {
    int result = cams.readSlip(CAMERA_SELECT[index], dx, dy, quality);
    if (result == 0) {                   // Successful read
      adx[index] += dx;
      ady[index] += dy;
    } else if (result == -1) {           // Overflow error
      Serial.printf("Overflow with Camera: %d \n", index + 1);
    } else if (result == -2) {
      // Do nothing for this error
    } else {
      Serial.printf("Unexpected Return Value (camera result) \n");
    }
  }
}

/*
 * Function: report_step
 * Input: none
 * Output: none
 * Reports cumulative slip values for each camera. Allows resetting via serial command.
 */
void report_step() {
  Serial.printf("Camera 1: adx = %d ady = %d, Camera 2: adx = %d ady = %d, Camera 3: adx = %d ady = %d \n", 
                adx[0], ady[0], adx[1], ady[1], adx[2], ady[2]);
  if (Serial.available()) {
    char found = Serial.read();
    if (found == 'c') {                  // Clear cumulative slip values if 'c' is received
      for (int i = 0; i < 3; ++i) {
        adx[i] = 0;
        ady[i] = 0;
      }
    }
  }
}

/*
 * Function: loop
 * Input: none
 * Output: none
 * Continuously executes periodic tasks for camera updates and reporting.
 */
void loop() {
  static PeriodicAction report_task(1000, report_step); // Report every 1 second
  static PeriodicAction camera_task(5, camera_step);    // Read camera slip every 5 ms
  camera_task.step();
  report_task.step();
}
