#include <quaternionFilters.h>  // Library for quaternion filtering for IMU data
#include <MPU9250.h>            // Library for MPU9250 IMU
#include <PWMServo.h>           // Library for PWM servo motor control
#include "project.h"            // Custom header file for project-specific enumerations and constants
#include "PeriodicAction.h"     // Library for scheduling periodic actions
#include <ImuUtils.h>           // IMU utility functions
#include <OpticalFlowCamera.h>  // Library for interfacing with optical flow cameras

// Pin assignments for motors, cameras, and other peripherals
#define switch_pin 0
#define m1_clockwise 23
#define m1_c_clockwise 22
#define m1_pulsewidth 21
#define m2_clockwise 1
#define m2_c_clockwise 2
#define m2_pulsewidth 3
#define m3_clockwise 8
#define m3_c_clockwise 7
#define m3_pulsewidth 9
#define SDA_0 18
#define SCL_0 19
#define pin_MISO 12
#define pin_SCL 13
#define pin_MOSI 11
#define cam_1 15
#define cam_2 16
#define cam_3 17
#define RESET 14

// Global variables for state and control
static boolean motor_on = false;                     // Tracks if drive motors are active
const uint8_t CAMERA_SELECT[3] = {cam_1, cam_2, cam_3}; // Pins for camera control
OpticalFlowCamera cams(RESET);                       // Optical flow camera object
int32_t adx[3] = {0, 0, 0};                          // Summed x-slip values from cameras
int32_t ady[3] = {0, 0, 0};                          // Summed y-slip values from cameras
PWMServo fan;                                        // Servo object to control the central fan
const int CENTRAL_FAN_PWM = 36;                      // Pin for central fan PWM
float linear_position_goal[2];                       // x, y position goals
float linear_velocity_goal[2];                       // x_dot, y_dot velocity goals

/*
 * Function: fan_setup
 * Initializes the brushless lift motor with low throttle for safety.
 */
void fan_setup() {
  fan.attach(CENTRAL_FAN_PWM);  // Attach the fan to the specified pin
  delay(100);
  fan.write(20);                // Set low throttle to initialize
  delay(3000);                  // Allow time for the fan to stabilize
}

/*
 * Function: camera_setup
 * Initializes all optical flow cameras. Retries until all cameras are functional.
 */
void camera_setup() {
  int ret_1 = -1, ret_2 = -1, ret_3 = -1;
  do { // Retry initialization until successful
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
 * Configures all pins as input/output, initializes cameras and IMU, and starts the serial monitor.
 */
void setup() {
  fan_setup();                  // Initialize lift motor
  camera_setup();               // Initialize cameras
  pinMode(switch_pin, INPUT);   // Configure switch pin as input

  // Initialize motor control pins
  pinMode(m1_clockwise, OUTPUT);
  pinMode(m1_c_clockwise, OUTPUT);
  pinMode(m1_pulsewidth, OUTPUT);
  pinMode(m2_clockwise, OUTPUT);
  pinMode(m2_c_clockwise, OUTPUT);
  pinMode(m2_pulsewidth, OUTPUT);
  pinMode(m3_clockwise, OUTPUT);
  pinMode(m3_c_clockwise, OUTPUT);
  pinMode(m3_pulsewidth, OUTPUT);

  // Set all motor pins to LOW (off)
  digitalWrite(m1_clockwise, LOW);
  digitalWrite(m1_c_clockwise, LOW);
  digitalWrite(m1_pulsewidth, LOW);
  digitalWrite(m2_clockwise, LOW);
  digitalWrite(m2_c_clockwise, LOW);
  digitalWrite(m2_pulsewidth, LOW);
  digitalWrite(m3_clockwise, LOW);
  digitalWrite(m3_c_clockwise, LOW);
  digitalWrite(m3_pulsewidth, LOW);

  Serial.begin(115200);         // Start serial communication
  Serial.setTimeout(100000000); // Set long timeout for serial communication
  imu_setup();                  // Initialize IMU

  // Apply IMU calibration values
  IMU.magbias[0] = 145.327698;
  IMU.magbias[1] = -69.055908;
  IMU.magbias[2] = 395.881012;
  delay(1000);
}

/*
 * Function: bound
 * Ensures a value stays within the specified range.
 * Input: value, min_value, max_value
 * Output: Bounded value
 */
float bound(float value, float min_value, float max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

/*
 * Function: set_motors
 * Configures motor directions and sets power levels based on input forces.
 */
void set_motors(float val[3]) {
  const float negative_gain[3] = {2.7, 2.3, 2.4}; // Gain for negative forces
  const float positive_gain[3] = {1.6, 1.2, 1.2}; // Gain for positive forces

  for (int i = 0; i <= 2; ++i) {
    // Determine motor pins
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

    // Apply direction and power
    if (val[i] > 0) { // Clockwise spin
      val[i] *= positive_gain[i];
      digitalWrite(motorClockwise, LOW);
      digitalWrite(motor_C_Clockwise, HIGH);
    } else if (val[i] < 0) { // Counterclockwise spin
      val[i] *= negative_gain[i];
      digitalWrite(motorClockwise, HIGH);
      digitalWrite(motor_C_Clockwise, LOW);
    } else { // Stop motor
      digitalWrite(motorClockwise, LOW);
      digitalWrite(motor_C_Clockwise, LOW);
    }

    // Write power to motor (bounded value)
    analogWrite(motorPulse, abs(bound(val[i], -90, 64)));
    Serial.printf("Power %f Motor %d \n", val[i], i);
  }
}

/*
   input: int deg - degree representation of an angle
   output: converted float of the radian value of an angle
   function: takes an angle and returns equivalent radian representation
*/
float angle_radians(int deg) {
  return (((float) deg) * (PI / 180.00));
}

/*
   input: float fx - desired forces in x direction, float fy - desired forces in y direction, float torque - desired torque of vehicle
   output: F1 - motor one forces, F2 - motor two forces, F3 - motor three forces
   function: Takes desired fx, fy, and torque specified and converts that to power vals of the three attached motors
*/
void set_hovercraft_forces(float fx, float fy, float torque) {
  float F1;
  float F2;
  float F3;
  float R = 5.5; //effective radius of hovercraft in inches
  F1 = ((1 / (2 * cos(angle_radians(30)))) * fx) + ((-1 / (2 + (2 * sin(angle_radians(30))))) * fy) + ((-1 / (2 * R * (1 + (sin(angle_radians(30)))))) * torque);
  F2 = ((1 / (2 * cos(angle_radians(30)))) * fx) + ((1 / (2 + (2 * sin(angle_radians(30))))) * fy) + ((1 / (2 * R * (1 + (sin(angle_radians(30)))))) * torque);
  F3 = ((-1 / (1 + (1 * sin(angle_radians(30))))) * fy) + ((sin(angle_radians(30)) / (R * (1 + (sin(angle_radians(30)))))) * torque);
  float forces[] = {F1, F2, F3};
  set_motors(forces); //sets the motor power with the calculated new forces
}

/*
    input: none
    output: none
    function: sets up the state machine behavior for vehicle function
*/
void fsm_step() {
  static int timingTotal = 0; //used for totalvehicle time
  static int refTime = timingTotal; //used for timing reference in wait stages
  const float motorPow = 84; //constant describing lift fan force (duty cycle = motorPow/255)
  static State current = STATE_START; //initialized in do nothing state
  static State prevState = STATE_START; //used for state transition determination based on specified steps (in instrustions)
  //Serial.printf("State: %d \n" , current);
}
   
/*
   input: theta (current orietation angle from IMU), heading_goal (desired orientation direction)
   output: float (returns the angle difference between current position and goal (bounded by -180 and 180);
   function: computes the return angle needed for the vehicle to travel
*/
float compute_error(float theta, float heading_goal) {
  float diff = heading_goal - theta;
  if (diff >= 0 && diff <= 180) {
    return diff;
  }
  else {
    return diff;
  }
}

/*
   input: error (angle error difference from heading goal), deadband (acceptable angle range to not turn motors), saturation (maximum angle)
   output: float - value (modified/scaled error returned)
   function: computes a new scaled error based on the original error difference
*/
float db_clip(float error, float deadband, float saturation) {
  float value = 0;
  int state = 0;
  if (error <= deadband && error >= -deadband) {
    value = 0;
    state = 3;
  }

  else if (abs(error) >= saturation) { //absolute value case here to handle potential negatives
    // Serial.printf("Error: %f \n", abs(error));
    value = (saturation - deadband);
    if (error < 0) value = value * -1;
    //value = value;
    state = 1;
  }
  else if (abs(error) <= saturation && abs(error) >= deadband) { //same as above absolute value handles negatives
    value =  ((abs(error) - deadband) / (saturation)) * (saturation - deadband);
    if (error < 0) value = value * -1;
    state = 2;
  }
  else { //should never execute
    value = 0.0;
    state = 3;
  }
  // Serial.printf("State: %d Value: %f \n", state, value);
  return value;
}

/*
 * Function: pd_step
 * Implements proportional-derivative (PD) control to counter unwanted rotation.
 */
void pd_step() {
  float Kp = 10.0;                 // Proportional gain
  float Kd = 0.75;                 // Derivative gain
  float fx = 0, fy = 0, rotation = 0;
  static float prev_error = 0;     // Track previous error
  float heading_error = compute_error(IMU.yaw, rotation); // Calculate heading error
  float modified_error = db_clip(heading_error, 3, 40);   // Clip error within deadband
  float derivative = Kd * (modified_error - prev_error) / 0.005; // Compute derivative
  float c_torque = Kp * modified_error + derivative;      // Compute torque

  if (motor_on) { // Apply forces if motors are active
    set_hovercraft_forces(fx, fy, c_torque);
  } else {
    set_hovercraft_forces(0, 0, 0);
  }
  prev_error = modified_error; // Update previous error
}

/*
   input: one
   output: none
   function: prints the summed adx and ady slip values as time increases for each motor.
*/
void report_step() {
  Serial.printf("Rotation Rate: %f \n", IMU.yaw);
  Serial.println(motor_on);
  Serial.printf("Camera 1: adx = %d ady = %d, Camera 2: adx = %d ady = %d, Camera 3: adx = %d ady = %d \n", adx[0], ady[0], adx[1], ady[1], adx[2], ady[2]); //each motor's value displayed
  if (Serial.available()) {
    char found = Serial.read(); //clear command available and found from monitor will reset all values summed to 0
    if (found == 'c') {
      for (int i = 0; i < 3; ++i) {
        adx[i] = 0;
        ady[i] = 0;
      }
    }
  }
}

/*
   input: none
   output: none
   function: calls the cameras in sequence to determine the slip in x and y directions
*/
void camera_step() {
  int8_t dx;
  int8_t  dy;
  uint8_t  quality;
  uint8_t aquality[3] = {0, 0, 0};
  int index = 0;
  for (index = 0; index < 3; ++index) {
    int result = cams.readSlip(CAMERA_SELECT[index], dx, dy, quality); //one line to represent each camera with for loop
    if (result == 0) {
      adx[index] += dx;
      ady[index] += dy;
      aquality[index] += quality;
    }
    else if (result == -1) {
      Serial.printf("Overlow with Camera: %d \n", index + 1); //case where overflow occured prints which camera
    }
    else if (result == -2) {
      //case to do nothing as sum should not be increased
    }
    else {
      Serial.printf("Unexpected Return Value (camera result) \n"); //state should never execute, if it does report
    }
  }
}
   
/*
   input: adx - sum of acculumated x slip for each of 3 cameras, ady - sum of acculumated y slip for each of 3 cameras, cartesian_pos - stores the cartesian motion in each coordinate direction
   output: none
   function: stores/recomputes the vehicle's chassis motion through the use of camera slip
*/
void compute_chassis_motion(int32_t adx[3], int32_t ady[3], float cartesian_pos[3]) {
  float X_calc = 44.4737 + (-0.02519 * adx[0]) + (-0.068568 * ady[0]) + (-0.36969 * adx[1]) + (0.037334 * ady[1]) + (0.040249 * adx[2]) + (0.0019116 * ady[2]);
  float Y_calc = 35.625 + (0.0050243 * adx[0]) + (0.045442 * ady[0]) + (0.004378 * adx[1]) + (-0.016571 * ady[1]) + (-0.028766 * adx[2]) + (0.0040616 * ady[2]);
  float T_calc = 21.1173 + (0.085319 * adx[0]) + (0.097841 * ady[0]) + (0.13789 * adx[1]) + (-0.087841 * ady[1]) + (-0.048582 * adx[2]) + (-0.025272 * ady[2]);
  cartesian_pos[0] = X_calc; //sets calling function array values due to pass by reference
  cartesian_pos[1] = Y_calc;
  cartesian_pos[2] = T_calc;
}

/*
 * Function: loop
 * Executes periodic tasks for hovercraft control and camera monitoring.
 */
void loop() {
  static PeriodicAction fsm_task(50, fsm_step);
  static PeriodicAction imu_task(5, imu_update);
  static PeriodicAction pd_task(10, pd_step);
  static PeriodicAction report_task(1000, report_step);
  static PeriodicAction camera_task(5, camera_step);

  imu_task.step();        // Update IMU
  pd_task.step();         // Perform PD control
  fsm_task.step();        // State machine step
  camera_task.step();     // Update camera slip values
  report_task.step();     // Report system status
}
