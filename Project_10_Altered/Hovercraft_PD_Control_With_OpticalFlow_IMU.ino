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
float carteisian_pos[3] = {0.0, 0.0, 0.0};
PWMServo fan; // create servo object to control the fan
const int CENTRAL_FAN_PWM = 36;
float linear_position_goal[2] = {0,0}; // x, y: set position goal for the hovercraft
float linear_position[2] = {0,0}; // current position of the hovercraft
float linear_velocity[2] = {0,0}; // current velocity of the hovercraft
float linear_velocity_goal[2] = {0,0}; // x_dot, y_dot: set velocity goal for the hovercraft
//Global Varaible end

/*
   input: none
   output: none
   function: initializes the Brushless lift motor for power commands
*/
void fan_setup() {
  fan.attach(CENTRAL_FAN_PWM); // attaches the fan to specified
  // Arduino pin to the object
  delay(100);
  fan.write(20); // write low throttle
  delay(3000);
}

/*
   input: none
   output: none
   function: will set up all cameras. If an issue occurs a while loop continues such that the remainder of the program cannot continue until all cameras are able
*/
void camera_setup() {
  int ret_1 = -1;
  int ret_2 = -1;
  int ret_3 = -1;
  do { //do while used so that the cameras are attempted to be initialized at least once
    ret_1 = cams.addCamera(CAMERA_SELECT[0]);
    ret_2 = cams.addCamera(CAMERA_SELECT[1]);
    ret_3 = cams.addCamera(CAMERA_SELECT[2]);
    if (ret_1 != 0) Serial.printf("Camera 1 Down \n");
    if (ret_2 != 0) Serial.printf("Camera 2 Down \n");
    if (ret_3 != 0) Serial.printf("Camera 3 Down \n");

  } while (ret_1 != 0 || ret_2 != 0 || ret_3 != 0);
}

/*
   input: none
   output: none
   Function: intitalizes all used pins as input/outputs and begins Serial monitor
*/
void setup() {
  fan_setup(); //sets the brushless motor power
  camera_setup();
  pinMode(switch_pin, INPUT);
  pinMode(m1_clockwise, OUTPUT);
  pinMode(m1_c_clockwise, OUTPUT);
  pinMode(m1_pulsewidth, OUTPUT);
  digitalWrite(m1_clockwise, LOW);
  digitalWrite(m1_c_clockwise, LOW);
  digitalWrite(m1_pulsewidth, LOW);
  pinMode(m2_clockwise, OUTPUT);
  pinMode(m2_c_clockwise, OUTPUT);
  pinMode(m2_pulsewidth, OUTPUT);
  digitalWrite(m2_clockwise, LOW);
  digitalWrite(m2_c_clockwise, LOW);
  digitalWrite(m2_pulsewidth, LOW);
  pinMode(m3_clockwise, OUTPUT);
  pinMode(m3_c_clockwise, OUTPUT);
  pinMode(m3_pulsewidth, OUTPUT);
  digitalWrite(m3_clockwise, LOW);
  digitalWrite(m3_c_clockwise, LOW);
  digitalWrite(m3_pulsewidth, LOW);
  Serial.begin(115200);
  Serial.setTimeout(100000000);
  imu_setup();
  //imu_calibrate_magbias();
  IMU.magbias[0] = 145.327698; //calibration information as recieved from IMU
  IMU.magbias[1] = -69.055908;
  IMU.magbias[2] = 395.881012;
  delay(1000);
}

/*
*
* input: value - the actual value to be bounded, min_value - the minimum bound, max_value - the maximum bound
* output: bounded value between min and max bounds
* function: takes a value and if outside of bounds specified, bounds it
*/
float bound(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value; //value below min value coersion
  }
  else if (value > max_value) {
    return max_value; //value above max value coersion
  }
  else {
    return value;
  }
}

/*
 * input: val[3] - desired motor power values in order of indices is left, right, back
 * output: none
 * Function: sets the motor directions and then sets motor speed by writing to motor speed function, setting motors in order of left, right, back
 */
void set_motors(float val[3]){
  const float negative_gain[3] = {2.4, 2.3, 2.4}; //negative gain to be used to multiply negative motor values
  const float positive_gain[3] = {1.6, 1.1, 1.1}; //used to correct weak motors in the normal direction
  for(int i = 0; i <= 2; ++i){
    int motorClockwise = 0; // following variables used as an alias to switch between motors for control code
    int motor_C_Clockwise = 0;
    int motorPulse = 0;
    if(i == 0){ //0 is left motor
      motorClockwise = m1_clockwise;
      motor_C_Clockwise = m1_c_clockwise;
      motorPulse = m1_pulsewidth;
    }
     else if(i == 1){ //1 is right motor
      motorClockwise = m2_clockwise;
      motor_C_Clockwise = m2_c_clockwise;
      motorPulse = m2_pulsewidth;
    }
     else if(i == 2){ // 2 is the back motor
      motorClockwise = m3_clockwise;
      motor_C_Clockwise = m3_c_clockwise;
      motorPulse = m3_pulsewidth;
    }
    else{
      break; //should never execute
    }
   if(val[i]> 0){ //if val is positive spin is clockwise
    val[i] = val[i] * positive_gain[i];
    digitalWrite(motorClockwise, LOW); //direction counter clockwise spin
    digitalWrite(motor_C_Clockwise, HIGH);
  }
  else if(val[i] < 0){ //if val is negative spin is counter clockwise
    val[i] = val[i] * negative_gain[i];
    digitalWrite(motorClockwise, HIGH); //direction clockwise spin
    digitalWrite(motor_C_Clockwise, LOW);
  }
  else{
    digitalWrite(motorClockwise, LOW); //off
    digitalWrite(motor_C_Clockwise, LOW);
  }
    analogWrite(motorPulse, abs(bound(val[i], -64, 64))); //power to motor absolute value taken because power must be > 0. (Bounds Can be Manipulated for Power Tuning)

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
  fx = -fx;
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
  Serial.printf("State: %d \n" , current);
  switch (current) {

    case STATE_START: // Starting Case for initial power on
    carteisian_pos[0] = 0;
    carteisian_pos[1] = 0;
    fan.write(0);
    if(digitalRead(switch_pin) == HIGH){
     current = STATE_ACTIVATE; //vehicle was turned on
     motor_on = true;
     prevState = STATE_START;
     Serial.printf("Start\n");
     }
    else current = STATE_START;
    Serial.printf("Starting\n");
    break;
    
    case STATE_ACTIVATE: // Activation Case for the lift fan
    Serial.printf("Activate\n");
    // Set position to zero
    adx[0] = 0;
    adx[1] = 0;
    adx[2] = 0;
    ady[0] = 0;
    ady[1] = 0;
    ady[2] = 0;
    // velocity goals set to zero
    linear_velocity_goal[0] = 0;
    linear_velocity_goal[1] = 0;
    fan.write(motorPow);
    current = STATE_WAIT;
    prevState = STATE_ACTIVATE;
    break;

    case STATE_WAIT: // Wait Case for determining switch pressed
    Serial.printf("Wait\n");
    if(digitalRead(switch_pin) == HIGH){
      current = STATE_MOVE_1;
      prevState = STATE_WAIT;
    }
    else current = STATE_WAIT;
    break;
    
    case STATE_MOVE_1: // First Movement Case 
    Serial.printf("Move 1\n");
    if(timingTotal * 0.050 < 10){
    // set velocity goal to move along x-axis
    linear_velocity_goal[0] = 0.5;
    linear_velocity_goal[1] = 0;
    timingTotal += 1;
    }
    else{
      // set position to zero
      adx[0] = 0;
      adx[1] = 0;
      adx[2] = 0;
      ady[0] = 0;
      ady[1] = 0;
      ady[2] = 0;
      // velocity goals set to zero
      linear_velocity_goal[0] = 0;
      linear_velocity_goal[1] = 0;
      fan.write(motorPow);
      current = STATE_MOVE_2;
      prevState = STATE_MOVE_1;
        timingTotal = 0;
    }
    timingTotal += 1;
    break;
    
    case STATE_MOVE_2: // Second Movement Case
    Serial.printf("Move 2\n");
    if(prevState == STATE_MOVE_1){
      prevState = STATE_MOVE_2;
      refTime = timingTotal;
    }
    if((timingTotal - refTime) * 0.05 < 10){
      // set velocity goal to move along y-axis
      linear_velocity_goal[0] = 0;
      linear_velocity_goal[1] = 0.5;
      timingTotal += 1;
    }
    else{
      // velocity goals set to zero
      linear_velocity_goal[0] = 0;
      linear_velocity_goal[1] = 0;
      current = STATE_DEACTIVATE;
    }
    break;
    
    case STATE_DEACTIVATE: // Deactivate Case for shutting down hovercraft
    Serial.printf("Shut Down\n");
    fan.write(0);
    for(int i = 0; i < 2; ++i){
      // set position to zero
      adx[0] = 0;
      adx[1] = 0;
      adx[2] = 0;
      ady[0] = 0;
      ady[1] = 0;
      ady[2] = 0;
      // position goals set to zero
      linear_position[i] = 0;
      linear_position_goal[i] = 0;
    }
    current = STATE_START;
    prevState = STATE_START;
    break;
  } 
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
  if(error <= deadband && error >= -deadband){
    value = 0;
    state = 3;
  }

  else if (abs(error) >= saturation){ //absolute value case here to handle potential negatives
   // Serial.printf("Error: %f \n", abs(error));
    value = saturation - deadband;
    if(error < 0) value = value * -1;
    //value = value;
    state = 1;
  }
  else if (abs(error) <= saturation && abs(error) >= deadband){ //same as above absolute value handles negatives
    value =  2* ((abs(error) - deadband)/(saturation))* (saturation - (deadband));
    if(error < 0) value = value *-1;
    state = 2;
  }
  else{ //should never execute
    value = 0.0;
    state = 3;
  }
 // Serial.printf("State: %d Value: %f \n", state, value);
  return value;
}

/*
   input: none
   output: none
   function: sets the hovercraft forces based on the IMU z rotation axis to counter the rotation
*/
void pd_step() {
  float Kp = 7; //required gain value
  float fx = 0;
  float fy = 0;
  float rotation = 0;
  float heading_error = compute_error(IMU.yaw, rotation);
  float modified_error = db_clip(heading_error, 3, 90);
  static float prev_error = 0;
  float Kd = 2.2;
  float derivative = Kd * ( modified_error - prev_error) / 0.005;
  float c_torque = Kp * modified_error + derivative; //set torque for counter balancing
  // Serial.printf("Torque: %f \n", torque);
  prev_error = modified_error;
  float dt = 0.010;
  for(int i = 0; i < 2; ++i){
  linear_position_goal[i] += linear_velocity_goal[i] * dt;
  }
  float KLp = 0.3;
  float KLv = 0.3;
 // Serial.printf("pos x: %f, pos y: %f \n", linear_position[0], linear_position[1]);
  fx = KLp * (linear_position_goal[0] - (-linear_position[0])) +  KLv * (linear_velocity_goal[0] - linear_velocity[0]);
  fy = KLp * (linear_position_goal[1] - linear_position[1]) +  KLv * (linear_velocity_goal[1] - linear_velocity[1]);
  if (motor_on == true) set_hovercraft_forces(fx, fy, c_torque); //determines if the motors should be activated (only if lift is on)
  else set_hovercraft_forces(0, 0, 0);
  //Serial.printf("Fx: %f, Fy: %f, Torque: %f \n", fx, fy, c_torque);
}

/*
   input: one
   output: none
   function: prints the summed adx and ady slip values as time increases for each motor.
*/
void report_step() {
 // Serial.printf("Rotation Rate: %f \n", IMU.yaw);
 // Serial.println(motor_on);
 // Serial.printf("Camera 1: adx = %d ady = %d, Camera 2: adx = %d ady = %d, Camera 3: adx = %d ady = %d \n", adx[0], ady[0], adx[1], ady[1], adx[2], ady[2]); //each motor's value displayed
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
  float x1 = carteisian_pos[0];
  float y1 = carteisian_pos[1];
  compute_chassis_motion(adx, ady, carteisian_pos);
  float x2 = carteisian_pos[0];
  float y2 = carteisian_pos[1];
  linear_position[0] += (x2 - x1); // computes the absolute position (x-axis)
  linear_position[1] += (y2 - y1); // computes the absolute position (y-axis)
  //Serial.printf("pos1: %f, pos2: %f \n", linear_position[0], linear_position[1]);
 // Serial.printf("x2-x1: %f \n", x2-x1);
  float dt = 0.005; // time derivative
  float tau = 0.05;
  linear_velocity[0] = linear_velocity_goal[0] * (1 - dt/tau) + (x2 - x1); // low-pass filtered velocity (x-axis)
  linear_velocity[1] = linear_velocity_goal[1] * (1 - dt/tau) + (y2 - y1); // low-pass filtered velocity (y-axis)
}

/*
   input: adx - sum of acculumated x slip for each of 3 cameras, ady - sum of acculumated y slip for each of 3 cameras, cartesian_pos - stores the cartesian motion in each coordinate direction
   output: none
   function: stores/recomputes the vehicle's chassis motion through the use of camera slip
*/
void compute_chassis_motion(int32_t adx[3], int32_t ady[3], float cartesian_pos[3]) {
  float X_calc = (-0.02519 * adx[0]) + (-0.068568 * ady[0]) + (-0.36969 * adx[1]) + (0.037334 * ady[1]) + (0.040249 * adx[2]) + (0.0019116 * ady[2]);
  float Y_calc = (0.0050243 * adx[0]) + (0.045442 * ady[0]) + (0.004378 * adx[1]) + (-0.016571 * ady[1]) + (-0.028766 * adx[2]) + (0.0040616 * ady[2]);
  float T_calc = (0.085319 * adx[0]) + (0.097841 * ady[0]) + (0.13789 * adx[1]) + (-0.087841 * ady[1]) + (-0.048582 * adx[2]) + (-0.025272 * ady[2]);
  if(adx[0] < 5 && adx[1] < 5 && adx[1] < 5 && ady[0] < 5 && ady[1] < 5 && ady[2] < 5){
    X_calc = 0;
    Y_calc = 0;
    T_calc = 0;
  }
  //Serial.printf("X: %f, Y: %f, \n", carteisian_pos[0],carteisian_pos[1]);
  carteisian_pos[1] = X_calc; //sets calling function array values due to pass by reference
  carteisian_pos[0] = 8 * Y_calc;
  carteisian_pos[2] = T_calc;

}

/*
   input: none
   output: none
   function: repeatedly calls the action events to take place with the hovercraft
*/
void loop() {
  // put your main code here, to run repeatedly:
  static PeriodicAction fsm_task(50, fsm_step);
  static PeriodicAction imu_task(5, imu_update);
  static PeriodicAction pd_task(10, pd_step);
  static PeriodicAction report_task(1000, report_step);
  static PeriodicAction camera_task(5, camera_step);
  imu_task.step();
  pd_task.step();
  fsm_task.step();
  camera_task.step();
  report_task.step();
}
