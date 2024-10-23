
/*
   Project 9
   Group 4
   Submitted: 4/20/23
   Authors: Kyler Clark, Ashley Yi, Caden Matthews
*/
#include <quaternionFilters.h>
#include <MPU9250.h>
#include <PWMServo.h>
#include "project.h" //brings enum vals in
#include "PeriodicAction.h"
#include <ImuUtils.h>
#include <OpticalFlowCamera.h>

#define switch_pin 0 //following lines give aliases to pin numbers
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
// Global Variables defined below
static boolean motor_on = false; //field to hold state of drive motors
const uint8_t CAMERA_SELECT[3] = {cam_1, cam_2, cam_3}; //pin selection for camera control
OpticalFlowCamera cams(RESET);
int32_t adx[3] = {0, 0, 0}; //holds summed camera x slips
int32_t ady[3] = {0, 0, 0}; //holds summed camera y slips
PWMServo fan; // create servo object to control the fan
const int CENTRAL_FAN_PWM = 36;
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
   input: value - the actual value to be bounded, min_value - the minimum bound, max_value - the maximum bound
   output: bounded value between min and max bounds
   function: takes a value and if outside of bounds specified, bounds it
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
   input: val[3] - desired motor power values in order of indices is left, right, back
   output: none
   Function: sets the motor directions and then sets motor speed by writing to motor speed function, setting motors in order of left, right, back
*/
void set_motors(float val[3]) {
  const float negative_gain[3] = {2.2, 2.2, 2.2}; //negative gain to be used to multiply negative motor values
  const float positive_gain[3] = {1.2, 1.2, 1.1}; //used to correct weak motors in the normal direction
  for (int i = 0; i <= 2; ++i) {
    int motorClockwise = 0; // following variables used as an alias to switch between motors for control code
    int motor_C_Clockwise = 0;
    int motorPulse = 0;
    if (i == 0) { //0 is left motor
      motorClockwise = m1_clockwise;
      motor_C_Clockwise = m1_c_clockwise;
      motorPulse = m1_pulsewidth;
    }
    else if (i == 1) { //1 is right motor
      motorClockwise = m2_clockwise;
      motor_C_Clockwise = m2_c_clockwise;
      motorPulse = m2_pulsewidth;
    }
    else if (i == 2) { // 2 is the back motor
      motorClockwise = m3_clockwise;
      motor_C_Clockwise = m3_c_clockwise;
      motorPulse = m3_pulsewidth;
    }
    else {
      break; //should never execute
    }
    if (val[i] > 0) { //if val is positive spin is clockwise
      val[i] = val[i] * positive_gain[i];
      digitalWrite(motorClockwise, LOW); //direction counter clockwise spin
      digitalWrite(motor_C_Clockwise, HIGH);
    }
    else if (val[i] < 0) { //if val is negative spin is counter clockwise
      val[i] = val[i] * negative_gain[i];
      digitalWrite(motorClockwise, HIGH); //direction clockwise spin
      digitalWrite(motor_C_Clockwise, LOW);
    }
    else {
      digitalWrite(motorClockwise, LOW); //off
      digitalWrite(motor_C_Clockwise, LOW);
    }
    analogWrite(motorPulse, abs(bound(val[i], -255, 255))); //power to motor absolute value taken because power must be > 0. (Bounds Can be Manipulated for Power Tuning)

    //Serial.printf("Power %f Motor %d \n", val[i], i);
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
  switch (current) { //switch for state transitions
    case STATE_START: //initial state
      fan.write(0);
      set_hovercraft_forces(0, 0, 0);
      if (digitalRead(switch_pin == HIGH)) {
        current = STATE_ACTIVATE; //vehicle was turned on
        motor_on = true;
        //  Serial.printf("Start\n");
      }
      else current = STATE_START;
      // Serial.printf("Starting\n");
      break;

    case STATE_ACTIVATE: //sets lift motor to motorPow
      fan.write(motorPow);
      current = STATE_WAIT_30;
      prevState = STATE_ACTIVATE;
      timingTotal += 1;
      break;

    case STATE_WAIT_30:
      fan.write(motorPow);
      if (prevState != STATE_WAIT_30) { //first iteration of state sets reference Time
        refTime = timingTotal;
        prevState = STATE_WAIT_30;
      }
      if ((timingTotal - refTime) <= 600) { //checks for 15 seconds (300 * 0.050)
        timingTotal += 1;
        current = STATE_WAIT_30;
        //Serial.printf("Going to STATE WAIT 30\n");
      }
      else current = STATE_DEACTIVATE;
      //Serial.printf("Pow: %f \n" , motorPow);
      break;
    case STATE_DEACTIVATE:
      fan.write(0);
      prevState = STATE_START;
      current = STATE_START;
      motor_on = false;
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
  if (error <= deadband && error >= -deadband) {
    return value;
  }

  else if (abs(error) >= saturation) { //absolute value case here to handle potential negatives
    Serial.printf("Error: %f \n", abs(error));
    value = saturation - deadband;
    state = 1;
  }
  else if (abs(error) <= saturation && abs(error) >= deadband) { //same as above absolute value handles negatives
    value = (error / (saturation)) * (saturation - deadband);
    state = 2;
  }
  else { //should never execute
    value = 0.0;
  }
  //Serial.printf("State: %d Value: %f \n", state, value);
  return value;
}

/*
   input: none
   output: none
   function: sets the hovercraft forces based on the IMU z rotation axis to counter the rotation
*/
void pd_step() {
  float Kd = 4.5; //required gain value
  float fx = 0;
  float fy = 0;
  float rotation = 0;
  float heading_error = compute_error(IMU.yaw, rotation);
  float modified_error = db_clip(heading_error, 15, 300);
  float torque = Kd * modified_error; //set torque for counter balancing
  // Serial.printf("Torque: %f \n", torque);
  if (motor_on == true) set_hovercraft_forces(fx, fy, torque); //determines if the motors should be activated (only if lift is on)
  else set_hovercraft_forces(0, 0, 0);
}

/*
   input: one
   output: none
   function: prints the summed adx and ady slip values as time increases for each motor.
*/
void report_step() {
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
   input: none
   output: none
   function: repeatedly calls the action events to take place with the hovercraft
*/
void loop() {
  // put your main code here, to run repeatedly:
  //static PeriodicAction fsm_task(50, fsm_step);
  // static PeriodicAction imu_task(5, imu_update);
  // static PeriodicAction pd_task(10, pd_step);
  static PeriodicAction report_task(1000, report_step);
  static PeriodicAction camera_task(5, camera_step);
  // imu_task.step(); ALL NON-ESSENTIAL tasks commmented out for this project
  // pd_task.step(); //commented out for prevented IMU and motor use
  // fsm_task.step();
  camera_task.step();
  report_task.step();
}
