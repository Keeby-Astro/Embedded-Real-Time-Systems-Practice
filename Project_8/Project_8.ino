/*
 * Project 8
 * Group 4
 * Submitted: 4/13/23
 * Authors: Kyler Clark, Ashley Yi, Caden Matthews
 */
#include <quaternionFilters.h>
#include <MPU9250.h>
#include <PWMServo.h>
#include "project.h" //brings enum vals in
#include "PeriodicAction.h"
#include <ImuUtils.h>

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
PWMServo fan; // create servo object to control the fan
const int CENTRAL_FAN_PWM = 36; 
/*
 * input: none
 * output: none
 * function: initializes the Brushless lift motor for power commands
 */
void fan_setup() {
fan.attach(CENTRAL_FAN_PWM); // attaches the fan to specified
// Arduino pin to the object
delay(100);
fan.write(20); // write low throttle
delay(3000);
}
/*
 * input: none
 * output: none
 * Function: intitalizes all used pins as input/outputs and begins Serial monitor
 */
void setup() {
  fan_setup(); //sets the brushless motor power
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
  delay(1000);
}

/*
 * input: value - the actual value to be bounded, min_value - the minimum bound, max_value - the maximum bound
 * output: bounded value between min and max bounds
 * function: takes a value and if outside of bounds specified, bounds it
 */
float bound(float value, float min_value, float max_value){
  if(value < min_value){
    return min_value; //value below min value coersion
  }
  else if(value > max_value){
    return max_value; //value above max value coersion
  }
  else{
    return value;
  }
}

/*
 * input: val[3] - desired motor power values in order of indices is left, right, back
 * output: none
 * Function: sets the motor directions and then sets motor speed by writing to motor speed function, setting motors in order of left, right, back
 */
void set_motors(float val[3]){
  const float negative_gain[3] = {2.2, 2.2, 2.2}; //negative gain to be used to multiply negative motor values
  const float positive_gain[3] = {1.2, 1.2, 1.1}; //used to correct weak motors in the normal direction
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
    analogWrite(motorPulse, abs(bound(val[i], -255, 255))); //power to motor absolute value taken because power must be > 0. (Bounds Can be Manipulated for Power Tuning)

 // Serial.printf("Power %f Motor %d \n", val[i], i);
  }
  
}

/*
 * input: int deg - degree representation of an angle
 * output: converted float of the radian value of an angle
 * function: takes an angle and returns equivalent radian representation
 */
float angle_radians(int deg){
  return (((float) deg) *(PI/180.00));
}

/*
 * input: float fx - desired forces in x direction, float fy - desired forces in y direction, float torque - desired torque of vehicle
 * output: F1 - motor one forces, F2 - motor two forces, F3 - motor three forces
 * function: Takes desired fx, fy, and torque specified and converts that to power vals of the three attached motors
 */
void set_hovercraft_forces(float fx, float fy, float torque){
  float F1;
  float F2;
  float F3;
  float R = 5.5; //effective radius of hovercraft in inches
  F1 = ((1/(2*cos(angle_radians(30)))) * fx) + ((-1/(2+(2*sin(angle_radians(30)))))*fy) + ((-1/(2*R*(1+(sin(angle_radians(30))))))*torque);
  F2 = ((1/(2*cos(angle_radians(30)))) * fx) + ((1/(2+(2*sin(angle_radians(30)))))*fy) + ((1/(2*R*(1+(sin(angle_radians(30))))))*torque);
  F3 = ((-1/(1+(1*sin(angle_radians(30)))))*fy) + ((sin(angle_radians(30))/(R*(1+(sin(angle_radians(30))))))*torque);
  float forces[] = {F1, F2, F3};
  set_motors(forces); //sets the motor power with the calculated new forces
  
}
 /*
  * input: none
  * output: none
  * function: sets up the state machine behavior for vehicle function
  */
void fsm_step() {
  static int timingTotal = 0; //used for totalvehicle time
  static int refTime = timingTotal; //used for timing reference in wait stages
  const float motorPow = 84; //constant describing lift fan force (duty cycle = motorPow/255)
  static State current = STATE_START; //initialized in do nothing state
  static State prevState = STATE_START; //used for state transition determination based on specified steps (in instrustions)

  switch (current) { //switch for state transitions
    case STATE_START: //initial state
       fan.write(0);
       set_hovercraft_forces(0,0,0);
       if(digitalRead(switch_pin == HIGH)){
        current = STATE_ACTIVATE; //vehicle was turned on
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
       if(prevState != STATE_WAIT_30){ //first iteration of state sets reference Time
        refTime = timingTotal;
        prevState = STATE_WAIT_30;
       }
       if((timingTotal - refTime) <= 600){  //checks for 15 seconds (300 * 0.050)
        timingTotal += 1;
        current = STATE_WAIT_30;
        //Serial.printf("Going to STATE WAIT 30\n");
       }
       else current = STATE_DEACTIVATE;
       break;
       case STATE_DEACTIVATE:
       fan.write(0);
       prevState = STATE_START;
       current = STATE_START;
       break;  
       default:
       current = STATE_START;
       break;
       
  }
}

float compute_error(float theta, float heading_goal){
  float diff = heading_goal - theta;
  if(diff >= 0 && diff <= 180){
    return diff;
  }
  else{
    return diff - 360;
  }
}

float db_clip(float error, float deadband, float saturation){
  if(error <= deadband && error >= -deadband){
    returned 0.0;
  }

  else if (abs(error) >= saturation){ //absolute value case here to handle potential negatives
    return saturation - deadband;
  }
  else if (abs(error) <= saturation && abs(error) >= deadband){ //same as above absolute value handles negatives
    return (error/(saturation)) * (saturation - deadband);
  }
  else{
    return 0.0;
  }
}

/*
 * input: none
 * output: none
 * function: sets the hovercraft forces based on the IMU z rotation axis to counter the rotation
 */
void pd_step(){
  float Kd = 4; //required gain value
  float fx = 0;
  float fy = 0;
  float theta = 0;
  float rotation = 0;
  float heading_error = compute_error(theta, rotation);
  float modified_error = db_clip(heading_error, 15, 45);
  float torque = Kd * modified_error; //set torque for counter balancing
 // Serial.printf("Torque: %f \n", torque);
  set_hovercraft_forces(fx, fy, torque);
}

/*
 * input: one
 * output: none
 * function: prints the current rotation about the Z axis
 */
void report_step(){
  Serial.printf("Rotation Rate: %f \n", IMU.gz);
}
void loop() {
  // put your main code here, to run repeatedly:
 static PeriodicAction fsm_task(50, fsm_step);
 static PeriodicAction imu_task(5, imu_update);
 static PeriodicAction pd_task(10, pd_step);
 static PeriodicAction report_task(1000, report_step);
 imu_task.step();
  pd_task.step();
  fsm_task.step();
  report_task.step();
}
