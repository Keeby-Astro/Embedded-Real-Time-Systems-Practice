#include <PWMServo.h>
#include "project.h" //brings enum vals in
#include "PeriodicAction.h"

//#define PI 3.1415926535897932384626433832795
#define switch_pin 0 //following lines give aliases to pin numbers
#define m1_clockwise 1
#define m1_c_clockwise 2
#define m1_pulsewidth 3
#define m2_clockwise 22
#define m2_c_clockwise 23
#define m2_pulsewidth 21
#define m3_clockwise 7
#define m3_c_clockwise 8
#define m3_pulsewidth 9
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
  const float negative_gain[3] = {1.1, 1.1, 1.2}; //negative gain to be used to multiply negative motor values
  const float positive_gain[3] = {1.0, 1.0, 1.1}; //used to correct weak motors in the normal direction
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

  Serial.printf("Power %f Motor %d \n", val[i], motorPulse);
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
  float R = 1; //effective radius of hovercraft
  F1 = ((1/(2*cos(angle_radians(30)))) * fx) + ((-1/(2+(2*sin(angle_radians(30)))))*fy) + ((-1/(2*R*(1+(sin(angle_radians(30))))))*torque);
  F2 = ((1/(2*cos(angle_radians(30)))) * fx) + ((1/(2+(2*sin(angle_radians(30)))))*fy) + ((1/(2*R*(1+(sin(angle_radians(30))))))*torque);
  F3 = ((-1/(1+(1*sin(angle_radians(30)))))*fy) + ((sin(angle_radians(30))/(R*(1+(sin(angle_radians(30))))))*torque);
  float forces[] = {F1, F2, F3};
  set_motors(forces); //sets the motor power with the calculated new forces
  
}








/*
 * input: none
 * output: none
 * function: direction vehicle control through state machine behavior
 */
void fsm_step(){
  static int timingTotal = 0; //used for totalvehicle time
  static int refTime = timingTotal; //used for timing reference in wait stages
  const float fx = 50; //constant describing fx forces whenver utilized in code
  const float fy = 50; //constant describing fy forces whenver utilized in code
  const float torque = 50; //constant describing torque forces whenver utilized in code
  const float motorPow = 84; //constant describing lift fan force (duty cycle = motorPow/255)
  static boolean motor_on = false; //describes state of lift fan
  static State current = STATE_START; //initialized in do nothing state
  static State prevState = STATE_START; //used for state transition determination based on specified steps (in instrustions)
  switch (current) { //switch for state transitions
    case STATE_START: //initial state
       fan.write(0);
       set_hovercraft_forces(0,0,0);
       if(digitalRead(switch_pin == HIGH)){
        current = STATE_ACTIVATE; //vehicle was turned on
       }
       else current = STATE_START;
       break;
       
       case STATE_ACTIVATE: //sets lift motor to motorPow
       motor_on = true;
       fan.write(motorPow);
       if(prevState == STATE_START) current = STATE_WAIT_15; //based on step 3
       else current = STATE_WAIT_10; //based on steps 9 and 15
       prevState = STATE_ACTIVATE;
       timingTotal += 1;
       break;
       
       case STATE_WAIT_15: //holds position for 15 seconds
       if(prevState != STATE_WAIT_15){ //first iteration of state sets reference Time
        refTime = timingTotal;
        prevState = STATE_WAIT_15;
       }
       if((timingTotal - refTime) <= 300){  //checks for 15 seconds (300 * 0.050)
        timingTotal += 1;
        current = STATE_WAIT_15;
        if(motor_on){ //if fan motor on continue power
          fan.write(motorPow);
        }
       }
       else {
        timingTotal +=1;
        if(motor_on){
          current = STATE_TORQUE_P; //based on step 4
          fan.write(motorPow);
        }
        else{
          current = STATE_ACTIVATE; //based on step 14
        }
       }
       break;
       
       case STATE_TORQUE_P: //generates a positive motor torque
       fan.write(motorPow);
       if(prevState != STATE_TORQUE_P){ //sets reference time in first iteration
         refTime = timingTotal;
         prevState = STATE_TORQUE_P;
       }
       if(timingTotal - refTime <= 200){
        set_hovercraft_forces(0,0,torque); //spins for 10 seconds
        timingTotal += 1;
        current = STATE_TORQUE_P;
       }
       else{
        current = STATE_TORQUE_N; //based on step 5
        timingTotal += 1;
        set_hovercraft_forces(0,0,0); //reset motor powers
       }
       break;

       case STATE_TORQUE_N: //generates a negative torque
        fan.write(motorPow);
       if(prevState != STATE_TORQUE_N){ //sets reference time
         refTime = timingTotal;
         prevState = STATE_TORQUE_N;
       }
       if(timingTotal - refTime <= 200){ //spins for 10 seconds
        set_hovercraft_forces(0,0,-torque);
        timingTotal += 1;
        current = STATE_TORQUE_N;
       }
       else{
        current = STATE_DEACTIVATE; //based on step 6
        timingTotal += 1;
        set_hovercraft_forces(0,0,0);
       }
       break;

       case STATE_DEACTIVATE: //shuts down lift fan
       fan.write(0);
       motor_on = false; //resets this value to indicate motor is off
       if(prevState != STATE_LEFT){
       current = STATE_WAIT_15; //based on step 7
       prevState = STATE_DEACTIVATE;
       }
       else{
        prevState = STATE_START; //based on step 18, would return vehicle to wait stage
        current = STATE_START; //resets all static variables
        timingTotal = 0; 
        refTime = 0;
       }
       break;

       case STATE_WAIT_10: //vehicle position remains constant for 10 seconds
       fan.write(motorPow); //based on steps fan motor is always on
       set_hovercraft_forces(0,0,0); //based on steps motors always off in this case
       if(prevState != STATE_WAIT_10){ //sets reference time
        refTime = timingTotal;
        prevState = STATE_WAIT_10;
       }
       
       if((timingTotal - refTime) <= 200){ //waits 10 seconds
        timingTotal += 1;
        current = STATE_WAIT_10;    
       }
      else if(timingTotal>=1000 && timingTotal <= 1100){ //based on totalTimes progress through step 9
          current = STATE_FORWARD;
          timingTotal += 1;
       }
       else{
          current = STATE_RIGHT; //should execute at step 15
          timingTotal += 1;
       }
       break;

       case STATE_FORWARD: //Affects fx forces, sets vehicle for forward thrust
       fan.write(motorPow); 
       if(prevState != STATE_FORWARD){ //sets reference time
         refTime = timingTotal;
         prevState = STATE_FORWARD;
       }
       if(timingTotal - refTime <= 200){ //runs for 10 seconds
        set_hovercraft_forces(fx,0,0); //sets fx force
        timingTotal += 1;
        current = STATE_FORWARD;
       }
       else{
        current = STATE_BACK; //based on step 11
        timingTotal += 1;
        set_hovercraft_forces(0,0,0);
       }
       break;

      case STATE_BACK: //Affects fx forces, sets vehicle for backward thrust
      fan.write(motorPow);
      if(prevState != STATE_BACK){ //sets reference time
         refTime = timingTotal;
         prevState = STATE_BACK;
       }
       if(timingTotal - refTime <= 200){ //operates for 10 seconds
        set_hovercraft_forces(-fx,0,0); //negative fx movement
        timingTotal += 1;
        current = STATE_BACK;
       }
       else{
        current = STATE_DEACTIVATE; //based on step 12
        timingTotal += 1;
        set_hovercraft_forces(0,0,0);
       }
       break;

       case STATE_RIGHT: //Affects fy forces, sets vehicle for right thrust
       fan.write(motorPow);
       if(prevState != STATE_RIGHT){ //sets reference time
         refTime = timingTotal;
         prevState = STATE_RIGHT;
       }
       if(timingTotal - refTime <= 200){ //operates in case for 10 seconds
        set_hovercraft_forces(0,fy,0); //sets positive fy (right movement)
        timingTotal += 1;
        current = STATE_RIGHT;
       }
       else{
        current = STATE_LEFT; //based on step 17
        timingTotal += 1;
        set_hovercraft_forces(0,0,0);
       }
       break;

      case STATE_LEFT: //Affects fy forces, sets vehicle for left thrust
      fan.write(motorPow);
      if(prevState != STATE_LEFT){ //sets reference time in first iteration
         refTime = timingTotal;
         prevState = STATE_LEFT;
       }
       if(timingTotal - refTime <= 200){ //operates for 10 seconds
        set_hovercraft_forces(0,-fy,0); //sets negative fy thrust
        timingTotal += 1;
        current = STATE_LEFT;
       }
       else{
        current = STATE_DEACTIVATE; //based on step 18
        timingTotal += 1;
        set_hovercraft_forces(0,0,0);
       }
       break;
  }
}




















/*
 * input: none
 * output: none
 * function: Continously calls fsm_step with 50 ms intervals
 */
void loop() {
  // put your main code here, to run repeatedly:
  static PeriodicAction fsm_task(50, fsm_step);
  fsm_task.step();

}
