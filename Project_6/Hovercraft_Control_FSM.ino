#include <PWMServo.h>         // Include PWMServo library for controlling brushless motor
#include "project.h"          // Include custom header file for enumerations and configurations
#include "PeriodicAction.h"   // Include library for scheduling periodic actions

//#define PI 3.1415926535897932384626433832795
#define switch_pin 0          // Pin connected to the control switch
#define m1_clockwise 1        // Motor 1 clockwise control pin
#define m1_c_clockwise 2      // Motor 1 counter-clockwise control pin
#define m1_pulsewidth 3       // Motor 1 PWM speed control pin
#define m2_clockwise 22       // Motor 2 clockwise control pin
#define m2_c_clockwise 23     // Motor 2 counter-clockwise control pin
#define m2_pulsewidth 21      // Motor 2 PWM speed control pin
#define m3_clockwise 7        // Motor 3 clockwise control pin
#define m3_c_clockwise 8      // Motor 3 counter-clockwise control pin
#define m3_pulsewidth 9       // Motor 3 PWM speed control pin

PWMServo fan;                 // Create PWMServo object to control the central fan
const int CENTRAL_FAN_PWM = 36; // Pin for controlling the central fan

/*
 * Initializes the brushless lift motor (fan) with a low throttle command.
 */
void fan_setup() {
  fan.attach(CENTRAL_FAN_PWM); // Attach the fan to the specified PWM pin
  delay(100);
  fan.write(20);               // Send a low throttle signal
  delay(3000);                 // Wait for motor to initialize
}

/*
 * Sets up all pins as inputs/outputs and initializes the serial monitor.
 */
void setup() {
  fan_setup();                 // Set up the brushless motor power
  pinMode(switch_pin, INPUT);  // Set switch pin as input

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

  Serial.begin(115200);        // Begin serial communication at 115200 baud rate
  Serial.setTimeout(100000000); // Set a long timeout for serial communication
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
    analogWrite(motorPulse, abs(bound(val[i], -64, 64)));
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
  float R = 1; // Effective radius of the hovercraft

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
  // FSM implementation with states and transitions (refer to code for full details)
}

/*
 * Main loop: Repeatedly calls the FSM step function at 50 ms intervals.
 */
void loop() {
  static PeriodicAction fsm_task(50, fsm_step);
  fsm_task.step();
}

/*
 * Implements the finite state machine (FSM) for hovercraft control.
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
 * Main loop: Repeatedly calls the FSM step function at 50 ms intervals.
 */
void loop() {
  static PeriodicAction fsm_task(50, fsm_step);
  fsm_task.step();
}
