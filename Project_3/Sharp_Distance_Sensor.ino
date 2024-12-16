#include "PeriodicAction.h"  // Include the library for periodic actions

const int sharpPin = A1;  // Define the analog pin for the Sharp distance sensor

void setup() {
  // Initialize the Sharp sensor pin as input
  pinMode(sharpPin, INPUT);

  // Initialize serial communication at a baud rate of 115200
  Serial.begin(115200);

  // Set a very large timeout for serial communication
  Serial.setTimeout(100000000);

  // Brief delay for stability during initialization
  delay(1000);
}

/*
 * Function: sensor_display_step
 * Input: none
 * Output: none
 * Description: Reads the distance from the sensor, calculates a scaled value, 
 *              and prints the result via serial communication.
 */
void sensor_display_step() {
  // Read the distance using the Sharp sensor
  float val = read_distance();

  // Scale the value for display (convert to a custom unit)
  float finalVal = ((1023 - (float)val) / 1023) * 60;

  // Print the scaled value to the serial monitor
  Serial.printf("%d \n", finalVal);
}

/*
 * Function: read_distance
 * Input: none
 * Output: float - calculated distance in centimeters
 * Description: Reads the analog value from the Sharp sensor, applies a 
 *              polynomial function for calibration, and returns the distance.
 */
float read_distance() {
  // Read the analog value from the Sharp sensor
  int val = analogRead(sharpPin);

  // Apply a quadratic formula to convert the analog reading to a distance
  float calcVal = 0.0004 * pow(val, 2) - (0.5028 * val) + 170;

  // Clamp the calculated distance to the range [0, 80] centimeters
  if (calcVal < 0 || calcVal > 80) {
    calcVal = 80;
  }

  // Print the raw sensor value and calculated distance to the serial monitor
  Serial.printf("Read: %d, Found: %f \n", val, calcVal);

  // Return the calculated distance
  return calcVal;
}

/*
 * Function: loop
 * Input: none
 * Output: none
 * Description: Continuously runs the periodic action for reading and displaying 
 *              sensor data at a specified interval (1000ms).
 */
void loop() {
  // Create a periodic action to execute sensor_display_step every 1000ms
  static PeriodicAction sensor_display_task(1000, sensor_display_step);

  // Execute the periodic task
  sensor_display_task.step();

  // Optionally, read and print the distance for debugging or additional logic
  Serial.printf("%f \n", read_distance());
}
