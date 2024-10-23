#include "PeriodicAction.h"
const int sharpPin = A1;

void setup() {
  // put your setup code here, to run once:
  pinMode(sharpPin, INPUT);
  Serial.begin(115200);
  Serial.setTimeout(100000000);
  delay(1000);

}

void sensor_display_step() {
  //int val2 = analogRead(sharpPin);
  float val = read_distance();
  float finalVal = ((1023 - (float)val) /1023) * 60;
  Serial.printf("%d \n", finalVal);
}

float read_distance() {
  int val = analogRead(sharpPin);
  //float calcVal = (0.0003 * (pow(val, 4))) - (0.0538 * (pow(val, 3))) + (3.6406 * (pow(val, 2))) - (101.52 * val) + 1300; // Digital Signal
  float calcVal = 0.0004 * pow(val, 2) - (0.5028 * val ) + 170; // Centimeters
  if (calcVal < 0 || calcVal > 80) {
    calcVal = 80;
  }
  Serial.printf("Read: %d, Found: %f \n",  val, calcVal);
  return (calcVal);
}

void loop() {
  static PeriodicAction sensor_display_task(1000, sensor_display_step);
  //static PeriodicAction sensor_display_task(1000, read_distance);
  sensor_display_task.step();
  Serial.printf("%f \n", read_distance());

}
