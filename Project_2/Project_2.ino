/*
 * Project 2
 * Group 4
 * Submitted: 2/16/23
 * Authors: Kyler Clark, Ashley Yi, Caden Matthews
 */

 /*
  * input: none
  * output: none
  * function: initializes all pins as input or output
  */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(100000000);
  delay(1000);
  PORTD_PCR2 =PORT_PCR_MUX(0x1); 
  PORTD_PCR3 =PORT_PCR_MUX(0x1);
  PORTD_PCR4 =PORT_PCR_MUX(0x1);
  PORTD_PCR0 =PORT_PCR_MUX(0x1);
  PORTD_PCR7 =PORT_PCR_MUX(0x1);
  GPIOD_PDDR |= (0x0009D);
}
/*
 * input: distance in meters 
 * output: none
 * function: displays meter value to LED lights in segments of 0.0875m 
 */
void display_distance(float dist){
  GPIOD_PDOR = 0x00;
  if(dist <= 0.8 & dist > 0.7125){ //highest range of distance measurements written to 10000 LED
    GPIOD_PDOR |= (0x80);
  }
  else if(dist <= 0.7125 & dist > 0.625){ //range of distance LED's at 11000
    GPIOD_PDOR |= (0x90);
  }
  else if(dist <= 0.625 & dist > 0.5375){ //range of distance LED's at 01000
    GPIOD_PDOR |= (0x10);
  }
  else if( dist <= 0.5375 & dist > 0.45){ //range of distance LED's at 01100
    GPIOD_PDOR |= (0x18);
  }
  else if(dist <= 0.45 & dist > 0.3625){ //range of distance LED's at 00100
    GPIOD_PDOR |= (0x08);
  }
  else if(dist <= 0.3625 & dist > 0.275){ //range of distance LED's at 00110
    GPIOD_PDOR |= (0x0C);
  }
  else if( dist <= 0.275 & dist > 0.1875){ //range of distance LED's at 00010
    GPIOD_PDOR |= (0x04);
  }
  else if(dist<= 0.1875 & dist > 0.1){ //range of distance LED's at 00011
    GPIOD_PDOR |= (0x05);
  }
  else if(dist == 0.1){ //final minimum value with LED's in position 00001
    GPIOD_PDOR |= (0x01);
  }
  delay(1500); //delay to allow for LED's to be viewed accurately
}

/*
 * input: none
 * output: none
 * function: writes LED values to true as needed.
 */
void loop() {
  // put your main code here, to run repeatedly:

 Serial.printf("Enter distance in millimeters", "\n");
 int i = Serial.parseInt();
 display_distance(((float)i) / 1000);
 

}
