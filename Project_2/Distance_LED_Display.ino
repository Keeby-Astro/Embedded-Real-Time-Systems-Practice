/*
 * input: none
 * output: none
 * function: initializes all pins as input or output
 */
void setup() {
  // Initialize serial communication at a baud rate of 115200
  Serial.begin(115200);

  // Set a timeout for serial communication (very large value)
  Serial.setTimeout(100000000);

  // Brief delay for initialization stability
  delay(1000);

  // Configure PORTD pins for GPIO functionality
  PORTD_PCR2 = PORT_PCR_MUX(0x1); // Configure pin D2 as GPIO
  PORTD_PCR3 = PORT_PCR_MUX(0x1); // Configure pin D3 as GPIO
  PORTD_PCR4 = PORT_PCR_MUX(0x1); // Configure pin D4 as GPIO
  PORTD_PCR0 = PORT_PCR_MUX(0x1); // Configure pin D0 as GPIO
  PORTD_PCR7 = PORT_PCR_MUX(0x1); // Configure pin D7 as GPIO

  // Set pins D0, D2, D3, D4, and D7 as outputs
  GPIOD_PDDR |= (0x0009D); // Binary mask to enable output on the specified pins
}

/*
 * input: distance in meters
 * output: none
 * function: displays meter value to LED lights in segments of 0.0875m
 */
void display_distance(float dist) {
  // Clear all LED outputs initially
  GPIOD_PDOR = 0x00;

  // Map distance ranges to specific LED patterns
  if (dist <= 0.8 && dist > 0.7125) { 
    // Highest range of distance: turn on LED pattern 10000 (pin D7)
    GPIOD_PDOR |= (0x80); 
  } else if (dist <= 0.7125 && dist > 0.625) { 
    // Range 11000 (pins D7 and D4)
    GPIOD_PDOR |= (0x90); 
  } else if (dist <= 0.625 && dist > 0.5375) { 
    // Range 01000 (pin D4)
    GPIOD_PDOR |= (0x10); 
  } else if (dist <= 0.5375 && dist > 0.45) { 
    // Range 01100 (pins D4 and D3)
    GPIOD_PDOR |= (0x18); 
  } else if (dist <= 0.45 && dist > 0.3625) { 
    // Range 00100 (pin D3)
    GPIOD_PDOR |= (0x08); 
  } else if (dist <= 0.3625 && dist > 0.275) { 
    // Range 00110 (pins D3 and D2)
    GPIOD_PDOR |= (0x0C); 
  } else if (dist <= 0.275 && dist > 0.1875) { 
    // Range 00010 (pin D2)
    GPIOD_PDOR |= (0x04); 
  } else if (dist <= 0.1875 && dist > 0.1) { 
    // Range 00011 (pins D2 and D0)
    GPIOD_PDOR |= (0x05); 
  } else if (dist == 0.1) { 
    // Minimum distance: turn on LED pattern 00001 (pin D0)
    GPIOD_PDOR |= (0x01); 
  }

  // Delay to ensure the LED pattern is visible for 1.5 seconds
  delay(1500);
}

/*
 * input: none
 * output: none
 * function: reads distance from serial input and updates LED display
 */
void loop() {
  // Prompt the user to enter a distance in millimeters
  Serial.printf("Enter distance in millimeters\n");

  // Read the integer input from the serial monitor
  int i = Serial.parseInt();

  // Convert the input to meters (divide by 1000) and display on LEDs
  display_distance(((float)i) / 1000);
}
