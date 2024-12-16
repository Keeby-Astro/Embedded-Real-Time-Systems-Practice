int counter = 0;  // Counter variable to track the LED sequence

void setup() {
  // Configure pin control registers for PORTB pins
  PORTB_PCR2 = PORT_PCR_MUX(0x1);  // Set pin B2 as GPIO
  PORTB_PCR3 = PORT_PCR_MUX(0x1);  // Set pin B3 as GPIO
  PORTB_PCR1 = PORT_PCR_MUX(0x1);  // Set pin B1 as GPIO
  PORTB_PCR0 = PORT_PCR_MUX(0x1);  // Set pin B0 as GPIO
  PORTB_PCR19 = PORT_PCR_MUX(0x1); // Set pin B19 as GPIO (input pin)

  // Configure pins B0, B1, B2, and B3 as outputs
  GPIOB_PDDR |= (0x0000F); // Set the first 4 bits of GPIOB as output
}

void loop() {
  // Continuously check the input pin (B19) for state
  if (GPIOB_PDIR & (0x80000)) { // Check if input pin B19 is HIGH (switch is open)
    // Determine the LED pattern based on the counter value
    switch (counter % 4) { // Cycle through 4 LED patterns
      case 0:
        GPIOB_PDOR |= (0x02); // Turn on LED connected to pin B1
        break;
      case 1:
        GPIOB_PDOR |= (0x01); // Turn on LED connected to pin B0
        break;
      case 2:
        GPIOB_PDOR |= (0x08); // Turn on LED connected to pin B3
        break;
      case 3:
        GPIOB_PDOR |= (0x04); // Turn on LED connected to pin B2
        break;
      default:
        GPIOB_PDOR |= (0x00); // Default case: turn off all LEDs
        break;
    }

    delay(1500); // Wait 1.5 seconds before changing the pattern
    counter += 1; // Increment the counter to cycle through patterns
    GPIOB_PDOR &= (0x00); // Turn off all LEDs before the next iteration
  } 
  else {
    // If the switch is closed (input pin B19 is LOW), turn off all LEDs
    GPIOB_PDOR = (0x00);
  }
}
