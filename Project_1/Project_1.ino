int counter = 0;
void setup() {
  // put your setup code here, to run once:
  PORTB_PCR2 = PORT_PCR_MUX(0x1);
  PORTB_PCR3 = PORT_PCR_MUX(0x1);
  PORTB_PCR1 = PORT_PCR_MUX(0x1);
  PORTB_PCR0 = PORT_PCR_MUX(0x1);
  PORTB_PCR19 = PORT_PCR_MUX(0x1); //will be assigned to input pin
  GPIOB_PDDR |= (0x0000F);



}

void loop() {
  // put your main code here, to run repeatedly:

  if (GPIOB_PDIR & (0x80000)) {
    //Switch is open in this case
    switch (counter % 4) {
      case 0:
        GPIOB_PDOR |= (0x02);
        break;
      case 1:
        GPIOB_PDOR |= (0x01);
        break;
      case 2:
        GPIOB_PDOR |= (0x08);
        break;
      case 3:
        GPIOB_PDOR |= (0x04);
        break;
      default:
        GPIOB_PDOR |= (0x00);
        break;
    }
    delay(1500);
    counter += 1;
    GPIOB_PDOR &= (0x00);
  }
  else GPIOB_PDOR = (0x00);
}
