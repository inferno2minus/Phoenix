/**
 * Project Lynxmotion Phoenix
 * Description: Phoenix tone file
 * Version: v1.0
 * Programmer: Jeroen Janssen (aka Xan)
 *             Kompanets Konstantin (aka I2M)
 *
 * Hardware setup: Buzzer module
 */

void Tone(byte pin, unsigned int frequency, unsigned long duration) {
  volatile uint8_t *pin_port;
  volatile uint8_t pin_mask;

  //Set the pinMode as OUTPUT
  pinMode(pin, OUTPUT);

  pin_port = portOutputRegister(digitalPinToPort(pin));
  pin_mask = digitalPinToBitMask(pin);

  long toggle_count = 2 * frequency * duration / 1000;
  long half_cycle = 1000000L / (frequency * 2);

  //if we are using an 8 bit timer, scan through prescaler to find the best fit
  while (toggle_count--) {
    //Toggle the pin
    *pin_port ^= pin_mask;
    //Delay a half cycle
    delayMicroseconds(half_cycle);
  }
  *pin_port &= ~(pin_mask); //Keep pin low after stop
}

void Sound(byte notes, ...) {
  va_list ap;
  unsigned int duration;
  unsigned int frequency;
  va_start(ap, notes);

  while (notes > 0) {
    duration = va_arg(ap, unsigned int);
    frequency = va_arg(ap, unsigned int);
    Tone(BUZZER, frequency, duration);
    notes--;
  }
  va_end(ap);
}
