/**
 * Project Lynxmotion Phoenix
 * Description: Phoenix tone file
 * Version: v1.0
 * Programmer: Jeroen Janssen (aka Xan)
 * Porting: Kompanets Konstantin (aka I2M)
 *
 * Hardware setup: Buzzer module
 */

#ifdef SOUND_MODE

void SoundNoTimer(unsigned int frequency, unsigned long duration) {
  volatile uint8_t *pin_port;
  volatile uint8_t pin_mask;

  //Set the pinMode as OUTPUT
  pinMode(BUZZER, OUTPUT);

  pin_port = portOutputRegister(digitalPinToPort(BUZZER));
  pin_mask = digitalPinToBitMask(BUZZER);

  long toggle_count = 2 * frequency * duration / 1000;
  long lusDelayPerHalfCycle = 1000000L / (frequency * 2);

  //if we are using an 8 bit timer, scan through prescaler to find the best fit
  while (toggle_count--) {
    //Toggle the pin
    *pin_port ^= pin_mask;
    //Delay a half cycle
    delayMicroseconds(lusDelayPerHalfCycle);
  }
  *pin_port &= ~(pin_mask); //Keep pin low after stop
}

void Sound(byte Notes, ...) {
  va_list ap;
  unsigned int duration;
  unsigned int frequency;
  va_start(ap, Notes);

  while (Notes > 0) {
    duration = va_arg(ap, unsigned int);
    frequency = va_arg(ap, unsigned int);
    SoundNoTimer(frequency, duration);
    Notes--;
  }
  va_end(ap);
}

#endif
