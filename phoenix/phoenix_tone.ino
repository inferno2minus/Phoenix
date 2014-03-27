/**
 * Project Lynxmotion Phoenix
 * Description: Phoenix tone file
 * Version: v1.0
 * Programmer: Jeroen Janssen (aka Xan)
 * Porting: Kompanets Konstantin (aka I2M)
 *
 * Hardware setup: Buzzer module
 */

#ifdef cBUZZER

//Quick and dirty tone function to try to output a frequency to a speaker for some simple sounds
void SoundNoTimer(unsigned long duration, unsigned int frequency) {
  volatile uint8_t *pin_port;
  volatile uint8_t pin_mask;

  long toggle_count = 0;
  long lusDelayPerHalfCycle;

  //Set the pinMode as OUTPUT
  pinMode(cBUZZER, OUTPUT);

  pin_port = portOutputRegister(digitalPinToPort(cBUZZER));
  pin_mask = digitalPinToBitMask(cBUZZER);

  toggle_count = 2 * frequency * duration / 1000;
  lusDelayPerHalfCycle = 1000000L / (frequency * 2);

  //if we are using an 8 bit timer, scan through prescaler to find the best fit
  while (toggle_count--) {
    //Toggle the pin
    *pin_port ^= pin_mask;
    //Delay a half cycle
    delayMicroseconds(lusDelayPerHalfCycle);
  }
  *pin_port &= ~(pin_mask); //Keep pin low after stop
}

void MSound(byte cNotes, ...) {
  va_list ap;
  unsigned int uDur;
  unsigned int uFreq;
  va_start(ap, cNotes);

  while (cNotes > 0) {
    uDur = va_arg(ap, unsigned int);
    uFreq = va_arg(ap, unsigned int);
    SoundNoTimer(uDur, uFreq);
    cNotes--;
  }
  va_end(ap);
}

#endif
