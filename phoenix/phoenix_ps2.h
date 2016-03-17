/**
 * Project:     Lynxmotion Phoenix
 * Description: Phoenix control file
 * Version:     v2.6
 * Author:      Jeroen Janssen (aka Xan)
 *              Kåre Halvorsen (aka Zenta)
 *              Kompanets Konstantin (aka I2M)
 */

#ifndef PHOENIX_PS2_H
#define PHOENIX_PS2_H

#include <PS2X.h>

//Control mode
enum {
  WALK_MODE,
  ROTATE_MODE,
  TRANSLATE_MODE,
  SINGLELEG_MODE
};

PS2X     PS2;
bool     DoubleHeight;
bool     DoubleTravel;
uint8_t  ControlMode;

//Body position
int16_t  BodyOffsetY;
int16_t  BodyShiftY;

//Timing
uint8_t  InputDelayTime;
uint16_t PrevSpeedControl;
uint16_t SpeedControl;

#endif
