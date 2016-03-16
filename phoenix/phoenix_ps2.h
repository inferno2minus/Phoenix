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
#define WALK_MODE         0
#define ROTATE_MODE       1
#define TRANSLATE_MODE    2
#define SINGLELEG_MODE    3

PS2X    PS2;
bool    DoubleHeight;
bool    DoubleTravel;
uint8_t ControlMode;
int16_t BodyOffsetY;
int16_t BodyShiftY;

#endif
