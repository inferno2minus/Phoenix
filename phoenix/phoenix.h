/**
 * Project:     Lynxmotion Phoenix
 * Description: Phoenix software
 * Version:     v2.6
 * Author:      Jeroen Janssen (aka Xan)
 *              Kåre Halvorsen (aka Zenta)
 *              Kompanets Konstantin (aka I2M)
 */

#ifndef PHOENIX_H
#define PHOENIX_H

#include <SSC32.h>
#include <MiniTone.h>
#include <PrintfSerial.h>
#include "phoenix_cfg.h"

//Software version
#define VERSION       "2.6.9"

//Math constants
#define PI            3.1415926535897932384626433832795
#define DEG_IN_RAD    0.0174532925199432957692369076848
#define RAD_IN_DEG    57.295779513082320876798154814105

//Legs sign
#define SIGN(leg)     ((leg) <= (2) ? (-1) : (1))

//Legs constants
enum {
  RR, //Right Rear
  RM, //Right Middle
  RF, //Right Front
  LR, //Left Rear
  LM, //Left Middle
  LF, //Left Front
  NOT_SELECTED = 0xFF
};

//Structs
typedef struct {
  float   Cos;
  float   Sin;
} trig;

typedef struct {
  uint16_t X;
  uint16_t Y;
  uint16_t Z;
} point3d;

typedef struct {
  uint16_t Y;
} ordinate;

typedef struct {
  point3d Pos;
  ordinate Rot;
} gait3d;

typedef struct {
  uint8_t NrLiftedPos;    //Number of positions that a single leg is lifted (1-5)
  uint8_t FrontDownPos;   //Where the leg should be put down to ground
  uint8_t LiftDivFactor;  //Result of the operation (NrLiftedPos == 5 ? 4 : 2)
  uint8_t HalfLiftHeight; //How high to lift at halfway up
  uint8_t TLDivFactor;    //Number of steps that a leg is on the floor while walking
  uint8_t StepsInGait;    //Number of steps in gait
  uint8_t NomGaitSpeed;   //Nominal speed of the gait
  uint8_t GaitLegNr[6];   //Init position of the leg (RR, RM, RF, LR, LM, LF)
} gait;

//Gaits type
const gait Gaits[] = {
  { 3,  2,  2,  3,  8, 12, 70, {  7, 11,  3,  1,  5,  9 } }, //Ripple 12 steps
  { 2,  1,  2,  1,  4,  6, 60, {  1,  4,  1,  4,  1,  4 } }, //Tripod 6 steps
  { 3,  2,  2,  3,  4,  8, 70, {  1,  5,  1,  5,  1,  5 } }, //Tripod 8 steps
  { 3,  2,  2,  3,  8, 12, 60, {  5, 10,  3, 11,  4,  9 } }, //Tripod 12 steps
  { 5,  3,  4,  1, 10, 16, 60, {  6, 13,  4, 14,  5, 12 } }, //Tripod 16 steps
  { 3,  2,  2,  3, 20, 24, 70, { 13, 17, 21,  1,  5,  9 } }  //Wave 24 steps
};

const uint8_t GaitsNumber = sizeof(Gaits)/sizeof(Gaits[0]);

//Build tables for leg configuration like I/O and Min/Max values to easy access values using a for loop
//Constants are still defined as single values in the cfg file to make it easy to read/configure

//SSC pin numbers
const uint8_t CoxaPin[]  PROGMEM = { RRCoxaPin,  RMCoxaPin,  RFCoxaPin,  LRCoxaPin,  LMCoxaPin,  LFCoxaPin  };
const uint8_t FemurPin[] PROGMEM = { RRFemurPin, RMFemurPin, RFFemurPin, LRFemurPin, LMFemurPin, LFFemurPin };
const uint8_t TibiaPin[] PROGMEM = { RRTibiaPin, RMTibiaPin, RFTibiaPin, LRTibiaPin, LMTibiaPin, LFTibiaPin };

//Min/Max values
const int16_t CoxaMin[]  PROGMEM = { RRCoxaMin,  RMCoxaMin,  RFCoxaMin,  LRCoxaMin,  LMCoxaMin,  LFCoxaMin  };
const int16_t CoxaMax[]  PROGMEM = { RRCoxaMax,  RMCoxaMax,  RFCoxaMax,  LRCoxaMax,  LMCoxaMax,  LFCoxaMax  };
const int16_t FemurMin[] PROGMEM = { RRFemurMin, RMFemurMin, RFFemurMin, LRFemurMin, LMFemurMin, LFFemurMin };
const int16_t FemurMax[] PROGMEM = { RRFemurMax, RMFemurMax, RFFemurMax, LRFemurMax, LMFemurMax, LFFemurMax };
const int16_t TibiaMin[] PROGMEM = { RRTibiaMin, RMTibiaMin, RFTibiaMin, LRTibiaMin, LMTibiaMin, LFTibiaMin };
const int16_t TibiaMax[] PROGMEM = { RRTibiaMax, RMTibiaMax, RFTibiaMax, LRTibiaMax, LMTibiaMax, LFTibiaMax };

//Body offsets
const int16_t OffsetX[]  PROGMEM = { RROffsetX,  RMOffsetX,  RFOffsetX,  LROffsetX,  LMOffsetX,  LFOffsetX  };
const int16_t OffsetZ[]  PROGMEM = { RROffsetZ,  RMOffsetZ,  RFOffsetZ,  LROffsetZ,  LMOffsetZ,  LFOffsetZ  };

//Default leg angle
const int16_t LegAngle[] PROGMEM = { RRLegAngle, RMLegAngle, RFLegAngle, LRLegAngle, LMLegAngle, LFLegAngle };

//Start positions for the leg
const int16_t InitPosX[] PROGMEM = { RRInitPosX, RMInitPosX, RFInitPosX, LRInitPosX, LMInitPosX, LFInitPosX };
const int16_t InitPosY[] PROGMEM = { RRInitPosY, RMInitPosY, RFInitPosY, LRInitPosY, LMInitPosY, LFInitPosY };
const int16_t InitPosZ[] PROGMEM = { RRInitPosZ, RMInitPosZ, RFInitPosZ, LRInitPosZ, LMInitPosZ, LFInitPosZ };

//Angles
float    CoxaAngle[6];
float    FemurAngle[6];
float    TibiaAngle[6];

//Body position
point3d  BodyPos;
point3d  BodyRot;
point3d  BodyFKPos;

//Timing
uint16_t PrevSSCTime;
uint16_t SSCTime;
uint32_t StartTime;

//Power
bool     PrevHexOn;
bool     HexOn;

//Balance
bool     BalanceMode;
point3d  TotalBalance;
point3d  TotalTranslate;

//Single leg
bool     SingleLegHold;
uint8_t  PrevSelectedLeg = NOT_SELECTED;
uint8_t  SelectedLeg = NOT_SELECTED;
point3d  LegPos[6];
point3d  SingleLegPos;

//Gait
gait     GaitCurrent;
bool     GaitInMotion;
bool     WalkStatus;
uint8_t  ExtraCycle;
uint8_t  GaitStep = 1;
uint8_t  GaitType;
uint8_t  LegLiftHeight = 50;
gait3d   Gait[6];
point3d  TravelLength;

#ifdef DEBUG_MODE
bool     PrevWalkStatus;
bool     DebugOutput;
#endif

#ifdef SOUND_MODE
MiniTone Sound;
#endif

#ifdef DEBUG_MODE
PrintfSerial DBGSerial(0, 1);
#define DebugPrint(msg, ...) DBGSerial.printf(msg, ##__VA_ARGS__)
#else
#define DebugPrint(msg, ...)
#endif
SSC32 Servo(SSC_RX, SSC_TX);

#endif
