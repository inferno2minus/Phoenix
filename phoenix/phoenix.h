/**
 * Project:     Lynxmotion Phoenix
 * Description: Phoenix software
 * Version:     v2.5
 * Author:      Jeroen Janssen (aka Xan)
 *              Kåre Halvorsen (aka Zenta)
 *              Kompanets Konstantin (aka I2M)
 */

#ifndef PHOENIX_H
#define PHOENIX_H

//Changing the sign, depending on the legs
#define sign(x) ((x) <= (2) ? (-1) : (1))

//Legs constants
#define RR              0 //Right Rear
#define RM              1 //Right Middle
#define RF              2 //Right Front
#define LR              3 //Left Rear
#define LM              4 //Left Middle
#define LF              5 //Left Front
#define NOT_SELECTED    255

//Mathematical constants
#define PI              3.1415926535897932384626433832795
#define DEG_IN_RAD      0.0174532925199432957692369076848
#define RAD_IN_DEG      57.295779513082320876798154814105

//Build tables for leg configuration like I/O and Min/Max values to easy access values using a for loop
//Constants are still defined as single values in the cfg file to make it easy to read/configure

//SSC pin numbers
const uint8_t CoxaPin[] PROGMEM = { RRCoxaPin, RMCoxaPin, RFCoxaPin, LRCoxaPin, LMCoxaPin, LFCoxaPin };
const uint8_t FemurPin[] PROGMEM = { RRFemurPin, RMFemurPin, RFFemurPin, LRFemurPin, LMFemurPin, LFFemurPin };
const uint8_t TibiaPin[] PROGMEM = { RRTibiaPin, RMTibiaPin, RFTibiaPin, LRTibiaPin, LMTibiaPin, LFTibiaPin };

//Min/Max values
const int16_t CoxaMin[] PROGMEM = { RRCoxaMin, RMCoxaMin, RFCoxaMin, LRCoxaMin, LMCoxaMin, LFCoxaMin };
const int16_t CoxaMax[] PROGMEM = { RRCoxaMax, RMCoxaMax, RFCoxaMax, LRCoxaMax, LMCoxaMax, LFCoxaMax };
const int16_t FemurMin[] PROGMEM = { RRFemurMin, RMFemurMin, RFFemurMin, LRFemurMin, LMFemurMin, LFFemurMin };
const int16_t FemurMax[] PROGMEM = { RRFemurMax, RMFemurMax, RFFemurMax, LRFemurMax, LMFemurMax, LFFemurMax };
const int16_t TibiaMin[] PROGMEM = { RRTibiaMin, RMTibiaMin, RFTibiaMin, LRTibiaMin, LMTibiaMin, LFTibiaMin };
const int16_t TibiaMax[] PROGMEM = { RRTibiaMax, RMTibiaMax, RFTibiaMax, LRTibiaMax, LMTibiaMax, LFTibiaMax };

//Body offsets (distance between the center of the body and the center of the coxa)
const int16_t OffsetX[] PROGMEM = { RROffsetX, RMOffsetX, RFOffsetX, LROffsetX, LMOffsetX, LFOffsetX };
const int16_t OffsetZ[] PROGMEM = { RROffsetZ, RMOffsetZ, RFOffsetZ, LROffsetZ, LMOffsetZ, LFOffsetZ };

//Default leg angle
const int16_t LegAngle[] PROGMEM = { RRLegAngle, RMLegAngle, RFLegAngle, LRLegAngle, LMLegAngle, LFLegAngle };

//Start positions for the leg
const int16_t InitPosX[] PROGMEM = { RRInitPosX, RMInitPosX, RFInitPosX, LRInitPosX, LMInitPosX, LFInitPosX };
const int16_t InitPosY[] PROGMEM = { RRInitPosY, RMInitPosY, RFInitPosY, LRInitPosY, LMInitPosY, LFInitPosY };
const int16_t InitPosZ[] PROGMEM = { RRInitPosZ, RMInitPosZ, RFInitPosZ, LRInitPosZ, LMInitPosZ, LFInitPosZ };

//Structs
typedef struct {
  float   Cos;
  float   Sin;
} trig;

typedef struct {
  uint8_t NrLiftedPos;    //Number of positions that a single leg is lifted (1-5)
  uint8_t FrontDownPos;   //Where the leg should be put down to ground
  uint8_t LiftDivFactor;  //Default: 2, when NrLiftedPos = 5: 4
  uint8_t HalfLiftHeight; //How high to lift at halfway up
  uint8_t TLDivFactor;    //Number of steps that a leg is on the floor while walking
  uint8_t StepsInGait;    //Number of steps in gait
  uint8_t NomGaitSpeed;   //Nominal speed of the gait
  uint8_t GaitLegNr[6];   //Init position of the leg (LR, RF, LM, RR, LF, RM)
} gait;

//Angles
float     CoxaAngle[6];
float     FemurAngle[6];
float     TibiaAngle[6];

//Body position
int16_t   BodyPosX;
int16_t   BodyPosY;
int16_t   BodyPosZ;
int16_t   BodyRotX;
int16_t   BodyRotY;
int16_t   BodyRotZ;
int16_t   BodyFKPosX;
int16_t   BodyFKPosY;
int16_t   BodyFKPosZ;

//Timing
uint8_t   InputTimeDelay;
uint16_t  PrevSpeedControl;
uint16_t  SpeedControl;
uint16_t  PrevSSCTime;
uint16_t  SSCTime;
uint32_t  TimeStart;

//Power
bool      PrevHexOn;
bool      HexOn;

//Balance
bool      BalanceMode;
int16_t   TotalBalX;
int16_t   TotalBalY;
int16_t   TotalBalZ;
int16_t   TotalTransX;
int16_t   TotalTransY;
int16_t   TotalTransZ;

//Single leg
bool      SLHold;
uint8_t   PrevSelectedLeg = NOT_SELECTED;
uint8_t   SelectedLeg = NOT_SELECTED;
int16_t   LegPosX[6];
int16_t   LegPosY[6];
int16_t   LegPosZ[6];
int16_t   SLLegX;
int16_t   SLLegY;
int16_t   SLLegZ;

//Gait
gait      GaitCurrent;
bool      GaitInMotion;
bool      Walking;
uint8_t   ExtraCycle;
uint8_t   GaitStep = 1;
uint8_t   GaitType;
uint8_t   LegLiftHeight = 50;
int16_t   GaitPosX[6];
int16_t   GaitPosY[6];
int16_t   GaitPosZ[6];
int16_t   GaitRotY[6];
int16_t   TravelLengthX;
int16_t   TravelLengthY;
int16_t   TravelLengthZ;

#ifdef DEBUG_MODE
bool      PrevWalking;
bool      DebugOutput;
#endif

#ifdef SOUND_MODE
MiniTone  Sound;
#endif

#endif
