/**
 * Project Lynxmotion Phoenix
 * Description: Phoenix software
 * Version: v2.0
 * Programmer: Jeroen Janssen (aka Xan)
 *             Kompanets Konstantin (aka I2M)
 *
 * Hardware setup: Arduino, SSC32 V2
 */

#ifndef PHOENIX_H
#define PHOENIX_H

//Changing the sign, depending on the legs
#define sign(leg) ((leg) <= (2) ? (-1) : (1))

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
  uint8_t NrLiftedPos;        //Number of positions that a single leg is lifted (1-5)
  uint8_t FrontDownPos;       //Where the leg should be put down to ground
  uint8_t LiftDivFactor;      //Default: 2, when NrLiftedPos = 5: 4
  uint8_t HalfLiftHeight;     //How high to lift at halfway up
  uint8_t TLDivFactor;        //Number of steps that a leg is on the floor while walking
  uint8_t StepsInGait;        //Number of steps in gait
  uint8_t NomGaitSpeed;       //Nominal speed of the gait
  uint8_t GaitLegNr[6];       //Init position of the leg (LR, RF, LM, RR, LF, RM)
} gait;

//Angles
float     CoxaAngle[6];       //Actual angle of the horizontal hip
float     FemurAngle[6];      //Actual angle of the vertical hip
float     TibiaAngle[6];      //Actual angle of the knee

//Body position
int16_t   BodyFKPosX;         //Output position X of feet with rotation
int16_t   BodyFKPosY;         //Output position Y of feet with rotation
int16_t   BodyFKPosZ;         //Output position Z of feet with rotation
int16_t   BodyPosX;           //Global input for the position of the body
int16_t   BodyPosY;
int16_t   BodyPosZ;
int16_t   BodyRotX;           //Global input pitch of the body
int16_t   BodyRotY;           //Global input rotation of the body
int16_t   BodyRotZ;           //Global input roll of the body

//Timing
uint8_t   InputTimeDelay;     //Delay that depends on the input to get the "sneaking" effect
uint16_t  PrevSpeedControl;
uint16_t  PrevSSCTime;        //Previous time for the servo updates
uint16_t  SpeedControl;       //Adjustable delay
uint16_t  SSCTime;            //Time for servo updates
uint32_t  TimerStart;         //Start time of the calculation cycles

//Power
bool      HexOn;              //Switch to turn on Phoenix
bool      PrevHexOn;          //Previous loop state 

//Balance
bool      BalanceMode;
int16_t   TotalBalX;
int16_t   TotalBalY;
int16_t   TotalBalZ;
int16_t   TotalTransX;
int16_t   TotalTransY;
int16_t   TotalTransZ;

//Single leg
bool      AllDown;
bool      SLHold;             //Single leg control mode
uint8_t   PrevSelectedLeg;
uint8_t   SelectedLeg;
int16_t   LegPosX[6];         //Actual X position of the leg
int16_t   LegPosY[6];         //Actual Y position of the leg
int16_t   LegPosZ[6];         //Actual Z position of the leg
int16_t   SLLegX;
int16_t   SLLegY;
int16_t   SLLegZ;

//Gait
bool      GaitInMotion;       //Temp to check if the gait is in motion
bool      Walking;            //True if the robot are walking
gait      GaitCurrent;
uint8_t   ExtraCycle;         //Forcing some extra timed cycles for avoiding "end of gait bug"
uint8_t   GaitStep;           //Actual gait step
uint8_t   GaitType;           //Gait type
uint8_t   LegLiftHeight;      //Current travel height
int16_t   GaitPosX[6];        //Array containing relative X position corresponding to the gait
int16_t   GaitPosY[6];        //Array containing relative Y position corresponding to the gait
int16_t   GaitPosZ[6];        //Array containing relative Z position corresponding to the gait
int16_t   GaitRotY[6];        //Array containing relative Y rotation corresponding to the gait
int16_t   TravelLengthX;      //Current travel length X
int16_t   TravelLengthY;      //Current travel rotation Y
int16_t   TravelLengthZ;      //Current travel length Z

#ifdef DEBUG_MODE
bool      PrevWalking;
bool      DebugOutput;
#endif

#ifdef SOUND_MODE
MiniTone  Sound;
#endif

#endif
