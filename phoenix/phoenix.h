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

//Build tables for leg configuration like I/O and Min/Max values to easy access values using a for loop
//Constants are still defined as single values in the cfg file to make it easy to read/configure

//SSC pin numbers
const byte CoxaPin[] PROGMEM = { RRCoxaPin, RMCoxaPin, RFCoxaPin, LRCoxaPin, LMCoxaPin, LFCoxaPin };
const byte FemurPin[] PROGMEM = { RRFemurPin, RMFemurPin, RFFemurPin, LRFemurPin, LMFemurPin, LFFemurPin };
const byte TibiaPin[] PROGMEM = { RRTibiaPin, RMTibiaPin, RFTibiaPin, LRTibiaPin, LMTibiaPin, LFTibiaPin };

//Min/Max values
const short CoxaMin[] PROGMEM = { RRCoxaMin, RMCoxaMin, RFCoxaMin, LRCoxaMin, LMCoxaMin, LFCoxaMin };
const short CoxaMax[] PROGMEM = { RRCoxaMax, RMCoxaMax, RFCoxaMax, LRCoxaMax, LMCoxaMax, LFCoxaMax };
const short FemurMin[] PROGMEM = { RRFemurMin, RMFemurMin, RFFemurMin, LRFemurMin, LMFemurMin, LFFemurMin };
const short FemurMax[] PROGMEM = { RRFemurMax, RMFemurMax, RFFemurMax, LRFemurMax, LMFemurMax, LFFemurMax };
const short TibiaMin[] PROGMEM = { RRTibiaMin, RMTibiaMin, RFTibiaMin, LRTibiaMin, LMTibiaMin, LFTibiaMin };
const short TibiaMax[] PROGMEM = { RRTibiaMax, RMTibiaMax, RFTibiaMax, LRTibiaMax, LMTibiaMax, LFTibiaMax };

//Body offsets (distance between the center of the body and the center of the coxa)
const short OffsetX[] PROGMEM = { RROffsetX, RMOffsetX, RFOffsetX, LROffsetX, LMOffsetX, LFOffsetX };
const short OffsetZ[] PROGMEM = { RROffsetZ, RMOffsetZ, RFOffsetZ, LROffsetZ, LMOffsetZ, LFOffsetZ };

//Default leg angle
const short LegAngle[] PROGMEM = { RRLegAngle, RMLegAngle, RFLegAngle, LRLegAngle, LMLegAngle, LFLegAngle };

//Start positions for the leg
const short InitPosX[] PROGMEM = { RRInitPosX, RMInitPosX, RFInitPosX, LRInitPosX, LMInitPosX, LFInitPosX };
const short InitPosY[] PROGMEM = { RRInitPosY, RMInitPosY, RFInitPosY, LRInitPosY, LMInitPosY, LFInitPosY };
const short InitPosZ[] PROGMEM = { RRInitPosZ, RMInitPosZ, RFInitPosZ, LRInitPosZ, LMInitPosZ, LFInitPosZ };

//Angles
float    CoxaAngle[6];       //Actual angle of the horizontal hip
float    FemurAngle[6];      //Actual angle of the vertical hip
float    TibiaAngle[6];      //Actual angle of the knee

//Body position
short    BodyFKPosX;         //Output position X of feet with rotation
short    BodyFKPosY;         //Output position Y of feet with rotation
short    BodyFKPosZ;         //Output position Z of feet with rotation
short    BodyPosX;           //Global input for the position of the body
short    BodyPosY;
short    BodyPosZ;
short    BodyRotX;           //Global input pitch of the body
short    BodyRotY;           //Global input rotation of the body
short    BodyRotZ;           //Global input roll of the body

//Timing
byte     InputTimeDelay;     //Delay that depends on the input to get the "sneaking" effect
long     TimerStart;         //Start time of the calculation cycles
word     Prev_SSCTime;       //Previous time for the servo updates
word     Prev_SpeedControl;
word     SpeedControl;       //Adjustable delay
word     SSCTime;            //Time for servo updates

//Power
bool     HexOn;              //Switch to turn on Phoenix
bool     Prev_HexOn;         //Previous loop state 

//Balance
bool     BalanceMode;
short    TotalTransX;
short    TotalTransY;
short    TotalTransZ;
short    TotalBalX;
short    TotalBalY;
short    TotalBalZ;

//Single leg
bool     AllDown;
bool     SLHold;             //Single leg control mode
byte     Prev_SelectedLeg;
byte     SelectedLeg;
short    LegPosX[6];         //Actual X position of the leg
short    LegPosY[6];         //Actual Y position of the leg
short    LegPosZ[6];         //Actual Z position of the leg
short    SLLegX;
short    SLLegY;
short    SLLegZ;

//Gait
bool     TravelRequest;      //Temp to check if the gait is in motion
bool     Walking;            //True if the robot are walking
byte     ExtraCycle;         //Forcing some extra timed cycles for avoiding "end of gait bug"
byte     FrontDownPos;       //Where the leg should be put down to ground
byte     GaitLegNr[6];       //Init position of the leg
byte     GaitStep;           //Actual gait step
byte     GaitType;           //Gait type
byte     HalfLiftHeight;     //How high to lift at halfway up
byte     LegLiftHeight;      //Current travel height
byte     LiftDivFactor;
byte     NomGaitSpeed;       //Nominal speed of the gait
byte     NrLiftedPos;        //Number of positions that a single leg is lifted (1-3)
byte     StepsInGait;        //Number of steps in gait
byte     TLDivFactor;        //Number of steps that a leg is on the floor while walking
short    GaitPosX[6];        //Array containing relative X position corresponding to the gait
short    GaitPosY[6];        //Array containing relative Y position corresponding to the gait
short    GaitPosZ[6];        //Array containing relative Z position corresponding to the gait
short    GaitRotY[6];        //Array containing relative Y rotation corresponding to the gait
short    TravelLengthX;      //Current travel length X
short    TravelLengthY;      //Current travel rotation Y
short    TravelLengthZ;      //Current travel length Z

#ifdef DEBUG_MODE
bool     Prev_Walking;
bool     DebugOutputOn;
#endif

#ifdef SOUND_MODE
MiniTone Sound;
#endif

typedef struct {
  float Cos;
  float Sin;
} angle;

#endif
