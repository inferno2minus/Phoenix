/**
 * Project Lynxmotion Phoenix
 * Description: Phoenix software
 * Version: v2.0
 * Programmer: Jeroen Janssen (aka Xan)
 * Porting: Kompanets Konstantin (aka I2M)
 *
 * Hardware setup: Arduino, SSC32 V2
 */

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "phoenix_cfg.h"

//[CONSTANTS]
#define c1DEC              10
#define c2DEC              100
#define c4DEC              10000
#define c6DEC              1000000

#define cRR                0
#define cRM                1
#define cRF                2
#define cLR                3
#define cLM                4
#define cLF                5

#define cPWMDiv            991
#define cPFCons            592

#define cTravelDeadZone    4 //The deadzone for the analog input from the remote

//[TABLES]
//ArcCosinus Table
//Table build in to 3 part to get higher accuracy near cos = 1.
//The biggest error is near cos = 1 and has a biggest value of 3*0.012098rad = 0.521 deg.
//- Cos 0 to 0.9 is done by steps of 0.0079 rad. [1/127]
//- Cos 0.9 to 0.99 is done by steps of 0.0008 rad [0.1/127]
//- Cos 0.99 to 1 is done by step of 0.0002 rad [0.01/64]
//Since the tables are overlapping the full range of 127+127+64 is not necessary. Total bytes: 277
static const byte cACosTable[] PROGMEM = {
  255,254,252,251,250,249,247,246,245,243,242,241,240,238,237,236,234,233,232,231,229,
  228,227,225,224,223,221,220,219,217,216,215,214,212,211,210,208,207,206,204,203,201,
  200,199,197,196,195,193,192,190,189,188,186,185,183,182,181,179,178,176,175,173,172,
  170,169,167,166,164,163,161,160,158,157,155,154,152,150,149,147,146,144,142,141,139,
  137,135,134,132,130,128,127,125,123,121,119,117,115,113,111,109,107,105,103,101,98,
  96,94,92,89,87,84,81,79,76,73,73,73,72,72,72,71,71,71,70,70,70,70,69,69,69,68,68,68,
  67,67,67,66,66,66,65,65,65,64,64,64,63,63,63,62,62,62,61,61,61,60,60,59,59,59,58,58,
  58,57,57,57,56,56,55,55,55,54,54,53,53,53,52,52,51,51,51,50,50,49,49,48,48,47,47,47,
  46,46,45,45,44,44,43,43,42,42,41,41,40,40,39,39,38,37,37,36,36,35,34,34,33,33,32,31,
  31,30,29,28,28,27,26,25,24,23,23,23,23,22,22,22,22,21,21,21,21,20,20,20,19,19,19,19,
  18,18,18,17,17,17,17,16,16,16,15,15,15,14,14,13,13,13,12,12,11,11,10,10,9,9,8,7,6,6,
  5,3,0 };

//Sin table 90 deg, precision 0.5 deg [180 values]
static const word cSinTable[] PROGMEM = {
  0,87,174,261,348,436,523,610,697,784,871,958,1045,1132,1218,1305,1391,1478,1564,1650,
  1736,1822,1908,1993,2079,2164,2249,2334,2419,2503,2588,2672,2756,2840,2923,3007,3090,
  3173,3255,3338,3420,3502,3583,3665,3746,3826,3907,3987,4067,4146,4226,4305,4383,4461,
  4539,4617,4694,4771,4848,4924,4999,5075,5150,5224,5299,5372,5446,5519,5591,5664,5735,
  5807,5877,5948,6018,6087,6156,6225,6293,6360,6427,6494,6560,6626,6691,6755,6819,6883,
  6946,7009,7071,7132,7193,7253,7313,7372,7431,7489,7547,7604,7660,7716,7771,7826,7880,
  7933,7986,8038,8090,8141,8191,8241,8290,8338,8386,8433,8480,8526,8571,8616,8660,8703,
  8746,8788,8829,8870,8910,8949,8987,9025,9063,9099,9135,9170,9205,9238,9271,9304,9335,
  9366,9396,9426,9455,9483,9510,9537,9563,9588,9612,9636,9659,9681,9702,9723,9743,9762,
  9781,9799,9816,9832,9848,9862,9876,9890,9902,9914,9925,9935,9945,9953,9961,9969,9975,
  9981,9986,9990,9993,9996,9998,9999,10000 };

//Build tables for Leg configuration like I/O and MIN/MAX values to easy access values using a FOR loop
//Constants are still defined as single values in the cfg file to make it easy to read/configure

//SSC Pin numbers
const byte cCoxaPin[] PROGMEM = { cRRCoxaPin, cRMCoxaPin, cRFCoxaPin, cLRCoxaPin, cLMCoxaPin, cLFCoxaPin };
const byte cFemurPin[] PROGMEM = { cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin };
const byte cTibiaPin[] PROGMEM = { cRRTibiaPin, cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin };

//Min / Max values
const short cCoxaMin[] PROGMEM = { cRRCoxaMin, cRMCoxaMin, cRFCoxaMin, cLRCoxaMin, cLMCoxaMin, cLFCoxaMin };
const short cCoxaMax[] PROGMEM = { cRRCoxaMax, cRMCoxaMax, cRFCoxaMax, cLRCoxaMax, cLMCoxaMax, cLFCoxaMax };
const short cFemurMin[] PROGMEM = { cRRFemurMin, cRMFemurMin, cRFFemurMin, cLRFemurMin, cLMFemurMin, cLFFemurMin };
const short cFemurMax[] PROGMEM = { cRRFemurMax, cRMFemurMax, cRFFemurMax, cLRFemurMax, cLMFemurMax, cLFFemurMax };
const short cTibiaMin[] PROGMEM = { cRRTibiaMin, cRMTibiaMin, cRFTibiaMin, cLRTibiaMin, cLMTibiaMin, cLFTibiaMin };
const short cTibiaMax[] PROGMEM = { cRRTibiaMax, cRMTibiaMax, cRFTibiaMax, cLRTibiaMax, cLMTibiaMax, cLFTibiaMax };

//Body Offsets [distance between the center of the body and the center of the coxa]
const short cOffsetX[] PROGMEM = { cRROffsetX, cRMOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLFOffsetX };
const short cOffsetZ[] PROGMEM = { cRROffsetZ, cRMOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLFOffsetZ };

//Default leg angle
const short cCoxaAngle[] PROGMEM = { cRRCoxaAngle, cRMCoxaAngle, cRFCoxaAngle, cLRCoxaAngle, cLMCoxaAngle, cLFCoxaAngle };

//Start positions for the leg
const short cInitPosX[] PROGMEM = { cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX };
const short cInitPosY[] PROGMEM = { cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY };
const short cInitPosZ[] PROGMEM = { cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ };

//[ANGLES]
short    CoxaAngle[6];       //Actual Angle of the horizontal hip, decimals = 1
short    FemurAngle[6];      //Actual Angle of the vertical hip, decimals = 1
short    TibiaAngle[6];      //Actual Angle of the knee, decimals = 1

//[VARIABLES]
bool     NegativeValue;      //If the value is Negative
byte     LegIndex;           //Index used for leg Index Number
long     AngleRad;           //Output Angle in radials, decimals = 4
long     IKA1;               //Angle of the line S>W with respect to the ground in radians, decimals = 4
long     IKA2;               //Angle of the line S>W with respect to the femur in radians, decimals = 4
long     IKSW;               //Length between Shoulder and Wrist, decimals = 2
short    BodyFKPosX;         //Output Position X of feet with Rotation
short    BodyFKPosY;         //Output Position Y of feet with Rotation
short    BodyFKPosZ;         //Output Position Z of feet with Rotation
short    BodyPosX;           //Global Input for the position of the body
short    BodyPosY;
short    BodyPosZ;
short    BodyRotX;           //Global Input pitch of the body
short    BodyRotY;           //Global Input rotation of the body
short    BodyRotZ;           //Global Input roll of the body
short    Cos;                //Output Cosinus of the given Angle, decimals = 4
short    CosA;               //Cos buffer for BodyRotX calculations
short    CosB;               //Cos buffer for BodyRotX calculations
short    CosG;               //Cos buffer for BodyRotZ calculations
short    PosXZ;              //Diagonal direction from Input X and Z
short    Sin;                //Output Sinus of the given Angle, decimals = 4
short    SinA;               //Sin buffer for BodyRotX calculations
short    SinB;               //Sin buffer for BodyRotX calculations
short    SinG;               //Sin buffer for BodyRotZ calculations
short    TotalX;             //Total X distance between the center of the body and the feet
short    TotalY;             //Total Y distance between the center of the body and the feet
short    TotalZ;             //Total Z distance between the center of the body and the feet
word     ABSAngleDeg;        //Absolute value of the Angle in Degrees, decimals = 1

//[TIMING]
byte     CycleTime;          //Total Cycle time
byte     InputTimeDelay;     //Delay that depends on the input to get the "sneaking" effect
long     lTimerEnd;          //End time of the calculation cycles
long     lTimerStart;        //Start time of the calculation cycles
word     Prev_SSCTime;       //Previous time for the servo updates
word     SpeedControl;       //Adjustable Delay
word     SSCTime;            //Time for servo updates

//[POWER]
bool     HexOn;              //Switch to turn on Phoenix
bool     Prev_HexOn;         //Previous loop state 

//[SERVO DRIVER]
byte     Array[3];
word     CoxaPWM;
word     FemurPWM;
word     TibiaPWM;

//[BALANCE]
bool     BalanceMode;
short    TotalTransX;
short    TotalTransZ;
short    TotalTransY;
short    TotalBalY;
short    TotalBalX;
short    TotalBalZ;

//[SINGLE LEG CONTROL]
bool     AllDown;
bool     SLHold;             //Single leg control mode
byte     Prev_SelectedLeg;
byte     SelectedLeg;
short    LegPosX[6];         //Actual X Position of the Leg
short    LegPosY[6];         //Actual Y Position of the Leg
short    LegPosZ[6];         //Actual Z Position of the Leg
short    SLLegX;
short    SLLegY;
short    SLLegZ;

//[GAIT]
bool     LastLeg;            //TRUE when the current leg is the last leg of the sequence
bool     TravelRequest;      //Temp to check if the gait is in motion
bool     Walking;            //True if the robot are walking
byte     ExtraCycle;         //Forcing some extra timed cycles for avoiding "end of gait bug"
byte     GaitLegNr[6];       //Init position of the leg
byte     GaitStep;           //Actual Gait step
byte     GaitType;           //Gait type
byte     HalfLiftHeight;     //How high to lift at halfway up
byte     LegLiftHeight;      //Current Travel height
byte     LiftDivFactor;      //Normally: 2, when NrLiftedPos=5: 4
byte     NomGaitSpeed;       //Nominal speed of the gait
byte     NrLiftedPos;        //Number of positions that a single leg is lifted (1-3)
byte     StepsInGait;        //Number of steps in gait
byte     TLDivFactor;        //Number of steps that a leg is on the floor while walking
short    GaitPosX[6];        //Array containing Relative X position corresponding to the Gait
short    GaitPosY[6];        //Array containing Relative Y position corresponding to the Gait
short    GaitPosZ[6];        //Array containing Relative Z position corresponding to the Gait
short    GaitRotY[6];        //Array containing Relative Y rotation corresponding to the Gait  
short    TravelLengthX;      //Current Travel length X
short    TravelLengthZ;      //Current Travel length Z
short    TravelLengthY;      //Current Travel Rotation Y

#ifdef DEBUG_MODE
bool     Prev_Walking;
bool     DebugOutputOn;
#endif

#ifdef SOUND_MODE
extern void MSound(byte cNotes, ...);
#endif

//Init
void setup() {
  SSCSerial.begin(cSSC_BAUD);

#ifdef DEBUG_MODE
  DBGSerial.begin(cDBG_BAUD);
  DBGSerial.println("Start Debugging");
#endif

  //Setup Init Positions
  for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
    LegPosX[LegIndex] = (short)pgm_read_word(&cInitPosX[LegIndex]); //Set start positions for each leg
    LegPosY[LegIndex] = (short)pgm_read_word(&cInitPosY[LegIndex]);
    LegPosZ[LegIndex] = (short)pgm_read_word(&cInitPosZ[LegIndex]);
  }

  //Single leg control. Make sure no leg is selected
  SelectedLeg = 255; //No Leg selected
  Prev_SelectedLeg = 255;

  //Body Positions
  BodyPosX = 0;
  BodyPosY = 0;
  BodyPosZ = 0;

  //Body Rotations
  BodyRotX = 0;
  BodyRotY = 0;
  BodyRotZ = 0;

  //Gait
  GaitType = 0;
  BalanceMode = 0;
  LegLiftHeight = 50;
  GaitStep = 1;
  GaitSelect();

  //Initialize Controller
  InitController();

  //SSC
  SSCTime = 150;
  HexOn = 0;
}

//Main
void loop() {
  //Start time
  lTimerStart = millis();

  //Read Input
  ControlInput();

  //Single leg control
  SingleLegControl();

  //Gait
  GaitSeq();

  //Balance calculations
  TotalTransX = 0; //Reset values used for calculation of balance
  TotalTransY = 0;
  TotalTransZ = 0;
  TotalBalX = 0;
  TotalBalY = 0;
  TotalBalZ = 0;

  if (BalanceMode) {
    for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
      if (LegIndex <= 2) {
        //Balance calculations for all Right legs
        BalCalcOneLeg(-LegPosX[LegIndex] + GaitPosX[LegIndex],
        (LegPosY[LegIndex] - (short)pgm_read_word(&cInitPosY[LegIndex])) + GaitPosY[LegIndex],
        LegPosZ[LegIndex] + GaitPosZ[LegIndex], LegIndex);
      }
      else {
        //Balance calculations for all Left legs
        BalCalcOneLeg(LegPosX[LegIndex] + GaitPosX[LegIndex],
        (LegPosY[LegIndex] - (short)pgm_read_word(&cInitPosY[LegIndex])) + GaitPosY[LegIndex],
        LegPosZ[LegIndex] + GaitPosZ[LegIndex], LegIndex);
      }
    }
    BalanceBody();
  }

  for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
    if (LegIndex <= 2) {
      //Do IK for all Right legs
      BodyFK(-LegPosX[LegIndex] + BodyPosX + GaitPosX[LegIndex] - TotalTransX,
      LegPosY[LegIndex] + BodyPosY + GaitPosY[LegIndex] - TotalTransY,
      LegPosZ[LegIndex] + BodyPosZ + GaitPosZ[LegIndex] - TotalTransZ,
      GaitRotY[LegIndex], LegIndex);

      LegIK(LegPosX[LegIndex] - BodyPosX + BodyFKPosX - (GaitPosX[LegIndex] - TotalTransX), 
      LegPosY[LegIndex] + BodyPosY - BodyFKPosY + GaitPosY[LegIndex] - TotalTransY,
      LegPosZ[LegIndex] + BodyPosZ - BodyFKPosZ + GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
    }
    else {
      //Do IK for all Left legs 
      BodyFK(LegPosX[LegIndex] - BodyPosX + GaitPosX[LegIndex] - TotalTransX,
      LegPosY[LegIndex] + BodyPosY + GaitPosY[LegIndex] - TotalTransY,
      LegPosZ[LegIndex] + BodyPosZ + GaitPosZ[LegIndex] - TotalTransZ,
      GaitRotY[LegIndex], LegIndex);

      LegIK(LegPosX[LegIndex] + BodyPosX - BodyFKPosX + GaitPosX[LegIndex] - TotalTransX,
      LegPosY[LegIndex] + BodyPosY - BodyFKPosY + GaitPosY[LegIndex] - TotalTransY,
      LegPosZ[LegIndex] + BodyPosZ - BodyFKPosZ + GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
    }
  }

  //Check mechanical limits
  CheckAngles();

  //Drive Servos
  if (HexOn) {
    if (HexOn && !Prev_HexOn) {
#ifdef SOUND_MODE
      MSound(3, 60, 1661, 80, 2217, 100, 2794);
#endif
    }

    //Set SSC time
    if ((abs(TravelLengthX) > cTravelDeadZone) || (abs(TravelLengthZ) > cTravelDeadZone) || (abs(TravelLengthY * 2) > cTravelDeadZone)) {

      SSCTime = NomGaitSpeed + (InputTimeDelay * 2) + SpeedControl;

      //Add additional delay when Balance mode is on
      if (BalanceMode) {
        SSCTime += 100;
      }
    }
    else { //Movement speed excl. Walking
      SSCTime = 200 + SpeedControl;
    }

    //Update servo positions without committing
    ServoDriverUpdate();

    //Finding any the biggest value for GaitPos/Rot
    for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
      if ((GaitPosX[LegIndex] > 2) || (GaitPosX[LegIndex] < -2) ||
        (GaitPosZ[LegIndex] > 2) || (GaitPosZ[LegIndex] < -2) ||
        (GaitRotY[LegIndex] > 2) || (GaitRotY[LegIndex] < -2)) {
        ExtraCycle = NrLiftedPos + 1; //For making sure that we are using timed move until all legs are down
        break;
      }
    }

    if (ExtraCycle > 0) {
      ExtraCycle--;
      Walking = !(ExtraCycle == 0);

      //Get endtime and calculate wait time
      lTimerEnd = millis();
      CycleTime = (lTimerEnd - lTimerStart);

#ifdef DEBUG_MODE
      if (Walking && !Prev_Walking) {
        DBGSerial.println("Walking: Start");
        Prev_Walking = true;
      }
      else if (!Walking) {
        DBGSerial.println("Walking: Finish");
        Prev_Walking = false;
      }
#endif

      //Wait for previous commands to be completed while walking
      delay(max(Prev_SSCTime - CycleTime, 1)); //Min 1 ensures that there always is a value in the pause command
    }

    //Commit servo positions
    ServoDriverCommit();
  }
  else if (Prev_HexOn || !AllDown) { //Turn the bot off
    SSCTime = 600;
    ServoDriverUpdate();
    ServoDriverCommit();
#ifdef SOUND_MODE
    MSound(3, 100, 2794, 80, 2217, 60, 1661);
#endif
    delay(600);
  }
  else {
    FreeServos();
    delay(20);
  }

  Prev_SSCTime = SSCTime;

  //Store previous HexOn State
  if (HexOn) {
    Prev_HexOn = 1;
  }
  else {
    Prev_HexOn = 0;
  }
}

//Single leg control
void SingleLegControl() {
  //Check if all legs are down
  AllDown = (LegPosY[cRF] == (short)pgm_read_word(&cInitPosY[cRF])) &&
    (LegPosY[cRM] == (short)pgm_read_word(&cInitPosY[cRM])) && 
    (LegPosY[cRR] == (short)pgm_read_word(&cInitPosY[cRR])) && 
    (LegPosY[cLR] == (short)pgm_read_word(&cInitPosY[cLR])) && 
    (LegPosY[cLM] == (short)pgm_read_word(&cInitPosY[cLM])) && 
    (LegPosY[cLF] == (short)pgm_read_word(&cInitPosY[cLF]));

  if (SelectedLeg <= 5) {
    if (SelectedLeg != Prev_SelectedLeg) {
      if (AllDown) { //Lift leg a bit when it got selected
        LegPosY[SelectedLeg] = (short)pgm_read_word(&cInitPosY[SelectedLeg]) - 20;

        //Store current status
        Prev_SelectedLeg = SelectedLeg;
      }
      else { //Return prev leg back to the init position
        LegPosX[Prev_SelectedLeg] = (short)pgm_read_word(&cInitPosX[Prev_SelectedLeg]);
        LegPosY[Prev_SelectedLeg] = (short)pgm_read_word(&cInitPosY[Prev_SelectedLeg]);
        LegPosZ[Prev_SelectedLeg] = (short)pgm_read_word(&cInitPosZ[Prev_SelectedLeg]);
      }
    }
    else if (!SLHold) {
      //TODO: LegPosY[SelectedLeg] = LegPosY[SelectedLeg] + SLLegY;
      LegPosY[SelectedLeg] = (short)pgm_read_word(&cInitPosY[SelectedLeg]) + SLLegY; //Using DIY remote Zenta prefer it this way
      LegPosX[SelectedLeg] = (short)pgm_read_word(&cInitPosX[SelectedLeg]) + SLLegX;
      LegPosZ[SelectedLeg] = (short)pgm_read_word(&cInitPosZ[SelectedLeg]) + SLLegZ;
    }
  }
  else { //All legs to init position
    if (!AllDown) {
      for(LegIndex = 0; LegIndex <= 5; LegIndex++) {
        LegPosX[LegIndex] = (short)pgm_read_word(&cInitPosX[LegIndex]);
        LegPosY[LegIndex] = (short)pgm_read_word(&cInitPosY[LegIndex]);
        LegPosZ[LegIndex] = (short)pgm_read_word(&cInitPosZ[LegIndex]);
      }
    }
    if (Prev_SelectedLeg != 255) {
      Prev_SelectedLeg = 255;
    }
  }
}

//Gait select
void GaitSelect() {
  //Gait selector
  switch (GaitType) {
  case 0: //Ripple 12 steps
    GaitLegNr[cLR] = 1;
    GaitLegNr[cRF] = 3;
    GaitLegNr[cLM] = 5;
    GaitLegNr[cRR] = 7;
    GaitLegNr[cLF] = 9;
    GaitLegNr[cRM] = 11;

    NrLiftedPos = 3;
    HalfLiftHeight = 3;
    TLDivFactor = 8;
    StepsInGait = 12;
    NomGaitSpeed = 70;
    break;
  case 1: //Tripod 6 steps
    GaitLegNr[cLR] = 4;
    GaitLegNr[cRF] = 1;
    GaitLegNr[cLM] = 1;
    GaitLegNr[cRR] = 1;
    GaitLegNr[cLF] = 4;
    GaitLegNr[cRM] = 4;

    NrLiftedPos = 2;
    HalfLiftHeight = 1;
    TLDivFactor = 4;
    StepsInGait = 6;
    NomGaitSpeed = 60;
    break;
  case 2: //Tripod 8 steps
    GaitLegNr[cLR] = 5;
    GaitLegNr[cRF] = 1;
    GaitLegNr[cLM] = 1;
    GaitLegNr[cRR] = 1;
    GaitLegNr[cLF] = 5;
    GaitLegNr[cRM] = 5;

    NrLiftedPos = 3;
    HalfLiftHeight = 3;
    TLDivFactor = 4;
    StepsInGait = 8;
    NomGaitSpeed = 70;
    break;
  case 3: //Tripod 12 steps
    GaitLegNr[cLR] = 11;
    GaitLegNr[cRF] = 3;
    GaitLegNr[cLM] = 4;
    GaitLegNr[cRR] = 5;
    GaitLegNr[cLF] = 9;
    GaitLegNr[cRM] = 10;

    NrLiftedPos = 3;
    HalfLiftHeight = 3;
    TLDivFactor = 8;
    StepsInGait = 12;
    NomGaitSpeed = 60;
    break;
  case 4: //Tripod 16 steps, use 5 lifted positions!
    GaitLegNr[cLR] = 14;
    GaitLegNr[cRF] = 4;
    GaitLegNr[cLM] = 5;
    GaitLegNr[cRR] = 6;
    GaitLegNr[cLF] = 12;
    GaitLegNr[cRM] = 13;

    NrLiftedPos = 5;
    HalfLiftHeight = 1;
    TLDivFactor = 10;
    StepsInGait = 16;
    NomGaitSpeed = 60;
    break;
  case 5: //Wave 24 steps
    GaitLegNr[cLR] = 1;
    GaitLegNr[cRF] = 21;
    GaitLegNr[cLM] = 5;
    GaitLegNr[cRR] = 13;
    GaitLegNr[cLF] = 9;
    GaitLegNr[cRM] = 17;

    NrLiftedPos = 3;
    HalfLiftHeight = 3;
    TLDivFactor = 20;
    StepsInGait = 24;
    NomGaitSpeed = 70;
    break;
  }
}

//Gait sequence
void GaitSeq() {
  //Check IF the Gait is in motion
  TravelRequest = (abs(TravelLengthX) > cTravelDeadZone) || (abs(TravelLengthZ) > cTravelDeadZone) || (abs(TravelLengthY) > cTravelDeadZone) || Walking;

  if (NrLiftedPos == 5) {
    LiftDivFactor = 4;
  }
  else {
    LiftDivFactor = 2;
  }

  //Calculate Gait sequence
  LastLeg = 0;
  for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
    if (LegIndex == 5) {
      LastLeg = 1;
    }
    Gait(LegIndex);
  }
}

//Gait
void Gait(byte LegIndex) {
  //Clear values under the cTravelDeadZone
  if (!TravelRequest) {
    TravelLengthX = 0;
    TravelLengthZ = 0;
    TravelLengthY = 0;
  }

  //Leg middle up position
  if ((TravelRequest && (NrLiftedPos == 1 || NrLiftedPos == 3 || NrLiftedPos == 5) &&
    GaitStep == GaitLegNr[LegIndex]) || (!TravelRequest && GaitStep == GaitLegNr[LegIndex] &&
    ((abs(GaitPosX[LegIndex]) > 2) || (abs(GaitPosZ[LegIndex]) > 2) || (abs(GaitRotY[LegIndex]) > 2)))) {
    GaitPosX[LegIndex] = 0;
    GaitPosY[LegIndex] = -LegLiftHeight;
    GaitPosZ[LegIndex] = 0;
    GaitRotY[LegIndex] = 0;
  }

  //Optional Half height Rear (2, 3, 5 lifted positions)
  else if (((NrLiftedPos == 2 && GaitStep==GaitLegNr[LegIndex]) || (NrLiftedPos >= 3 &&
    (GaitStep == GaitLegNr[LegIndex] - 1 || GaitStep == GaitLegNr[LegIndex] + (StepsInGait - 1)))) && TravelRequest) {
    GaitPosX[LegIndex] = -TravelLengthX / LiftDivFactor;
    GaitPosY[LegIndex] = -3 * LegLiftHeight / (3 + HalfLiftHeight); //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[LegIndex] = -TravelLengthZ / LiftDivFactor;
    GaitRotY[LegIndex] = -TravelLengthY / LiftDivFactor;
  }

  //Optional Half height Front (2, 3, 5 lifted positions)
  else if ((NrLiftedPos >= 2) && (GaitStep == GaitLegNr[LegIndex] + 1 ||
    GaitStep == GaitLegNr[LegIndex] - (StepsInGait - 1)) && TravelRequest) {
    GaitPosX[LegIndex] = TravelLengthX / LiftDivFactor;
    GaitPosY[LegIndex] = -3 * LegLiftHeight / (3 + HalfLiftHeight); //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[LegIndex] = TravelLengthZ / LiftDivFactor;
    GaitRotY[LegIndex] = TravelLengthY / LiftDivFactor;
  }

  //Optional Half height Rear 5 LiftedPos (5 lifted positions)
  else if (((NrLiftedPos == 5 && (GaitStep == GaitLegNr[LegIndex] - 2))) && TravelRequest) {
    GaitPosX[LegIndex] = -TravelLengthX / 2;
    GaitPosY[LegIndex] = -LegLiftHeight / 2;
    GaitPosZ[LegIndex] = -TravelLengthZ / 2;
    GaitRotY[LegIndex] = -TravelLengthY / 2;
  }

  //Optional Half height Front 5 LiftedPos (5 lifted positions)
  else if ((NrLiftedPos == 5) && (GaitStep == GaitLegNr[LegIndex] + 2 ||
    GaitStep == GaitLegNr[LegIndex] - (StepsInGait - 2)) && TravelRequest) {
    GaitPosX[LegIndex] = TravelLengthX / 2;
    GaitPosY[LegIndex] = -LegLiftHeight / 2;
    GaitPosZ[LegIndex] = TravelLengthZ / 2;
    GaitRotY[LegIndex] = TravelLengthY / 2;
  }

  //Leg front down position
  else if ((GaitStep == GaitLegNr[LegIndex] + NrLiftedPos ||
    GaitStep == GaitLegNr[LegIndex] - (StepsInGait - NrLiftedPos)) && GaitPosY[LegIndex] < 0) {
    GaitPosX[LegIndex] = TravelLengthX / 2;
    GaitPosY[LegIndex] = 0;
    GaitPosZ[LegIndex] = TravelLengthZ / 2;
    GaitRotY[LegIndex] = TravelLengthY / 2;
  }

  //Move body forward
  else {
    GaitPosX[LegIndex] = GaitPosX[LegIndex] - (TravelLengthX / TLDivFactor);
    GaitPosY[LegIndex] = 0;
    GaitPosZ[LegIndex] = GaitPosZ[LegIndex] - (TravelLengthZ / TLDivFactor);
    GaitRotY[LegIndex] = GaitRotY[LegIndex] - (TravelLengthY / TLDivFactor);
  }

  //Advance to the next step
  if (LastLeg) { //The last leg in this step
    GaitStep++;
    if (GaitStep > StepsInGait) {
      GaitStep = 1;
    }
  }
}

//Balance calculation one leg
void BalCalcOneLeg (short PosX, short PosY, short PosZ, byte LegIndex) {
  //Calculating totals from center of the body to the feet
  TotalX = (short)pgm_read_word(&cOffsetX[LegIndex]) + PosX;
  TotalY = 150 + PosY; //Using the value 150 to lower the center point of rotation BodyPosY
  TotalZ = (short)pgm_read_word(&cOffsetZ[LegIndex]) + PosZ;

  TotalTransX += TotalX;
  TotalTransY += PosY;
  TotalTransZ += TotalZ;

  TotalBalX += (GetATan2(TotalZ, TotalY) * 1800) / 31415 - 900; //Rotate balance circle 90 deg
  TotalBalY += (GetATan2(TotalX, TotalZ) * 1800) / 31415;
  TotalBalZ += (GetATan2(TotalX, TotalY) * 1800) / 31415 - 900; //Rotate balance circle 90 deg
}

//Balance body
void BalanceBody() {
  TotalTransX = TotalTransX / 6;
  TotalTransY = TotalTransY / 6;
  TotalTransZ = TotalTransZ / 6;

  if (TotalBalY > 0) { //Rotate balance circle by +/- 180 deg
    TotalBalY -= 1800;
  }
  else {
    TotalBalY += 1800;
  }

  if (TotalBalZ < -1800) { //Compensate for extreme balance positions that causes overflow
    TotalBalZ += 3600;
  }

  if (TotalBalX < -1800) { //Compensate for extreme balance positions that causes overflow
    TotalBalX += 3600;
  }

  //Balance rotation
  TotalBalX = -TotalBalX / 6;
  TotalBalY = -TotalBalY / 6;
  TotalBalZ = TotalBalZ / 6;
}

//Get the sinus and cosinus from the angle +/- multiple circles
void GetSinCos(short AngleDeg) {
  //Get the absolute value of AngleDeg
  ABSAngleDeg = abs(AngleDeg);

  //Shift rotation to a full circle of 360 deg -> AngleDeg // 360
  if (AngleDeg < 0) { //Negative values
    AngleDeg = 3600 - (ABSAngleDeg - (3600 * (ABSAngleDeg / 3600)));
  }
  else { //Positive values
    AngleDeg = ABSAngleDeg - (3600 * (ABSAngleDeg / 3600));
  }

  if (AngleDeg >= 0 && AngleDeg <= 900) { //0 to 90 deg
    Sin = pgm_read_word(&cSinTable[AngleDeg / 5]); //5 is the precision (0.5) of the table
    Cos = pgm_read_word(&cSinTable[(900 - AngleDeg) / 5]);
  }
  else if (AngleDeg > 900 && AngleDeg <= 1800) { //90 to 180 deg
    Sin = pgm_read_word(&cSinTable[(900 - (AngleDeg - 900)) / 5]); //5 is the precision (0.5) of the table 
    Cos = -pgm_read_word(&cSinTable[(AngleDeg - 900) / 5]);
  }
  else if (AngleDeg > 1800 && AngleDeg <= 2700) { //180 to 270 deg
    Sin = -pgm_read_word(&cSinTable[(AngleDeg - 1800) / 5]); //5 is the precision (0.5) of the table
    Cos = -pgm_read_word(&cSinTable[(2700 - AngleDeg) / 5]);
  }
  else if(AngleDeg > 2700 && AngleDeg <= 3600) { //270 to 360 deg
    Sin = -pgm_read_word(&cSinTable[(3600 - AngleDeg) / 5]); //5 is the precision (0.5) of the table 
    Cos = pgm_read_word(&cSinTable[(AngleDeg - 2700) / 5]);
  }
}

//Get the sinus and cosinus from the angle +/- multiple circles
long GetACos(short Cos) {
  //Check for negative value
  if (Cos < 0) {
    Cos = -Cos;
    NegativeValue = 1;
  }
  else {
    NegativeValue = 0;
  }

  //Limit Cos to his maximal value
  Cos = min(Cos, c4DEC);

  if (Cos >= 0 && Cos < 9000) {
    AngleRad = (byte)pgm_read_byte(&cACosTable[Cos / 79]); //79=table resolution (1/127)
    AngleRad = (AngleRad * 616) / c1DEC; //616=acos resolution (pi/2/255)
  }
  else if (Cos >= 9000 && Cos < 9900) {
    AngleRad = (byte)pgm_read_byte(&cACosTable[(Cos - 9000) / 8 + 114]); //8=table resolution (0.1/127), 114 start address 2nd byte table range
    AngleRad = (AngleRad * 616) / c1DEC; //616=acos resolution (pi/2/255) 
  }
  else if (Cos >= 9900 && Cos <= 10000) {
    AngleRad = (byte)pgm_read_byte(&cACosTable[(Cos - 9900) / 2 + 227]); //2=table resolution (0.01/64), 227 start address 3rd byte table range 
    AngleRad = (AngleRad * 616) / c1DEC; //616=acos resolution (pi/2/255) 
  }

  //Add negative sign
  if (NegativeValue) {
    AngleRad = 31416 - AngleRad;
  }
  return AngleRad;
}

//Simplified ArcTan2 function based on fixed point ArcCos
long GetATan2 (long AtanX, long AtanY) {
  return GetACos((AtanX * c6DEC) / sqrt((pow(AtanX, 2) * c4DEC) + (pow(AtanY, 2) * c4DEC))) * (AtanY / abs(AtanY));
}

//Body forward kinematics
void BodyFK (short PosX, short PosY, short PosZ, short RotY, byte LegIndex) {
  //Calculating totals from center of the body to the feet 
  TotalX = (short)pgm_read_word(&cOffsetX[LegIndex]) + PosX;
  TotalY = PosY;
  TotalZ = (short)pgm_read_word(&cOffsetZ[LegIndex]) + PosZ;

  //Successive global rotation matrix: 
  //Math shorts for rotation: Alfa [A] = X rotate, Beta [B] = Z rotate, Gamma [G] = Y rotate
  //Sinus Alfa = SinA, cosinus Alfa = cosA. and so on...

  //First calculate sinus and cosinus for each rotation:
  GetSinCos(BodyRotX + TotalBalX);
  SinG = Sin;
  CosG = Cos;

  GetSinCos(BodyRotZ + TotalBalZ);
  SinB = Sin;
  CosB = Cos;

  GetSinCos(BodyRotY + (RotY * c1DEC) + TotalBalY);
  SinA = Sin;
  CosA = Cos;

  //Calculation of rotation matrix:
  BodyFKPosX = ((long)TotalX * c2DEC - ((long)TotalX * c2DEC * CosA / c4DEC * CosB / c4DEC -
    (long)TotalZ * c2DEC * CosB / c4DEC * SinA / c4DEC + (long)TotalY * c2DEC * SinB / c4DEC)) / c2DEC;

  BodyFKPosZ = ((long)TotalZ * c2DEC - ((long)TotalX * c2DEC * CosG / c4DEC * SinA / c4DEC +
    (long)TotalX * c2DEC * CosA / c4DEC * SinB / c4DEC * SinG / c4DEC + (long)TotalZ * c2DEC * CosA / c4DEC * CosG / c4DEC -
    (long)TotalZ * c2DEC * SinA / c4DEC * SinB / c4DEC * SinG / c4DEC - (long)TotalY * c2DEC * CosB / c4DEC * SinG / c4DEC)) / c2DEC;

  BodyFKPosY = ((long)TotalY * c2DEC - ((long)TotalX * c2DEC * SinA / c4DEC * SinG / c4DEC -
    (long)TotalX * c2DEC * CosA / c4DEC * CosG / c4DEC * SinB / c4DEC + (long)TotalZ * c2DEC * CosA / c4DEC * SinG / c4DEC +
    (long)TotalZ * c2DEC * CosG / c4DEC * SinA / c4DEC * SinB / c4DEC + (long)TotalY * c2DEC * CosB / c4DEC * CosG / c4DEC)) / c2DEC;
}

//Calculates the angles of the coxa, femur and tibia for the given position of the feet
void LegIK (short PosX, short PosY, short PosZ, byte LegIndex) {
  //Length between the Coxa and tars (foot)
  PosXZ = sqrt((pow(PosX, 2) * c4DEC) + (pow(PosZ, 2) * c4DEC)) / c2DEC;

  //Length between femur axis and tars
  IKSW = sqrt((pow(PosY, 2) * c4DEC) + (pow(PosXZ - cCoxaLength, 2) * c4DEC));

  //Angle between SW line and the ground in radians
  IKA1 = GetATan2(PosY, PosXZ - cCoxaLength);

  //Angle of the line S>W with respect to the femur in radians
  IKA2 = GetACos(((pow(cFemurLength, 2) - pow(cTibiaLength, 2)) * c4DEC + pow(IKSW, 2)) / (((2 * cFemurLength) * c2DEC * IKSW) / c4DEC));

  CoxaAngle[LegIndex] = ((GetATan2(PosX, PosZ) * 180) / 3141) + (short)pgm_read_word(&cCoxaAngle[LegIndex]);
  FemurAngle[LegIndex] = -(IKA1 + IKA2) * 180 / 3141 + 900;
  TibiaAngle[LegIndex] = -(900 - GetACos(((pow(cFemurLength, 2) + pow(cTibiaLength, 2)) * c4DEC - pow(IKSW, 2)) / (2 * cFemurLength * cTibiaLength)) * 180 / 3141);
}

//Checks the mechanical limits of the servos
void CheckAngles() {
  for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
    CoxaAngle[LegIndex] = min(max(CoxaAngle[LegIndex], (short)pgm_read_word(&cCoxaMin[LegIndex])), (short)pgm_read_word(&cCoxaMax[LegIndex]));
    FemurAngle[LegIndex] = min(max(FemurAngle[LegIndex], (short)pgm_read_word(&cFemurMin[LegIndex])), (short)pgm_read_word(&cFemurMax[LegIndex]));
    TibiaAngle[LegIndex] = min(max(TibiaAngle[LegIndex], (short)pgm_read_word(&cTibiaMin[LegIndex])), (short)pgm_read_word(&cTibiaMax[LegIndex]));
  }
}

//Update the positions of the servos
void ServoDriverUpdate() {
  for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
    //Update Right Legs
    if (LegIndex <= 2) {
      CoxaPWM = ((long)(-CoxaAngle[LegIndex] + 900)) * 1000 / cPWMDiv + cPFCons;
      FemurPWM = ((long)(-FemurAngle[LegIndex] + 900)) * 1000 / cPWMDiv + cPFCons;
      TibiaPWM = ((long)(-TibiaAngle[LegIndex] + 900)) * 1000 / cPWMDiv + cPFCons;
    }
    else {
      //Update Left Legs
      CoxaPWM = ((long)(CoxaAngle[LegIndex] + 900)) * 1000 / cPWMDiv + cPFCons;
      FemurPWM = ((long)(FemurAngle[LegIndex] + 900)) * 1000 / cPWMDiv + cPFCons;
      TibiaPWM = ((long)(TibiaAngle[LegIndex] + 900)) * 1000 / cPWMDiv + cPFCons;
    }

#ifdef DEBUG_MODE
    if(DebugOutputOn) {
      DBGSerial.print(LegIndex + 1, DEC);
      DBGSerial.print(": ");
      DBGSerial.print(CoxaPWM, DEC);
      DBGSerial.print(" ");
      DBGSerial.print(FemurPWM, DEC);
      DBGSerial.print(" ");
      DBGSerial.print(TibiaPWM, DEC);
      if (LegIndex != 5) {
        DBGSerial.print(" | ");
      }
      if (LegIndex == 5) {
        DBGSerial.println();
      }
    }
#endif

    SSCWrite(pgm_read_byte(&cCoxaPin[LegIndex]) + 0x80, highByte(CoxaPWM), lowByte(CoxaPWM));
    SSCWrite(pgm_read_byte(&cFemurPin[LegIndex]) + 0x80, highByte(FemurPWM), lowByte(FemurPWM));
    SSCWrite(pgm_read_byte(&cTibiaPin[LegIndex]) + 0x80, highByte(TibiaPWM), lowByte(TibiaPWM));
  }
}

//Commit the positions of the servos
void ServoDriverCommit() {
  SSCWrite(0xA1, highByte(SSCTime), lowByte(SSCTime));
}

//Frees all the servos
void FreeServos() {
  for (LegIndex = 0; LegIndex <= 31; LegIndex++) {
    SSCWrite(LegIndex + 0x80, 0x00, 0x00);
  }
  SSCWrite(0xA1, 0x00, 0xC8);
}

//Write bytes to SSC
void SSCWrite(byte a, byte b, byte c) {
  Array[0] = a;
  Array[1] = b;
  Array[2] = c;
  SSCSerial.write(Array, 3);
}

