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
#define c1DEC        10
#define c2DEC        100
#define c4DEC        10000
#define c6DEC        1000000

#define cRR          0
#define cRM          1
#define cRF          2
#define cLR          3
#define cLM          4
#define cLF          5

#define cPWMDiv      991
#define cPFCons      592

//[TABLES]
//ArcCosinus Table
//Table build in to 3 part to get higher accuracy near cos = 1.
//The biggest error is near cos = 1 and has a biggest value of 3*0.012098rad = 0.521 deg.
//- Cos 0 to 0.9 is done by steps of 0.0079 rad. [1/127]
//- Cos 0.9 to 0.99 is done by steps of 0.0008 rad [0.1/127]
//- Cos 0.99 to 1 is done by step of 0.0002 rad [0.01/64]
//Since the tables are overlapping the full range of 127+127+64 is not necessary. Total bytes: 277
static const byte GetACos[] PROGMEM = {
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

//Sin table 90 deg, persision 0.5 deg [180 values]
static const word GetSin[] PROGMEM = {
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
const byte cCoxaPin[] PROGMEM = {
  cRRCoxaPin, cRMCoxaPin, cRFCoxaPin, cLRCoxaPin, cLMCoxaPin, cLFCoxaPin };
const byte cFemurPin[] PROGMEM = {
  cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin };
const byte cTibiaPin[] PROGMEM = {
  cRRTibiaPin, cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin };

//Min / Max values
const short cCoxaMin1[] PROGMEM = {
  cRRCoxaMin1, cRMCoxaMin1, cRFCoxaMin1, cLRCoxaMin1, cLMCoxaMin1, cLFCoxaMin1 };
const short cCoxaMax1[] PROGMEM = {
  cRRCoxaMax1, cRMCoxaMax1, cRFCoxaMax1, cLRCoxaMax1, cLMCoxaMax1, cLFCoxaMax1 };
const short cFemurMin1[] PROGMEM ={
  cRRFemurMin1, cRMFemurMin1, cRFFemurMin1, cLRFemurMin1, cLMFemurMin1, cLFFemurMin1 };
const short cFemurMax1[] PROGMEM ={
  cRRFemurMax1, cRMFemurMax1, cRFFemurMax1, cLRFemurMax1, cLMFemurMax1, cLFFemurMax1 };
const short cTibiaMin1[] PROGMEM ={
  cRRTibiaMin1, cRMTibiaMin1, cRFTibiaMin1, cLRTibiaMin1, cLMTibiaMin1, cLFTibiaMin1 };
const short cTibiaMax1[] PROGMEM = {
  cRRTibiaMax1, cRMTibiaMax1, cRFTibiaMax1, cLRTibiaMax1, cLMTibiaMax1, cLFTibiaMax1 };

//Body Offsets [distance between the center of the body and the center of the coxa]
const short cOffsetX[] PROGMEM = {
  cRROffsetX, cRMOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLFOffsetX };
const short cOffsetZ[] PROGMEM = {
  cRROffsetZ, cRMOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLFOffsetZ };

//Default leg angle
const short cCoxaAngle1[] PROGMEM = {
  cRRCoxaAngle1, cRMCoxaAngle1, cRFCoxaAngle1, cLRCoxaAngle1, cLMCoxaAngle1, cLFCoxaAngle1 };

//Start positions for the leg
const short cInitPosX[] PROGMEM = {
  cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX };
const short cInitPosY[] PROGMEM = {
  cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY };
const short cInitPosZ[] PROGMEM = {
  cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ };

//[REMOTE]
#define cTravelDeadZone    4 //The deadzone for the analog input from the remote

//[ANGLES]
short    CoxaAngle1[6];      //Actual Angle of the horizontal hip, decimals = 1
short    FemurAngle1[6];     //Actual Angle of the vertical hip, decimals = 1
short    TibiaAngle1[6];     //Actual Angle of the knee, decimals = 1

//[POSITIONS SINGLE LEG CONTROL]
bool     SLHold;             //Single leg control mode
short    LegPosX[6];         //Actual X Position of the Leg
short    LegPosY[6];         //Actual Y Position of the Leg
short    LegPosZ[6];         //Actual Z Position of the Leg

//[VARIABLES]
byte     LegIndex;           //Index used for leg Index Number
word     ABSAngleDeg1;       //Absolute value of the Angle in Degrees, decimals = 1
short    Sin4;               //Output Sinus of the given Angle, decimals = 4
short    Cos4;               //Output Cosinus of the given Angle, decimals = 4
short    AngleRad4;          //Output Angle in radials, decimals = 4
bool     NegativeValue;      //If the the value is Negative
short    Atan4;              //ArcTan2 output
short    XYhyp2;             //Output presenting Hypotenuse of X and Y
short    BodyPosX;           //Global Input for the position of the body
short    BodyPosY;
short    BodyPosZ;
short    BodyRotX;           //Global Input pitch of the body
short    BodyRotY;           //Global Input rotation of the body
short    BodyRotZ;           //Global Input roll of the body
short    SinA4;              //Sin buffer for BodyRotX calculations
short    CosA4;              //Cos buffer for BodyRotX calculations
short    SinB4;              //Sin buffer for BodyRotX calculations
short    CosB4;              //Cos buffer for BodyRotX calculations
short    SinG4;              //Sin buffer for BodyRotZ calculations
short    CosG4;              //Cos buffer for BodyRotZ calculations
short    TotalX;             //Total X distance between the center of the body and the feet
short    TotalZ;             //Total Z distance between the center of the body and the feet
short    BodyFKPosX;         //Output Position X of feet with Rotation
short    BodyFKPosY;         //Output Position Y of feet with Rotation
short    BodyFKPosZ;         //Output Position Z of feet with Rotation
short    IKFeetPosXZ;        //Diagonal direction from Input X and Z
long     IKSW2;              //Length between Shoulder and Wrist, decimals = 2
long     IKA14;              //Angle of the line S>W with respect to the ground in radians, decimals = 4
long     IKA24;              //Angle of the line S>W with respect to the femur in radians, decimals = 4
long     Temp1;
long     Temp2;
//bool   IKSolution;         //Output true if the solution is possible
//bool   IKSolutionWarning;  //Output true if the solution is NEARLY possible
//bool   IKSolutionError;    //Output true if the solution is NOT possible

//[TIMING]
long     lTimerStart;        //Start time of the calculation cycles
long     lTimerEnd;          //End time of the calculation cycles
byte     CycleTime;          //Total Cycle time
word     SSCTime;            //Time for servo updates
word     Prev_SSCTime;       //Previous time for the servo updates
byte     InputTimeDelay;     //Delay that depends on the input to get the "sneaking" effect
word     SpeedControl;       //Adjustable Delay

//[GLOABAL]
bool     HexOn;              //Switch to turn on Phoenix
bool     Prev_HexOn;         //Previous loop state 

//[SERVO DRIVER]
word     CoxaPWM;
word     FemurPWM;
word     TibiaPWM;
byte     Array[3];

//[BALANCE]
bool     BalanceMode;
short    TotalTransX;
short    TotalTransZ;
short    TotalTransY;
short    TotalYBal;
short    TotalXBal;
short    TotalZBal;
short    TotalY;             //Total Y distance between the center of the body and the feet

//[SINGLE LEG CONTROL]
byte     SelectedLeg;
byte     Prev_SelectedLeg;
short    SLLegX;
short    SLLegY;
short    SLLegZ;
bool     AllDown;

//[GAIT]
byte     GaitType;           //Gait type
byte     NomGaitSpeed;       //Nominal speed of the gait
byte     LegLiftHeight;      //Current Travel height
short    TravelLengthX;      //Current Travel length X
short    TravelLengthZ;      //Current Travel length Z
short    TravelRotationY;    //Current Travel Rotation Y
byte     TLDivFactor;        //Number of steps that a leg is on the floor while walking
byte     NrLiftedPos;        //Number of positions that a single leg is lifted (1-3)
bool     HalfLiftHeigth;     //If TRUE the outer positions of the lighted legs will be half height
byte     LiftDivFactor;      //Normally: 2, when NrLiftedPos=5: 4
bool     TravelRequest;      //Temp to check if the gait is in motion
byte     StepsInGait;        //Number of steps in gait
bool     LastLeg;            //TRUE when the current leg is the last leg of the sequence
byte     GaitStep;           //Actual Gait step
byte     GaitLegNr[6];       //Init position of the leg
short    GaitPosX[6];        //Array containing Relative X position corresponding to the Gait
short    GaitPosY[6];        //Array containing Relative Y position corresponding to the Gait
short    GaitPosZ[6];        //Array containing Relative Z position corresponding to the Gait
short    GaitRotY[6];        //Array containing Relative Y rotation corresponding to the Gait  

#ifdef cBUZZER
extern void MSound(byte cNotes, ...);
#endif

//[INIT]
void setup() {
  SSCSerial.begin(cSSC_BAUD);

#ifdef DBGSerial
  DBGSerial.begin(cDBG_BAUD);
  DBGSerial.println("START DEBUGGING");
#endif

  //Tars Init Positions
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

//[MAIN]
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
  TotalTransZ = 0;
  TotalTransY = 0;
  TotalXBal = 0;
  TotalYBal = 0;
  TotalZBal = 0;

  if (BalanceMode) {
    for (LegIndex = 0; LegIndex <= 2; LegIndex++) { //Balance calculations for all Right legs
      BalCalcOneLeg(-LegPosX[LegIndex] + GaitPosX[LegIndex], LegPosZ[LegIndex] + GaitPosZ[LegIndex],
      (LegPosY[LegIndex] - (short)pgm_read_word(&cInitPosY[LegIndex])) + GaitPosY[LegIndex], LegIndex);
    }

    for (LegIndex = 3; LegIndex <= 5; LegIndex++) { //Balance calculations for all Left legs
      BalCalcOneLeg(LegPosX[LegIndex] + GaitPosX[LegIndex], LegPosZ[LegIndex] + GaitPosZ[LegIndex], 
      (LegPosY[LegIndex] - (short)pgm_read_word(&cInitPosY[LegIndex])) + GaitPosY[LegIndex], LegIndex);
    }
    BalanceBody();
  }

  //Reset IKsolution indicators
  //IKSolution = 0;
  //IKSolutionWarning = 0;
  //IKSolutionError = 0;

  //Do IK for all Right legs
  for (LegIndex = 0; LegIndex <= 2; LegIndex++) {
    BodyFK(-LegPosX[LegIndex] + BodyPosX + GaitPosX[LegIndex] - TotalTransX,
    LegPosZ[LegIndex] + BodyPosZ + GaitPosZ[LegIndex] - TotalTransZ,
    LegPosY[LegIndex] + BodyPosY + GaitPosY[LegIndex] - TotalTransY,
    GaitRotY[LegIndex], LegIndex);

    LegIK(LegPosX[LegIndex] - BodyPosX + BodyFKPosX - (GaitPosX[LegIndex] - TotalTransX), 
    LegPosY[LegIndex] + BodyPosY - BodyFKPosY + GaitPosY[LegIndex] - TotalTransY,
    LegPosZ[LegIndex] + BodyPosZ - BodyFKPosZ + GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
  }

  //Do IK for all Left legs 
  for (LegIndex = 3; LegIndex <= 5; LegIndex++) {
    BodyFK(LegPosX[LegIndex] - BodyPosX + GaitPosX[LegIndex] - TotalTransX,
    LegPosZ[LegIndex] + BodyPosZ + GaitPosZ[LegIndex] - TotalTransZ,
    LegPosY[LegIndex] + BodyPosY + GaitPosY[LegIndex] - TotalTransY,
    GaitRotY[LegIndex], LegIndex);

    LegIK(LegPosX[LegIndex] + BodyPosX - BodyFKPosX + GaitPosX[LegIndex] - TotalTransX,
    LegPosY[LegIndex] + BodyPosY - BodyFKPosY + GaitPosY[LegIndex] - TotalTransY,
    LegPosZ[LegIndex] + BodyPosZ - BodyFKPosZ + GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
  }

  //Check mechanical limits
  CheckAngles();

  //Drive Servos
  if (HexOn) {
    if (HexOn && !Prev_HexOn) {
#ifdef cBUZZER
      MSound(3, 60, 2000, 80, 2250, 100, 2500);
#endif
    }

    //Set SSC time
    if ((abs(TravelLengthX) > cTravelDeadZone) || (abs(TravelLengthZ) > cTravelDeadZone) || (abs(TravelRotationY * 2 ) > cTravelDeadZone)) {

      SSCTime = NomGaitSpeed + (InputTimeDelay * 2) + SpeedControl;

      //Add additional delay when Balance mode is on
      if (BalanceMode) {
        SSCTime += 100;
      }
    }
    else { //Movement speed excl. Walking
      SSCTime = 200 + SpeedControl;
    }

    //Update servo positions without commiting
    ServoDriverUpdate();

    //Sync BAP with SSC while walking to ensure the prev is completed before sending the next one
    if (GaitPosX[cRF] || GaitPosX[cRM] || GaitPosX[cRR] || GaitPosX[cLF] || GaitPosX[cLM] || GaitPosX[cLR] ||
      GaitPosY[cRF] || GaitPosY[cRM] || GaitPosY[cRR] || GaitPosY[cLF] || GaitPosY[cLM] || GaitPosY[cLR] ||
      GaitPosZ[cRF] || GaitPosZ[cRM] || GaitPosZ[cRR] || GaitPosZ[cLF] || GaitPosZ[cLM] || GaitPosZ[cLR] ||
      GaitRotY[cRF] || GaitRotY[cRM] || GaitRotY[cRR] || GaitRotY[cLF] || GaitRotY[cLM] || GaitRotY[cLR]) {

      //Get endtime and calculate wait time
      lTimerEnd = millis();
      CycleTime = (lTimerEnd - lTimerStart);

      //Wait for previous commands to be completed while walking
      delay(max(Prev_SSCTime - CycleTime, 1)); //Min 1 ensures that there alway is a value in the pause command
    }

    //Commit servo positions
    ServoDriverCommit();
  }
  else if (Prev_HexOn || !AllDown) { //Turn the bot off
    SSCTime = 600;
    ServoDriverUpdate();
    ServoDriverCommit();
#ifdef cBUZZER
    MSound(3, 100, 2500, 80, 2250, 60, 2000);
#endif
    delay(600);
  }
  else {
    FreeServos();
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

//[SINGLE LEG CONTROL]
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

//[GAIT SELECT]
void GaitSelect() {
  //Gait selector
  switch (GaitType) {
  case 0: //Ripple Gait 12 steps
    GaitLegNr[cLR] = 1;
    GaitLegNr[cRF] = 3;
    GaitLegNr[cLM] = 5;
    GaitLegNr[cRR] = 7;
    GaitLegNr[cLF] = 9;
    GaitLegNr[cRM] = 11;

    NrLiftedPos = 3;
    HalfLiftHeigth = 3;
    TLDivFactor = 8;
    StepsInGait = 12;
    NomGaitSpeed = 70;
    break;
  case 1: //Tripod 8 steps
    GaitLegNr[cLR] = 5;
    GaitLegNr[cRF] = 1;
    GaitLegNr[cLM] = 1;
    GaitLegNr[cRR] = 1;
    GaitLegNr[cLF] = 5;
    GaitLegNr[cRM] = 5;

    NrLiftedPos = 3;
    HalfLiftHeigth = 3;
    TLDivFactor = 4;
    StepsInGait = 8;
    NomGaitSpeed = 70;
    break;
  case 2: //Triple Tripod 12 steps
    GaitLegNr[cRF] = 3;
    GaitLegNr[cLM] = 4;
    GaitLegNr[cRR] = 5;
    GaitLegNr[cLF] = 9;
    GaitLegNr[cRM] = 10;
    GaitLegNr[cLR] = 11;

    NrLiftedPos = 3;
    HalfLiftHeigth = 3;
    TLDivFactor = 8;
    StepsInGait = 12;
    NomGaitSpeed = 60;
    break;
  case 3: //Triple Tripod 16 steps, use 5 lifted positions!
    GaitLegNr[cLR] = 4;
    GaitLegNr[cRF] = 5;
    GaitLegNr[cLM] = 6;
    GaitLegNr[cRR] = 12;
    GaitLegNr[cLF] = 13;
    GaitLegNr[cRM] = 14;

    NrLiftedPos = 5;
    HalfLiftHeigth = 1;
    TLDivFactor = 10;
    StepsInGait = 16;
    NomGaitSpeed = 60;
    break;
  case 4: //Wave 24 steps
    GaitLegNr[cLR] = 1;
    GaitLegNr[cRF] = 21;
    GaitLegNr[cLM] = 5;
    GaitLegNr[cRR] = 13;
    GaitLegNr[cLF] = 9;
    GaitLegNr[cRM] = 17;

    NrLiftedPos = 3;
    HalfLiftHeigth = 3;
    TLDivFactor = 20;
    StepsInGait = 24;
    NomGaitSpeed = 70;
    break;
  }
}

//[GAIT SEQUENCE]
void GaitSeq() {
  //Check IF the Gait is in motion
  TravelRequest = (abs(TravelLengthX) > cTravelDeadZone) || (abs(TravelLengthZ) > cTravelDeadZone) || (abs(TravelRotationY) > cTravelDeadZone);

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

//[GAIT]
void Gait(byte GaitCurrentLegNr) {
  //Clear values under the cTravelDeadZone
  if (!TravelRequest) {
    TravelLengthX = 0;
    TravelLengthZ = 0;
    TravelRotationY = 0;
  }

  //Leg middle up position
  if ((TravelRequest && (NrLiftedPos == 1 || NrLiftedPos == 3 || NrLiftedPos == 5) &&
    GaitStep == GaitLegNr[GaitCurrentLegNr]) || (!TravelRequest && GaitStep == GaitLegNr[GaitCurrentLegNr] &&
    ((abs(GaitPosX[GaitCurrentLegNr]) > 2) || (abs(GaitPosZ[GaitCurrentLegNr]) > 2) || (abs(GaitRotY[GaitCurrentLegNr]) > 2)))) {
    GaitPosX[GaitCurrentLegNr] = 0;
    GaitPosY[GaitCurrentLegNr] = -LegLiftHeight;
    GaitPosZ[GaitCurrentLegNr] = 0;
    GaitRotY[GaitCurrentLegNr] = 0;
  }

  //Optional Half height Rear (2, 3, 5 lifted positions)
  else if (((NrLiftedPos == 2 && GaitStep==GaitLegNr[GaitCurrentLegNr]) || (NrLiftedPos >= 3 &&
    (GaitStep == GaitLegNr[GaitCurrentLegNr] - 1 || GaitStep == GaitLegNr[GaitCurrentLegNr] + (StepsInGait - 1)))) && TravelRequest) {
    GaitPosX[GaitCurrentLegNr] = -TravelLengthX / LiftDivFactor;
    GaitPosY[GaitCurrentLegNr] = -3 * LegLiftHeight / (3 + HalfLiftHeigth); //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[GaitCurrentLegNr] = -TravelLengthZ / LiftDivFactor;
    GaitRotY[GaitCurrentLegNr] = -TravelRotationY / LiftDivFactor;
  }

  //Optional Half height Front (2, 3, 5 lifted positions)
  else if ((NrLiftedPos >= 2) && (GaitStep == GaitLegNr[GaitCurrentLegNr] + 1 ||
    GaitStep == GaitLegNr[GaitCurrentLegNr] - (StepsInGait - 1)) && TravelRequest) {
    GaitPosX[GaitCurrentLegNr] = TravelLengthX / LiftDivFactor;
    GaitPosY[GaitCurrentLegNr] = -3 * LegLiftHeight / (3 + HalfLiftHeigth); //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[GaitCurrentLegNr] = TravelLengthZ / LiftDivFactor;
    GaitRotY[GaitCurrentLegNr] = TravelRotationY / LiftDivFactor;
  }

  //Optional Half height Rear 5 LiftedPos (5 lifted positions)
  else if (((NrLiftedPos == 5 && (GaitStep == GaitLegNr[GaitCurrentLegNr] - 2 ))) && TravelRequest) {
    GaitPosX[GaitCurrentLegNr] = -TravelLengthX / 2;
    GaitPosY[GaitCurrentLegNr] = -LegLiftHeight / 2;
    GaitPosZ[GaitCurrentLegNr] = -TravelLengthZ / 2;
    GaitRotY[GaitCurrentLegNr] = -TravelRotationY / 2;
  }

  //Optional Half height Front 5 LiftedPos (5 lifted positions)
  else if ((NrLiftedPos == 5) && (GaitStep == GaitLegNr[GaitCurrentLegNr] + 2 ||
    GaitStep == GaitLegNr[GaitCurrentLegNr] - (StepsInGait - 2)) && TravelRequest) {
    GaitPosX[GaitCurrentLegNr] = TravelLengthX / 2;
    GaitPosY[GaitCurrentLegNr] = -LegLiftHeight / 2;
    GaitPosZ[GaitCurrentLegNr] = TravelLengthZ / 2;
    GaitRotY[GaitCurrentLegNr] = TravelRotationY / 2;
  }

  //Leg front down position
  else if ((GaitStep == GaitLegNr[GaitCurrentLegNr] + NrLiftedPos ||
    GaitStep == GaitLegNr[GaitCurrentLegNr] - (StepsInGait - NrLiftedPos)) && GaitPosY[GaitCurrentLegNr] < 0) {
    GaitPosX[GaitCurrentLegNr] = TravelLengthX / 2;
    GaitPosZ[GaitCurrentLegNr] = TravelLengthZ / 2;
    GaitRotY[GaitCurrentLegNr] = TravelRotationY / 2;
    GaitPosY[GaitCurrentLegNr] = 0; //Only move leg down at once if terrain adaptation is turned off
  }

  //Move body forward
  else {
    GaitPosX[GaitCurrentLegNr] = GaitPosX[GaitCurrentLegNr] - (TravelLengthX / TLDivFactor);
    GaitPosY[GaitCurrentLegNr] = 0;
    GaitPosZ[GaitCurrentLegNr] = GaitPosZ[GaitCurrentLegNr] - (TravelLengthZ / TLDivFactor);
    GaitRotY[GaitCurrentLegNr] = GaitRotY[GaitCurrentLegNr] - (TravelRotationY / TLDivFactor);
  }

  //Advance to the next step
  if (LastLeg) { //The last leg in this step
    GaitStep++;
    if (GaitStep > StepsInGait) {
      GaitStep = 1;
    }
  }
}

//[BALCALCONELEG]
void BalCalcOneLeg (long PosX, long PosZ, long PosY, byte BalLegNr) {
  //Calculating totals from center of the body to the feet
  TotalZ = (short)pgm_read_word(&cOffsetZ[BalLegNr]) + PosZ;
  TotalX = (short)pgm_read_word(&cOffsetX[BalLegNr]) + PosX;
  TotalY = 150 + PosY; //Using the value 150 to lower the center point of rotation BodyPosY
  TotalTransY += PosY;
  TotalTransZ += TotalZ;
  TotalTransX += TotalX;

  Atan4 = GetATan2(TotalX, TotalZ);
  TotalYBal += (Atan4 * 1800) / 31415;

  Atan4 = GetATan2(TotalX, TotalY);
  TotalZBal += ((Atan4 * 1800) / 31415) - 900; //Rotate balance circle 90 deg

  Atan4 = GetATan2(TotalZ, TotalY);
  TotalXBal += ((Atan4 * 1800) / 31415) - 900; //Rotate balance circle 90 deg
}

//[BALANCE BODY]
void BalanceBody() {
  TotalTransZ = TotalTransZ / 6;
  TotalTransX = TotalTransX / 6;
  TotalTransY = TotalTransY / 6;

  if (TotalYBal > 0) { //Rotate balance circle by +/- 180 deg
    TotalYBal -= 1800;
  }
  else {
    TotalYBal += 1800;
  }

  if (TotalZBal < -1800) { //Compensate for extreme balance positions that causes overflow
    TotalZBal += 3600;
  }

  if (TotalXBal < -1800) { //Compensate for extreme balance positions that causes overflow
    TotalXBal += 3600;
  }

  //Balance rotation
  TotalYBal = -TotalYBal / 6;
  TotalXBal = -TotalXBal / 6;
  TotalZBal = TotalZBal / 6;
}

//[GETSINCOS] Get the sinus and cosinus from the angle +/- multiple circles
//AngleDeg1 - Input Angle in degrees
//Sin4 - Output Sinus of AngleDeg
//Cos4 - Output Cosinus of AngleDeg
void GetSinCos(short AngleDeg1) {
  //Get the absolute value of AngleDeg
  if (AngleDeg1 < 0) {
    ABSAngleDeg1 = AngleDeg1 * -1;
  }
  else {
    ABSAngleDeg1 = AngleDeg1;
  }

  //Shift rotation to a full circle of 360 deg -> AngleDeg // 360
  if (AngleDeg1 < 0) { //Negative values
    AngleDeg1 = 3600 - (ABSAngleDeg1 - (3600 * (ABSAngleDeg1 / 3600)));
  }
  else { //Positive values
    AngleDeg1 = ABSAngleDeg1 - (3600 * (ABSAngleDeg1 / 3600));
  }

  if (AngleDeg1 >= 0 && AngleDeg1 <= 900) { //0 to 90 deg
    Sin4 = pgm_read_word(&GetSin[AngleDeg1 / 5]); //5 is the presision (0.5) of the table
    Cos4 = pgm_read_word(&GetSin[(900 - (AngleDeg1)) / 5]);
  }
  else if (AngleDeg1 > 900 && AngleDeg1 <= 1800) { //90 to 180 deg
    Sin4 = pgm_read_word(&GetSin[(900 - (AngleDeg1 - 900)) / 5]); //5 is the presision (0.5) of the table 
    Cos4 = -pgm_read_word(&GetSin[(AngleDeg1 - 900) / 5]);
  }
  else if (AngleDeg1 > 1800 && AngleDeg1 <= 2700) { //180 to 270 deg
    Sin4 = -pgm_read_word(&GetSin[(AngleDeg1 - 1800) / 5]); //5 is the presision (0.5) of the table
    Cos4 = -pgm_read_word(&GetSin[(2700 - AngleDeg1) / 5]);
  }
  else if(AngleDeg1 > 2700 && AngleDeg1 <= 3600) { //270 to 360 deg
    Sin4 = -pgm_read_word(&GetSin[(3600 - AngleDeg1) / 5]); //5 is the presision (0.5) of the table 
    Cos4 = pgm_read_word(&GetSin[(AngleDeg1 - 2700) / 5]);
  }
}

//[GETARCCOS] Get the sinus and cosinus from the angle +/- multiple circles
//Cos4 - Input Cosinus
//AngleRad4 - Output Angle in AngleRad4
long GetArcCos(short Cos4) {
  //Check for negative value
  if (Cos4 < 0) {
    Cos4 = -Cos4;
    NegativeValue = 1;
  }
  else {
    NegativeValue = 0;
  }

  //Limit Cos4 to his maximal value
  Cos4 = min(Cos4, c4DEC);

  if (Cos4 >= 0 && Cos4 < 9000) {
    AngleRad4 = (byte)pgm_read_byte(&GetACos[Cos4 / 79]); //79=table resolution (1/127)
    AngleRad4 = (AngleRad4 * 616) / c1DEC; //616=acos resolution (pi/2/255)
  }
  else if (Cos4 >= 9000 && Cos4 < 9900) {
    AngleRad4 = (byte)pgm_read_byte(&GetACos[(Cos4 - 9000) / 8 + 114]); //8=table resolution (0.1/127), 114 start address 2nd bytetable range
    AngleRad4 = (AngleRad4 * 616) / c1DEC; //616=acos resolution (pi/2/255) 
  }
  else if (Cos4 >= 9900 && Cos4 <= 10000) {
    AngleRad4 = (byte)pgm_read_byte(&GetACos[(Cos4 - 9900) / 2 + 227]); //2=table resolution (0.01/64), 227 start address 3rd bytetable range 
    AngleRad4 = (AngleRad4 * 616) / c1DEC; //616=acos resolution (pi/2/255) 
  }

  //Add negative sign
  if (NegativeValue) {
    AngleRad4 = 31416 - AngleRad4;
  }
  return AngleRad4;
}

//[GETATAN2] Simplified ArcTan2 function based on fixed point ArcCos
//ArcTanX - Input X
//ArcTanY - Input Y
//ArcTan4 - Output ARCTAN2(X/Y)
//XYhyp2 - Output presenting Hypotenuse of X and Y
short GetATan2 (short AtanX, short AtanY) {
  XYhyp2 = sqrt((AtanX * AtanX * c4DEC) + (AtanY * AtanY * c4DEC));
  AngleRad4 = GetArcCos((AtanX * c6DEC) / XYhyp2);

  if (AtanY < 0) { //removed overhead... Atan4 = AngleRad4 * (AtanY/abs(AtanY));
    Atan4 = -AngleRad4;
  }
  else {
    Atan4 = AngleRad4;
  }
  return Atan4;
}

//[BODY FORWARD KINEMATICS]
//BodyRotX - Global Input pitch of the body
//BodyRotY - Global Input rotation of the body
//BodyRotZ - Global Input roll of the body
//RotationY - Input Rotation for the gait
//PosX - Input position of the feet X 
//PosZ - Input position of the feet Z
//SinB - Sin buffer for BodyRotX
//CosB - Cos buffer for BodyRotX
//SinG - Sin buffer for BodyRotZ
//CosG - Cos buffer for BodyRotZ
//BodyFKPosX - Output Position X of feet with Rotation
//BodyFKPosY - Output Position Y of feet with Rotation
//BodyFKPosZ - Output Position Z of feet with Rotation
void BodyFK (short PosX, short PosZ, short PosY, short RotationY, byte BodyFKLeg) {
  //Calculating totals from center of the body to the feet 
  TotalZ = (short)pgm_read_word(&cOffsetZ[BodyFKLeg]) + PosZ;
  TotalX = (short)pgm_read_word(&cOffsetX[BodyFKLeg]) + PosX;
  //PosY are equal to a "TotalY"

  //Successive global rotation matrix: 
  //Math shorts for rotation: Alfa [A] = X rotate, Beta [B] = Z rotate, Gamma [G] = Y rotate
  //Sinus Alfa = SinA, cosinus Alfa = cosA. and so on...

  //First calculate sinus and cosinus for each rotation:
  GetSinCos(BodyRotX + TotalXBal);
  SinG4 = Sin4;
  CosG4 = Cos4;

  GetSinCos(BodyRotZ + TotalZBal);
  SinB4 = Sin4;
  CosB4 = Cos4;

  GetSinCos(BodyRotY + (RotationY * c1DEC) + TotalYBal);
  SinA4 = Sin4;
  CosA4 = Cos4;

  //Calculation of rotation matrix:
  BodyFKPosX = (TotalX * c2DEC - (TotalX * c2DEC * CosA4 / c4DEC * CosB4 / c4DEC -
    TotalZ * c2DEC * CosB4 / c4DEC * SinA4 / c4DEC + PosY * c2DEC * SinB4 / c4DEC )) / c2DEC;

  BodyFKPosZ = (TotalZ * c2DEC - (TotalX * c2DEC * CosG4 / c4DEC * SinA4 / c4DEC +
    TotalX * c2DEC * CosA4 / c4DEC * SinB4 / c4DEC * SinG4 / c4DEC + TotalZ * c2DEC * CosA4 / c4DEC * CosG4 / c4DEC -
    TotalZ * c2DEC * SinA4 / c4DEC * SinB4 / c4DEC * SinG4 / c4DEC - PosY * c2DEC * CosB4 / c4DEC * SinG4 / c4DEC )) / c2DEC;

  BodyFKPosY = (PosY * c2DEC - (TotalX * c2DEC * SinA4 / c4DEC * SinG4 / c4DEC -
    TotalX * c2DEC * CosA4 / c4DEC * CosG4 / c4DEC * SinB4 / c4DEC + TotalZ * c2DEC * CosA4 / c4DEC * SinG4 / c4DEC +
    TotalZ * c2DEC * CosG4 / c4DEC * SinA4 / c4DEC * SinB4 / c4DEC + PosY * c2DEC * CosB4 / c4DEC * CosG4 / c4DEC )) / c2DEC;
}

//[LEG INVERSE KINEMATICS] Calculates the angles of the coxa, femur and tibia for the given position of the feet
//IKFeetPosX - Input position of the Feet X
//IKFeetPosY - Input position of the Feet Y
//IKFeetPosZ - Input Position of the Feet Z
//IKSolution - Output true if the solution is possible
//IKSolutionWarning - Output true if the solution is NEARLY possible
//IKSolutionError - Output true if the solution is NOT possible
//FemurAngle1 - Output Angle of Femur in degrees
//TibiaAngle1 - Output Angle of Tibia in degrees
//CoxaAngle1 - Output Angle of Coxa in degrees
void LegIK (short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ, byte LegIKLegNr) {
  //Calculate IKCoxaAngle and IKFeetPosXZ
  Atan4 = GetATan2(IKFeetPosX, IKFeetPosZ);
  CoxaAngle1[LegIKLegNr] = ((Atan4 * 180) / 3141) + (short)pgm_read_word(&cCoxaAngle1[LegIKLegNr]);

  //Length between the Coxa and tars (foot)
  IKFeetPosXZ = XYhyp2 / c2DEC;

  //Using GetAtan2 for solving IKA1 and IKSW
  //IKA14 - Angle between SW line and the ground in radians
  IKA14 = GetATan2(IKFeetPosY, IKFeetPosXZ - cCoxaLength);

  //IKSW2 - Length between femur axis and tars
  IKSW2 = XYhyp2;

  //IKA2 - Angle of the line S>W with respect to the femur in radians
  Temp1 = (((cFemurLength * cFemurLength) - (cTibiaLength * cTibiaLength)) * c4DEC + (IKSW2 * IKSW2));
  Temp2 = ((2 * cFemurLength) * c2DEC * IKSW2);
  IKA24 = GetArcCos(Temp1 / (Temp2 / c4DEC));

  //IKFemurAngle
  FemurAngle1[LegIKLegNr] = -(IKA14 + IKA24) * 180 / 3141 + 900;

  //IKTibiaAngle
  Temp1 = (((cFemurLength * cFemurLength) + (cTibiaLength * cTibiaLength)) * c4DEC - (IKSW2 * IKSW2));
  Temp2 = (2 * cFemurLength * cTibiaLength);
  AngleRad4 = GetArcCos(Temp1 / Temp2);
  TibiaAngle1[LegIKLegNr] = -(900 - AngleRad4 * 180 / 3141);

  //Set the Solution quality 
  //if(IKSW2 < (cFemurLength + cTibiaLength - 30) * c2DEC) {
  //  IKSolution = 1;
  //}
  //else if(IKSW2 < (cFemurLength + cTibiaLength) * c2DEC) {
  //  IKSolutionWarning = 1;
  //}
  //else {
  //  IKSolutionError = 1;
  //}
}

//[CHECK ANGLES] Checks the mechanical limits of the servos
void CheckAngles() {
  for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
    CoxaAngle1[LegIndex] = min(max(CoxaAngle1[LegIndex], (short)pgm_read_word(&cCoxaMin1[LegIndex])),
    (short)pgm_read_word(&cCoxaMax1[LegIndex]));

    FemurAngle1[LegIndex] = min(max(FemurAngle1[LegIndex], (short)pgm_read_word(&cFemurMin1[LegIndex])),
    (short)pgm_read_word(&cFemurMax1[LegIndex]));

    TibiaAngle1[LegIndex] = min(max(TibiaAngle1[LegIndex], (short)pgm_read_word(&cTibiaMin1[LegIndex])),
    (short)pgm_read_word(&cTibiaMax1[LegIndex]));
  }
}

//[SERVO DRIVER UPDATE] Update the positions of the servos
void ServoDriverUpdate() {
  for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
    //Update Right Legs
    if (LegIndex <= 2) {
      CoxaPWM = ((word)(-pgm_read_byte(&CoxaAngle1[LegIndex]) + 900)) * 1000 / cPWMDiv + cPFCons;
      FemurPWM = ((word)(-pgm_read_byte(&FemurAngle1[LegIndex]) + 900)) * 1000 / cPWMDiv + cPFCons;
      TibiaPWM = ((word)(-pgm_read_byte(&TibiaAngle1[LegIndex]) + 900)) * 1000 / cPWMDiv + cPFCons;
    }
    else {
      //Update Left Legs
      CoxaPWM = ((word)(pgm_read_byte(&CoxaAngle1[LegIndex]) + 900)) * 1000 / cPWMDiv + cPFCons;
      FemurPWM = ((word)(pgm_read_byte(&FemurAngle1[LegIndex]) + 900)) * 1000 / cPWMDiv + cPFCons;
      TibiaPWM = ((word)(pgm_read_byte(&TibiaAngle1[LegIndex]) + 900)) * 1000 / cPWMDiv + cPFCons;
    }

    SSCWrite(pgm_read_byte(&cCoxaPin[LegIndex]) + 0x80, CoxaPWM >> 8, CoxaPWM & 0xFF);
    SSCWrite(pgm_read_byte(&cFemurPin[LegIndex]) + 0x80, FemurPWM >> 8, FemurPWM & 0xFF);
    SSCWrite(pgm_read_byte(&cTibiaPin[LegIndex]) + 0x80, TibiaPWM >> 8, TibiaPWM & 0xFF);
  }
}

//[SERVO DRIVER COMMIT] Commit the positions of the servos
void ServoDriverCommit() {
  SSCWrite(0xA1, SSCTime >> 8, SSCTime & 0xFF);
}

//[FREE SERVOS] Frees all the servos
void FreeServos() {
  for (LegIndex = 0; LegIndex <= 31; LegIndex++) {
    SSCWrite(LegIndex + 0x80, 0x00, 0x00);
  }
  SSCWrite(0xA1, 0x00, 0xC8);
}

//[SSC WRITE] Write bytes to SSC
void SSCWrite(byte a, byte b, byte c) {
  Array[0] = a;
  Array[1] = b;
  Array[2] = c;
  SSCSerial.write(Array, 3);
}
