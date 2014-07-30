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
short    CoxaAngle[6];       //Actual angle of the horizontal hip
short    FemurAngle[6];      //Actual angle of the vertical hip
short    TibiaAngle[6];      //Actual angle of the knee

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

//Cos/Sin
float    Cos;                //Output cosinus of the given angle
float    Sin;                //Output sinus of the given angle

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
short    TotalTransZ;
short    TotalTransY;
short    TotalBalY;
short    TotalBalX;
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
short    TravelLengthZ;      //Current travel length Z
short    TravelLengthY;      //Current travel rotation Y

#ifdef DEBUG_MODE
bool     Prev_Walking;
bool     DebugOutputOn;
#endif

#ifdef SOUND_MODE
extern void Sound(byte Notes, ...);
#endif

void setup() {
  SSCSerial.begin(SSC_BAUD);

#ifdef DEBUG_MODE
  DBGSerial.begin(DBG_BAUD);
  DBGSerial.println("Start Debugging");
#endif

  //Setup Init Positions
  for (byte LegIndex = 0; LegIndex <= 5; LegIndex++) {
    LegPosX[LegIndex] = (short)pgm_read_word(&InitPosX[LegIndex]); //Set start positions for each leg
    LegPosY[LegIndex] = (short)pgm_read_word(&InitPosY[LegIndex]);
    LegPosZ[LegIndex] = (short)pgm_read_word(&InitPosZ[LegIndex]);
  }

  //Single leg control. Make sure no leg is selected
  SelectedLeg = 255; //No leg selected
  Prev_SelectedLeg = 255;

  //Gait
  GaitType = 0;
  BalanceMode = false;
  LegLiftHeight = 50;
  GaitStep = 1;
  GaitSelect();

  //Initialize controller
  InitControl();
}

void loop() {
  //Start time
  TimerStart = millis();

  //Read input
  InputControl();

  //Single leg control
  SingleLegControl();

  //Gait
  GaitSequence();

  //Balance calculations
  TotalTransX = 0; //Reset values used for calculation of balance
  TotalTransY = 0;
  TotalTransZ = 0;
  TotalBalX = 0;
  TotalBalY = 0;
  TotalBalZ = 0;

  if (BalanceMode) {
    for (byte LegIndex = 0; LegIndex <= 5; LegIndex++) {
      if (LegIndex <= 2) {
        //Balance calculations for all right legs
        BalanceLeg(-LegPosX[LegIndex] + GaitPosX[LegIndex],
        (LegPosY[LegIndex] - (short)pgm_read_word(&InitPosY[LegIndex])) + GaitPosY[LegIndex],
        LegPosZ[LegIndex] + GaitPosZ[LegIndex], LegIndex);
      }
      else {
        //Balance calculations for all left legs
        BalanceLeg(LegPosX[LegIndex] + GaitPosX[LegIndex],
        (LegPosY[LegIndex] - (short)pgm_read_word(&InitPosY[LegIndex])) + GaitPosY[LegIndex],
        LegPosZ[LegIndex] + GaitPosZ[LegIndex], LegIndex);
      }
    }
    BalanceBody();
  }

  for (byte LegIndex = 0; LegIndex <= 5; LegIndex++) {
    if (LegIndex <= 2) {
      //Do IK for all right legs
      BodyFK(-LegPosX[LegIndex] + BodyPosX + GaitPosX[LegIndex] - TotalTransX,
      LegPosY[LegIndex] + BodyPosY + GaitPosY[LegIndex] - TotalTransY,
      LegPosZ[LegIndex] + BodyPosZ + GaitPosZ[LegIndex] - TotalTransZ,
      GaitRotY[LegIndex], LegIndex);

      LegIK(LegPosX[LegIndex] - BodyPosX + BodyFKPosX - (GaitPosX[LegIndex] - TotalTransX), 
      LegPosY[LegIndex] + BodyPosY - BodyFKPosY + GaitPosY[LegIndex] - TotalTransY,
      LegPosZ[LegIndex] + BodyPosZ - BodyFKPosZ + GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
    }
    else {
      //Do IK for all left legs 
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

  //Drive servos
  if (HexOn) {
    if (HexOn && !Prev_HexOn) {
#ifdef SOUND_MODE
      Sound(3, 60, 1661, 80, 2217, 100, 2794);
#endif
    }

    //Set SSC time
    if ((abs(TravelLengthX) > TRAVEL_DEADZONE) || (abs(TravelLengthZ) > TRAVEL_DEADZONE) || (abs(TravelLengthY * 2) > TRAVEL_DEADZONE)) {

      SSCTime = NomGaitSpeed + (InputTimeDelay * 2) + SpeedControl;

      //Add additional delay when balance mode is on
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
    for (byte LegIndex = 0; LegIndex <= 5; LegIndex++) {
      if ((GaitPosX[LegIndex] > 2) || (GaitPosX[LegIndex] < -2) ||
          (GaitRotY[LegIndex] > 2) || (GaitRotY[LegIndex] < -2) ||
          (GaitPosZ[LegIndex] > 2) || (GaitPosZ[LegIndex] < -2)) {
        ExtraCycle = NrLiftedPos + 1; //For making sure that we are using timed move until all legs are down
        break;
      }
    }

    if (ExtraCycle > 0) {
      ExtraCycle--;
      Walking = !(ExtraCycle == 0);

      //Get endtime and calculate wait time
      byte CycleTime = (millis() - TimerStart);

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
    Sound(3, 100, 2794, 80, 2217, 60, 1661);
#endif
    delay(600);
  }
  else {
    FreeServos();
    delay(20);
  }

  Prev_SSCTime = SSCTime;

  //Store previous HexOn state
  if (HexOn) {
    Prev_HexOn = true;
  }
  else {
    Prev_HexOn = false;
  }
}

void SingleLegControl() {
  //Check if all legs are down
  AllDown = (LegPosY[RF] == (short)pgm_read_word(&InitPosY[RF])) &&
    (LegPosY[RM] == (short)pgm_read_word(&InitPosY[RM])) && 
    (LegPosY[RR] == (short)pgm_read_word(&InitPosY[RR])) && 
    (LegPosY[LR] == (short)pgm_read_word(&InitPosY[LR])) && 
    (LegPosY[LM] == (short)pgm_read_word(&InitPosY[LM])) && 
    (LegPosY[LF] == (short)pgm_read_word(&InitPosY[LF]));

  if (SelectedLeg <= 5) {
    if (SelectedLeg != Prev_SelectedLeg) {
      if (AllDown) { //Lift leg a bit when it got selected
        LegPosY[SelectedLeg] = (short)pgm_read_word(&InitPosY[SelectedLeg]) - 20;

        //Store current status
        Prev_SelectedLeg = SelectedLeg;
      }
      else { //Return prev leg back to the init position
        LegPosX[Prev_SelectedLeg] = (short)pgm_read_word(&InitPosX[Prev_SelectedLeg]);
        LegPosY[Prev_SelectedLeg] = (short)pgm_read_word(&InitPosY[Prev_SelectedLeg]);
        LegPosZ[Prev_SelectedLeg] = (short)pgm_read_word(&InitPosZ[Prev_SelectedLeg]);
      }
    }
    else if (!SLHold) {
      LegPosX[SelectedLeg] = (short)pgm_read_word(&InitPosX[SelectedLeg]) + SLLegX;
      LegPosY[SelectedLeg] = (short)pgm_read_word(&InitPosY[SelectedLeg]) + SLLegY;
      LegPosZ[SelectedLeg] = (short)pgm_read_word(&InitPosZ[SelectedLeg]) + SLLegZ;
    }
  }
  else { //All legs to init position
    if (!AllDown) {
      for(byte LegIndex = 0; LegIndex <= 5; LegIndex++) {
        LegPosX[LegIndex] = (short)pgm_read_word(&InitPosX[LegIndex]);
        LegPosY[LegIndex] = (short)pgm_read_word(&InitPosY[LegIndex]);
        LegPosZ[LegIndex] = (short)pgm_read_word(&InitPosZ[LegIndex]);
      }
    }
    if (Prev_SelectedLeg != 255) {
      Prev_SelectedLeg = 255;
    }
  }
}

void GaitSelect() {
  switch (GaitType) {
  case 0: //Ripple 12 steps
    GaitLegNr[LR] = 1;
    GaitLegNr[RF] = 3;
    GaitLegNr[LM] = 5;
    GaitLegNr[RR] = 7;
    GaitLegNr[LF] = 9;
    GaitLegNr[RM] = 11;

    NrLiftedPos = 3;
    LiftDivFactor = 2;
    HalfLiftHeight = 3;
    TLDivFactor = 8;
    StepsInGait = 12;
    NomGaitSpeed = 70;
    break;
  case 1: //Tripod 6 steps
    GaitLegNr[LR] = 4;
    GaitLegNr[RF] = 1;
    GaitLegNr[LM] = 1;
    GaitLegNr[RR] = 1;
    GaitLegNr[LF] = 4;
    GaitLegNr[RM] = 4;

    NrLiftedPos = 2;
    LiftDivFactor = 2;
    HalfLiftHeight = 1;
    TLDivFactor = 4;
    StepsInGait = 6;
    NomGaitSpeed = 60;
    break;
  case 2: //Tripod 8 steps
    GaitLegNr[LR] = 5;
    GaitLegNr[RF] = 1;
    GaitLegNr[LM] = 1;
    GaitLegNr[RR] = 1;
    GaitLegNr[LF] = 5;
    GaitLegNr[RM] = 5;

    NrLiftedPos = 3;
    LiftDivFactor = 2;
    HalfLiftHeight = 3;
    TLDivFactor = 4;
    StepsInGait = 8;
    NomGaitSpeed = 70;
    break;
  case 3: //Tripod 12 steps
    GaitLegNr[LR] = 11;
    GaitLegNr[RF] = 3;
    GaitLegNr[LM] = 4;
    GaitLegNr[RR] = 5;
    GaitLegNr[LF] = 9;
    GaitLegNr[RM] = 10;

    NrLiftedPos = 3;
    LiftDivFactor = 2;
    HalfLiftHeight = 3;
    TLDivFactor = 8;
    StepsInGait = 12;
    NomGaitSpeed = 60;
    break;
  case 4: //Tripod 16 steps, use 5 lifted positions!
    GaitLegNr[LR] = 14;
    GaitLegNr[RF] = 4;
    GaitLegNr[LM] = 5;
    GaitLegNr[RR] = 6;
    GaitLegNr[LF] = 12;
    GaitLegNr[RM] = 13;

    NrLiftedPos = 5;
    LiftDivFactor = 4;
    HalfLiftHeight = 1;
    TLDivFactor = 10;
    StepsInGait = 16;
    NomGaitSpeed = 60;
    break;
  case 5: //Wave 24 steps
    GaitLegNr[LR] = 1;
    GaitLegNr[RF] = 21;
    GaitLegNr[LM] = 5;
    GaitLegNr[RR] = 13;
    GaitLegNr[LF] = 9;
    GaitLegNr[RM] = 17;

    NrLiftedPos = 3;
    LiftDivFactor = 2;
    HalfLiftHeight = 3;
    TLDivFactor = 20;
    StepsInGait = 24;
    NomGaitSpeed = 70;
    break;
  }
}

void GaitSequence() {
  //Check if the gait is in motion
  TravelRequest = (abs(TravelLengthX) > TRAVEL_DEADZONE) || (abs(TravelLengthZ) > TRAVEL_DEADZONE) || (abs(TravelLengthY) > TRAVEL_DEADZONE) || Walking;

  //Calculate gait sequence
  for (byte LegIndex = 0; LegIndex <= 5; LegIndex++) {
    Gait(LegIndex);
  }

  //Advance to the next step
  GaitStep++;
  if (GaitStep > StepsInGait) {
    GaitStep = 1;
  }
}

void Gait(byte LegIndex) {
  //Clear values under the TRAVEL_DEADZONE
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

  //Optional half height Rear (2, 3, 5 lifted positions)
  else if (((NrLiftedPos == 2 && GaitStep == GaitLegNr[LegIndex]) || (NrLiftedPos >= 3 &&
    (GaitStep == GaitLegNr[LegIndex] - 1 || GaitStep == GaitLegNr[LegIndex] + (StepsInGait - 1)))) && TravelRequest) {
    GaitPosX[LegIndex] = -TravelLengthX / LiftDivFactor;
    GaitPosY[LegIndex] = -3 * LegLiftHeight / (3 + HalfLiftHeight); //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[LegIndex] = -TravelLengthZ / LiftDivFactor;
    GaitRotY[LegIndex] = -TravelLengthY / LiftDivFactor;
  }

  //Optional half height Front (2, 3, 5 lifted positions)
  else if ((NrLiftedPos >= 2) && (GaitStep == GaitLegNr[LegIndex] + 1 ||
    GaitStep == GaitLegNr[LegIndex] - (StepsInGait - 1)) && TravelRequest) {
    GaitPosX[LegIndex] = TravelLengthX / LiftDivFactor;
    GaitPosY[LegIndex] = -3 * LegLiftHeight / (3 + HalfLiftHeight); //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[LegIndex] = TravelLengthZ / LiftDivFactor;
    GaitRotY[LegIndex] = TravelLengthY / LiftDivFactor;
  }

  //Optional half height Rear (5 lifted positions)
  else if (((NrLiftedPos == 5 && (GaitStep == GaitLegNr[LegIndex] - 2))) && TravelRequest) {
    GaitPosX[LegIndex] = -TravelLengthX / 2;
    GaitPosY[LegIndex] = -LegLiftHeight / 2;
    GaitPosZ[LegIndex] = -TravelLengthZ / 2;
    GaitRotY[LegIndex] = -TravelLengthY / 2;
  }

  //Optional half height Front (5 lifted positions)
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
}

void BalanceLeg(short PosX, short PosY, short PosZ, byte LegIndex) {
  //Calculating totals from center of the body to the feet
  short TotalX = (short)pgm_read_word(&OffsetX[LegIndex]) + PosX;
  short TotalY = 150 + PosY; //Using the value 150 to lower the center point of rotation BodyPosY
  short TotalZ = (short)pgm_read_word(&OffsetZ[LegIndex]) + PosZ;

  TotalTransX += TotalX;
  TotalTransY += PosY;
  TotalTransZ += TotalZ;

  TotalBalX += atan2(TotalY, TotalZ) * 180 / PI - 90; //Rotate balance circle 90 deg
  TotalBalY += atan2(TotalZ, TotalX) * 180 / PI;
  TotalBalZ += atan2(TotalY, TotalX) * 180 / PI - 90; //Rotate balance circle 90 deg
}

void BalanceBody() {
  TotalTransX = TotalTransX / 6;
  TotalTransY = TotalTransY / 6;
  TotalTransZ = TotalTransZ / 6;

  if (TotalBalY > 0) { //Rotate balance circle by +/- 180 deg
    TotalBalY -= 180;
  }
  else {
    TotalBalY += 180;
  }

  if (TotalBalZ < -180) { //Compensate for extreme balance positions that causes overflow
    TotalBalZ += 360;
  }

  if (TotalBalX < -180) { //Compensate for extreme balance positions that causes overflow
    TotalBalX += 360;
  }

  //Balance rotation
  TotalBalX = -TotalBalX / 6;
  TotalBalY = -TotalBalY / 6;
  TotalBalZ = TotalBalZ / 6;
}

void GetSinCos(short AngleDeg) {
  //Get the absolute value of AngleDeg
  short ABSAngleDeg = abs(AngleDeg);

  //Shift rotation to a full circle of 360 deg -> AngleDeg // 360
  if (AngleDeg < 0) { //Negative values
    AngleDeg = 360 - (ABSAngleDeg - (360 * (ABSAngleDeg / 360)));
  }
  else { //Positive values
    AngleDeg = ABSAngleDeg - (360 * (ABSAngleDeg / 360));
  }

  if (AngleDeg < 180) { //Angle between 0 and 180
    AngleDeg = AngleDeg - 90; //Subtract 90 to shift range
    Sin = cos(radians(AngleDeg));
    Cos = -sin(radians(AngleDeg));
  }
  else { //Angle between 180 and 360
    AngleDeg = AngleDeg - 270; // Subtract 270 to shift range
    Sin = -cos(radians(AngleDeg));
    Cos = sin(radians(AngleDeg));
  }
}

void BodyFK(short PosX, short PosY, short PosZ, short RotY, byte LegIndex) {
  //Calculating totals from center of the body to the feet 
  short TotalX = (short)pgm_read_word(&OffsetX[LegIndex]) + PosX;
  short TotalY = PosY;
  short TotalZ = (short)pgm_read_word(&OffsetZ[LegIndex]) + PosZ;

  //Successive global rotation matrix:
  //Math shorts for rotation: Alfa [A] = X rotate, Beta [B] = Z rotate, Gamma [G] = Y rotate
  //Sinus Alfa = SinA, cosinus Alfa = cosA, and so on...

  //First calculate sinus and cosinus for each rotation
  GetSinCos((BodyRotX + TotalBalX) / 10);
  float SinG = Sin;
  float CosG = Cos;

  GetSinCos((BodyRotZ + TotalBalZ) / 10);
  float SinB = Sin;
  float CosB = Cos;

  GetSinCos((BodyRotY + (RotY * 10) + TotalBalY) / 10);
  float SinA = Sin;
  float CosA = Cos;

  //Calculation of rotation matrix
  BodyFKPosX = TotalX - (TotalX * CosA * CosB - TotalZ * CosB * SinA + TotalY * SinB);
  BodyFKPosY = TotalY - (TotalX * SinA * SinG - TotalX * CosA * CosG * SinB + TotalZ * CosA * SinG + TotalZ * CosG * SinA * SinB + TotalY * CosB * CosG);
  BodyFKPosZ = TotalZ - (TotalX * CosG * SinA + TotalX * CosA * SinB * SinG + TotalZ * CosA * CosG - TotalZ * SinA * SinB * SinG - TotalY * CosB * SinG);
}

void LegIK(short PosX, short PosY, short PosZ, byte LegIndex) {
  //Length between the coxa and feet
  short PosXZ = sqrt(pow(PosX, 2) + pow(PosZ, 2));

  //Length between shoulder and wrist
  float IKSW = sqrt(pow(PosXZ - CoxaLength, 2) + pow(PosY, 2));

  //Angle of the line S>W with respect to the ground in radians
  float IKA1 = atan2(PosXZ - CoxaLength, PosY);

  //Angle of the line S>W with respect to the femur in radians
  float IKA2 = acos((pow(FemurLength, 2) - pow(TibiaLength, 2) + pow(IKSW, 2)) / (2 * FemurLength * IKSW));

  CoxaAngle[LegIndex] = atan2(PosZ, PosX) * 180 / PI + (short)pgm_read_word(&LegAngle[LegIndex]);
  FemurAngle[LegIndex] = -(IKA1 + IKA2) * 180 / PI + 90;
  TibiaAngle[LegIndex] = -(90 - acos((pow(FemurLength, 2) + pow(TibiaLength, 2) - pow(IKSW, 2)) / (2 * FemurLength * TibiaLength)) * 180 / PI);
}

void CheckAngles() {
  for (byte LegIndex = 0; LegIndex <= 5; LegIndex++) {
    CoxaAngle[LegIndex] = min(max(CoxaAngle[LegIndex], (short)pgm_read_word(&CoxaMin[LegIndex])), (short)pgm_read_word(&CoxaMax[LegIndex]));
    FemurAngle[LegIndex] = min(max(FemurAngle[LegIndex], (short)pgm_read_word(&FemurMin[LegIndex])), (short)pgm_read_word(&FemurMax[LegIndex]));
    TibiaAngle[LegIndex] = min(max(TibiaAngle[LegIndex], (short)pgm_read_word(&TibiaMin[LegIndex])), (short)pgm_read_word(&TibiaMax[LegIndex]));
  }
}

void ServoDriverUpdate() {
  for (byte LegIndex = 0; LegIndex <= 5; LegIndex++) {
    word CoxaPWM, FemurPWM, TibiaPWM;
    //Update right legs
    if (LegIndex <= 2) {
      CoxaPWM = (-CoxaAngle[LegIndex] + 90) / 0.0991 + 592;
      FemurPWM = (-FemurAngle[LegIndex] + 90) / 0.0991 + 592;
      TibiaPWM = (-TibiaAngle[LegIndex] + 90) / 0.0991 + 592;
    }
    else {
      //Update left legs
      CoxaPWM = (CoxaAngle[LegIndex] + 90) / 0.0991 + 592;
      FemurPWM = (FemurAngle[LegIndex] + 90) / 0.0991 + 592;
      TibiaPWM = (TibiaAngle[LegIndex] + 90) / 0.0991 + 592;
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
      else {
        DBGSerial.println();
      }
    }
#endif

    SSCWrite(pgm_read_byte(&CoxaPin[LegIndex]) + 0x80, highByte(CoxaPWM), lowByte(CoxaPWM));
    SSCWrite(pgm_read_byte(&FemurPin[LegIndex]) + 0x80, highByte(FemurPWM), lowByte(FemurPWM));
    SSCWrite(pgm_read_byte(&TibiaPin[LegIndex]) + 0x80, highByte(TibiaPWM), lowByte(TibiaPWM));
  }
}

void ServoDriverCommit() {
  SSCWrite(0xA1, highByte(SSCTime), lowByte(SSCTime));
}

void FreeServos() {
  for (byte LegIndex = 0; LegIndex <= 31; LegIndex++) {
    SSCWrite(LegIndex + 0x80, 0x00, 0x00);
  }
  SSCWrite(0xA1, 0x00, 0xC8);
}

void SSCWrite(byte a, byte b, byte c) {
  byte Array[3];
  Array[0] = a;
  Array[1] = b;
  Array[2] = c;
  SSCSerial.write(Array, 3);
}
