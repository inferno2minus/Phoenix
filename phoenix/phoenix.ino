/**
 * Project Lynxmotion Phoenix
 * Description: Phoenix software
 * Version: v2.0
 * Programmer: Jeroen Janssen (aka Xan)
 *             Kompanets Konstantin (aka I2M)
 *
 * Hardware setup: Arduino, SSC32 V2
 */

#include <MiniTone.h>
#include <SoftwareSerial.h>
#include "phoenix_cfg.h"
#include "phoenix.h"

//[0] NrLiftedPos    — Number of positions that a single leg is lifted (1-5)
//[1] FrontDownPos   — Where the leg should be put down to ground
//[2] LiftDivFactor  — Default: 2, when NrLiftedPos = 5: 4
//[3] HalfLiftHeight — How high to lift at halfway up
//[4] TLDivFactor    — Number of steps that a leg is on the floor while walking
//[5] StepsInGait    — Number of steps in gait
//[6] NomGaitSpeed   — Nominal speed of the gait
//[7] GaitLegNr[6]   — Init position of the leg (LR, RF, LM, RR, LF, RM)

const gait Gaits[] = {
  { 3,  2,  2,  3,  8, 12, 70, {  1,  3,  5,  7,  9, 11 } }, //Ripple 12 steps
  { 2,  1,  2,  1,  4,  6, 60, {  4,  1,  1,  1,  4,  4 } }, //Tripod 6 steps
  { 3,  2,  2,  3,  4,  8, 70, {  5,  1,  1,  1,  5,  5 } }, //Tripod 8 steps
  { 3,  2,  2,  3,  8, 12, 60, { 11,  3,  4,  5,  9, 10 } }, //Tripod 12 steps
  { 5,  3,  4,  1, 10, 16, 60, { 14,  4,  5,  6, 12, 13 } }, //Tripod 16 steps
  { 3,  2,  2,  3, 20, 24, 70, {  1, 21,  5, 13,  9, 17 } }  //Wave 24 steps
};

const uint8_t GaitsLength = sizeof(Gaits)/sizeof(Gaits[0]);

void setup() {
  SSCSerial.begin(SSC_BAUD);

#ifdef DEBUG_MODE
  DBGSerial.begin(DBG_BAUD);
  const char *Build = "Compiled " __DATE__ ", " __TIME__;
  DBGSerial.println(Build);
  DBGSerial.println(F(" _____                       _ "));
  DBGSerial.println(F("|  |  |___ _ _ ___ ___ ___ _| |"));
  DBGSerial.println(F("|     | -_|_'_| .'| . | . | . |"));
  DBGSerial.println(F("|__|__|___|_,_|__,|  _|___|___|"));
  DBGSerial.println(F("       Lynxmotion |_| Phoenix  "));
  DBGSerial.println();
  DBGSerial.println(F("Press 'Start' to initialize..."));
  DBGSerial.println();
#endif

#ifdef SOUND_MODE
  Sound.begin(BUZZER);
#endif

  //Initialize leg positions
  InitLegPosition();

  //Single leg control
  PrevSelectedLeg = NOT_SELECTED;
  SelectedLeg = NOT_SELECTED;

  //Gait
  LegLiftHeight = 50;
  GaitStep = 1;
  GaitType = 0;
  GaitSelect();

  //Initialize controller
  InitControl();
}

void loop() {
  //Start time
  TimeStart = millis();

  //Read controller
  ReadControl();

  //Single leg control
  SingleLegControl();

  //Gait sequence
  GaitSequence();

  //Balance calculations
  BalanceCalc();

  //Kinematic calculations
  KinematicCalc();

  //Check of mechanical limits
  CheckAngles();

  //Drive servos
  ServoDriver();
}

void InitLegPosition() {
  for (uint8_t LegIndex = 0; LegIndex <= 5; LegIndex++) {
    //All legs to init position
    LegPosX[LegIndex] = (int16_t)pgm_read_word(&InitPosX[LegIndex]);
    LegPosY[LegIndex] = (int16_t)pgm_read_word(&InitPosY[LegIndex]);
    LegPosZ[LegIndex] = (int16_t)pgm_read_word(&InitPosZ[LegIndex]);
  }
}

void SingleLegControl() {
  //Check if all legs are down
  bool AllDown = (LegPosY[RF] == (int16_t)pgm_read_word(&InitPosY[RF])) &&
    (LegPosY[RM] == (int16_t)pgm_read_word(&InitPosY[RM])) &&
    (LegPosY[RR] == (int16_t)pgm_read_word(&InitPosY[RR])) &&
    (LegPosY[LR] == (int16_t)pgm_read_word(&InitPosY[LR])) &&
    (LegPosY[LM] == (int16_t)pgm_read_word(&InitPosY[LM])) &&
    (LegPosY[LF] == (int16_t)pgm_read_word(&InitPosY[LF]));

  if (SelectedLeg <= 5) {
    if (SelectedLeg != PrevSelectedLeg) {
      if (AllDown) {
        //Lift leg a bit when it got selected
        LegPosY[SelectedLeg] = (int16_t)pgm_read_word(&InitPosY[SelectedLeg]) - 20;
        //Store current state
        PrevSelectedLeg = SelectedLeg;
      }
      else {
        //Return previous leg back to the init position
        LegPosX[PrevSelectedLeg] = (int16_t)pgm_read_word(&InitPosX[PrevSelectedLeg]);
        LegPosY[PrevSelectedLeg] = (int16_t)pgm_read_word(&InitPosY[PrevSelectedLeg]);
        LegPosZ[PrevSelectedLeg] = (int16_t)pgm_read_word(&InitPosZ[PrevSelectedLeg]);
      }
    }
    else if (!SLHold) {
      LegPosX[SelectedLeg] = (int16_t)pgm_read_word(&InitPosX[SelectedLeg]) + SLLegX;
      LegPosY[SelectedLeg] = (int16_t)pgm_read_word(&InitPosY[SelectedLeg]) + SLLegY;
      LegPosZ[SelectedLeg] = (int16_t)pgm_read_word(&InitPosZ[SelectedLeg]) + SLLegZ;
    }
  }
  else if (!AllDown) {
    InitLegPosition();
  }
  else if (PrevSelectedLeg != NOT_SELECTED) {
    PrevSelectedLeg = NOT_SELECTED;
  }
}

void Gait(uint8_t LegIndex) {
  //Try to reduce the number of time we look at GaitLegNr and GaitStep
  int16_t LegStep = GaitStep - GaitCurrent.GaitLegNr[LegIndex];

  //Leg middle up position
  if ((GaitInMotion && (GaitCurrent.NrLiftedPos & 1) && (LegStep == 0)) ||
    (!GaitInMotion && (LegStep == 0) && ((abs(GaitPosX[LegIndex]) > 2) ||
    (abs(GaitPosZ[LegIndex]) > 2) || (abs(GaitRotY[LegIndex]) > 2)))) {
    GaitPosX[LegIndex] = 0;
    GaitPosY[LegIndex] = -LegLiftHeight;
    GaitPosZ[LegIndex] = 0;
    GaitRotY[LegIndex] = 0;
  }

  //Optional half height Rear (2, 3, 5 lifted positions)
  else if ((((GaitCurrent.NrLiftedPos == 2) && (LegStep == 0)) ||
    ((GaitCurrent.NrLiftedPos >= 3) && ((LegStep == -1) ||
    (LegStep == (GaitCurrent.StepsInGait - 1))))) && GaitInMotion) {
    GaitPosX[LegIndex] = -TravelLengthX / GaitCurrent.LiftDivFactor;
    GaitPosY[LegIndex] = -3 * LegLiftHeight / (3 + GaitCurrent.HalfLiftHeight);
    GaitPosZ[LegIndex] = -TravelLengthZ / GaitCurrent.LiftDivFactor;
    GaitRotY[LegIndex] = -TravelLengthY / GaitCurrent.LiftDivFactor;
  }

  //Optional half height Front (2, 3, 5 lifted positions)
  else if ((GaitCurrent.NrLiftedPos >= 2) && ((LegStep == 1) ||
    (LegStep == -(GaitCurrent.StepsInGait - 1))) && GaitInMotion) {
    GaitPosX[LegIndex] = TravelLengthX / GaitCurrent.LiftDivFactor;
    GaitPosY[LegIndex] = -3 * LegLiftHeight / (3 + GaitCurrent.HalfLiftHeight);
    GaitPosZ[LegIndex] = TravelLengthZ / GaitCurrent.LiftDivFactor;
    GaitRotY[LegIndex] = TravelLengthY / GaitCurrent.LiftDivFactor;
  }

  //Optional half height Rear (5 lifted positions)
  else if ((GaitCurrent.NrLiftedPos == 5) && (LegStep == -2) && GaitInMotion) {
    GaitPosX[LegIndex] = -TravelLengthX / 2;
    GaitPosY[LegIndex] = -LegLiftHeight / 2;
    GaitPosZ[LegIndex] = -TravelLengthZ / 2;
    GaitRotY[LegIndex] = -TravelLengthY / 2;
  }

  //Optional half height Front (5 lifted positions)
  else if ((GaitCurrent.NrLiftedPos == 5) && ((LegStep == 2) ||
    (LegStep == -(GaitCurrent.StepsInGait - 2))) && GaitInMotion) {
    GaitPosX[LegIndex] = TravelLengthX / 2;
    GaitPosY[LegIndex] = -LegLiftHeight / 2;
    GaitPosZ[LegIndex] = TravelLengthZ / 2;
    GaitRotY[LegIndex] = TravelLengthY / 2;
  }

  //Leg front down position
  else if (((LegStep == GaitCurrent.FrontDownPos) ||
    (LegStep == -(GaitCurrent.StepsInGait - GaitCurrent.FrontDownPos))) && (GaitPosY[LegIndex] < 0)) {
    GaitPosX[LegIndex] = TravelLengthX / 2;
    GaitPosY[LegIndex] = 0;
    GaitPosZ[LegIndex] = TravelLengthZ / 2;
    GaitRotY[LegIndex] = TravelLengthY / 2;
  }

  //Move body forward
  else {
    GaitPosX[LegIndex] = GaitPosX[LegIndex] - (TravelLengthX / GaitCurrent.TLDivFactor);
    GaitPosY[LegIndex] = 0;
    GaitPosZ[LegIndex] = GaitPosZ[LegIndex] - (TravelLengthZ / GaitCurrent.TLDivFactor);
    GaitRotY[LegIndex] = GaitRotY[LegIndex] - (TravelLengthY / GaitCurrent.TLDivFactor);
  }
}

void GaitSelect() {
  if (GaitType < GaitsLength - 1) {
    GaitCurrent = Gaits[GaitType];
  }
}

void GaitSequence() {
  //Check if the gait is in motion
  GaitInMotion = Walking ||
    (abs(TravelLengthX) > TRAVEL_DEADZONE) ||
    (abs(TravelLengthZ) > TRAVEL_DEADZONE) ||
    (abs(TravelLengthY) > TRAVEL_DEADZONE);

  //Clear values under the TRAVEL_DEADZONE
  if (!GaitInMotion) {
    TravelLengthX = 0;
    TravelLengthZ = 0;
    TravelLengthY = 0;
  }

  //Calculate gait sequence
  for (uint8_t LegIndex = 0; LegIndex <= 5; LegIndex++) {
    Gait(LegIndex);
  }

  //Advance to the next step
  if (GaitStep < GaitCurrent.StepsInGait) {
    GaitStep++;
  }
  else {
    GaitStep = 1;
  }
}

void BalanceLeg(int16_t PosX, int16_t PosY, int16_t PosZ, uint8_t LegIndex) {
  //Calculating totals from center of the body to the feet
  int16_t TotalX = (int16_t)pgm_read_word(&OffsetX[LegIndex]) + PosX;
  int16_t TotalY = 150 + PosY; //Using the value 150 to lower the center point of rotation
  int16_t TotalZ = (int16_t)pgm_read_word(&OffsetZ[LegIndex]) + PosZ;

  TotalTransX += TotalX;
  TotalTransY += PosY;
  TotalTransZ += TotalZ;

  TotalBalX += atan2(TotalY, TotalZ) * RAD_IN_DEG - 90; //Rotate balance circle 90 deg
  TotalBalY += atan2(TotalZ, TotalX) * RAD_IN_DEG;
  TotalBalZ += atan2(TotalY, TotalX) * RAD_IN_DEG - 90; //Rotate balance circle 90 deg
}

void BalanceBody() {
  TotalTransX = TotalTransX / 6;
  TotalTransY = TotalTransY / 6;
  TotalTransZ = TotalTransZ / 6;

  //Rotate balance circle by +/- 180 deg
  if (TotalBalY > 0) {
    TotalBalY -= 180;
  }
  else {
    TotalBalY += 180;
  }

  //Compensate for extreme balance positions that causes overflow
  if (TotalBalZ < -180) {
    TotalBalZ += 360;
  }

  //Compensate for extreme balance positions that causes overflow
  if (TotalBalX < -180) {
    TotalBalX += 360;
  }

  //Balance rotation
  TotalBalX = -TotalBalX / 6;
  TotalBalY = -TotalBalY / 6;
  TotalBalZ = TotalBalZ / 6;
}

void BalanceCalc() {
  //Reset values used for calculation of balance
  TotalBalX = 0;
  TotalBalY = 0;
  TotalBalZ = 0;
  TotalTransX = 0;
  TotalTransY = 0;
  TotalTransZ = 0;

  if (BalanceMode) {
    for (uint8_t LegIndex = 0; LegIndex <= 5; LegIndex++) {
      int8_t Sign = sign(LegIndex);
      //Balance calculations for all legs
      BalanceLeg(Sign * LegPosX[LegIndex] + GaitPosX[LegIndex],
        LegPosY[LegIndex] - (int16_t)pgm_read_word(&InitPosY[LegIndex]) + GaitPosY[LegIndex],
        LegPosZ[LegIndex] + GaitPosZ[LegIndex], LegIndex);
    }
    BalanceBody();
  }
}

trig GetSinCos(int16_t AngleDeg) {
  //Get the absolute value of AngleDeg
  int16_t ABSAngleDeg = abs(AngleDeg);

  //Shift rotation to a full circle of 360 deg
  if (AngleDeg < 0) {
    AngleDeg = 360 - (ABSAngleDeg - (360 * (ABSAngleDeg / 360)));
  }
  else {
    AngleDeg = ABSAngleDeg - (360 * (ABSAngleDeg / 360));
  }

  trig Trig;
  if (AngleDeg < 180) {
    //Subtract 90 to shift range
    AngleDeg = AngleDeg - 90;
    Trig.Sin = cos(AngleDeg * DEG_IN_RAD);
    Trig.Cos = -sin(AngleDeg * DEG_IN_RAD);
  }
  else {
    //Subtract 270 to shift range
    AngleDeg = AngleDeg - 270;
    Trig.Sin = -cos(AngleDeg * DEG_IN_RAD);
    Trig.Cos = sin(AngleDeg * DEG_IN_RAD);
  }
  return Trig;
}

void BodyFK(int16_t PosX, int16_t PosY, int16_t PosZ, int16_t RotY, uint8_t LegIndex) {
  //Calculating totals from center of the body to the feet
  int16_t TotalX = (int16_t)pgm_read_word(&OffsetX[LegIndex]) + PosX;
  int16_t TotalY = PosY;
  int16_t TotalZ = (int16_t)pgm_read_word(&OffsetZ[LegIndex]) + PosZ;

  //First calculate sinus and cosinus for each rotation
  trig G = GetSinCos(BodyRotX + TotalBalX);
  trig B = GetSinCos(BodyRotZ + TotalBalZ);
  trig A = GetSinCos(BodyRotY + TotalBalY + RotY);

  //Calculation of rotation matrix
  BodyFKPosX = TotalX - (TotalX * A.Cos * B.Cos - TotalZ * B.Cos * A.Sin + TotalY * B.Sin);
  BodyFKPosY = TotalY - (TotalX * A.Sin * G.Sin - TotalX * A.Cos * G.Cos * B.Sin + TotalZ *
    A.Cos * G.Sin + TotalZ * G.Cos * A.Sin * B.Sin + TotalY * B.Cos * G.Cos);
  BodyFKPosZ = TotalZ - (TotalX * G.Cos * A.Sin + TotalX * A.Cos * B.Sin * G.Sin + TotalZ *
    A.Cos * G.Cos - TotalZ * A.Sin * B.Sin * G.Sin - TotalY * B.Cos * G.Sin);
}

void LegIK(int16_t PosX, int16_t PosY, int16_t PosZ, uint8_t LegIndex) {
  //Length between the coxa and feet
  float PosXZ = sqrt(pow(PosX, 2) + pow(PosZ, 2));

  //Length between shoulder and wrist
  float IKSW = sqrt(pow(PosXZ - CoxaLength, 2) + pow(PosY, 2));

  //Angle of the line SW with respect to the ground in radians
  float IKA1 = atan2(PosXZ - CoxaLength, PosY);

  //Angle of the line SW with respect to the femur in radians
  float IKA2 = acos((pow(FemurLength, 2) - pow(TibiaLength, 2) + pow(IKSW, 2)) / (2 * FemurLength * IKSW));

  CoxaAngle[LegIndex] = atan2(PosZ, PosX) * RAD_IN_DEG + (int16_t)pgm_read_word(&LegAngle[LegIndex]);
  FemurAngle[LegIndex] = -(IKA1 + IKA2) * RAD_IN_DEG + 90;
  TibiaAngle[LegIndex] = -(90 - acos((pow(FemurLength, 2) + pow(TibiaLength, 2) - pow(IKSW, 2)) /
    (2 * FemurLength * TibiaLength)) * RAD_IN_DEG);
}

void KinematicCalc() {
  for (uint8_t LegIndex = 0; LegIndex <= 5; LegIndex++) {
    int8_t Sign = sign(LegIndex);
    //Kinematic calculations for all legs
    BodyFK(Sign * (LegPosX[LegIndex] + BodyPosX) + GaitPosX[LegIndex] - TotalTransX,
      LegPosY[LegIndex] + BodyPosY + GaitPosY[LegIndex] - TotalTransY,
      LegPosZ[LegIndex] + BodyPosZ + GaitPosZ[LegIndex] - TotalTransZ,
      GaitRotY[LegIndex], LegIndex);

    LegIK(LegPosX[LegIndex] + Sign * (BodyPosX - BodyFKPosX + GaitPosX[LegIndex]) - TotalTransX,
      LegPosY[LegIndex] + BodyPosY - BodyFKPosY + GaitPosY[LegIndex] - TotalTransY,
      LegPosZ[LegIndex] + BodyPosZ - BodyFKPosZ + GaitPosZ[LegIndex] - TotalTransZ,
      LegIndex);
  }
}

void CheckAngles() {
  for (uint8_t LegIndex = 0; LegIndex <= 5; LegIndex++) {
    CoxaAngle[LegIndex] = min(max(CoxaAngle[LegIndex],
      (int16_t)pgm_read_word(&CoxaMin[LegIndex])), (int16_t)pgm_read_word(&CoxaMax[LegIndex]));
    FemurAngle[LegIndex] = min(max(FemurAngle[LegIndex],
      (int16_t)pgm_read_word(&FemurMin[LegIndex])), (int16_t)pgm_read_word(&FemurMax[LegIndex]));
    TibiaAngle[LegIndex] = min(max(TibiaAngle[LegIndex],
      (int16_t)pgm_read_word(&TibiaMin[LegIndex])), (int16_t)pgm_read_word(&TibiaMax[LegIndex]));
  }
}

void SSCWrite(uint8_t Command, uint16_t Data) {
  uint8_t Array[3];
  Array[0] = Command;
  Array[1] = Data >> 8;
  Array[2] = Data & 0xFF;
  SSCSerial.write(Array, 3);
}

void ServoDriverUpdate() {
  for (uint8_t LegIndex = 0; LegIndex <= 5; LegIndex++) {
    int8_t Sign = sign(LegIndex);
    //Update all legs
    uint16_t CoxaPWM = (Sign * CoxaAngle[LegIndex] + 90) / 0.0991 + 592;
    uint16_t FemurPWM = (Sign * FemurAngle[LegIndex] + 90) / 0.0991 + 592;
    uint16_t TibiaPWM = (Sign * TibiaAngle[LegIndex] + 90) / 0.0991 + 592;

#ifdef DEBUG_MODE
    if (DebugOutput) {
      DBGSerial.print(LegIndex + 1, DEC);
      DBGSerial.print(F(": "));
      DBGSerial.print(CoxaPWM, DEC);
      DBGSerial.print(F(" "));
      DBGSerial.print(FemurPWM, DEC);
      DBGSerial.print(F(" "));
      DBGSerial.print(TibiaPWM, DEC);
      if (LegIndex != 5) {
        DBGSerial.print(F(" | "));
      }
      else {
        DBGSerial.println();
      }
    }
#endif

    SSCWrite(pgm_read_byte(&CoxaPin[LegIndex]) + 0x80, CoxaPWM);
    SSCWrite(pgm_read_byte(&FemurPin[LegIndex]) + 0x80, FemurPWM);
    SSCWrite(pgm_read_byte(&TibiaPin[LegIndex]) + 0x80, TibiaPWM);
  }
}

void ServoDriverCommit() {
  SSCWrite(0xA1, SSCTime);
}

void ServoDriverFree() {
  for (uint8_t LegIndex = 0; LegIndex <= 31; LegIndex++) {
    SSCWrite(LegIndex + 0x80, 0x00);
  }
  SSCWrite(0xA1, 0xC8);
}

void ServoDriver() {
  if (HexOn) {
    //Update servo positions without committing
    ServoDriverUpdate();

    //Finding any the biggest value for GaitPos/Rot
    for (uint8_t LegIndex = 0; LegIndex <= 5; LegIndex++) {
      if ((GaitPosX[LegIndex] > 2) || (GaitPosX[LegIndex] < -2) ||
          (GaitPosZ[LegIndex] > 2) || (GaitPosZ[LegIndex] < -2) ||
          (GaitRotY[LegIndex] > 2) || (GaitRotY[LegIndex] < -2)) {
        //For making sure that we are using timed move until all legs are down
        ExtraCycle = GaitCurrent.NrLiftedPos + 1;
        break;
      }
    }

    if (ExtraCycle > 0) {
      ExtraCycle--;
      Walking = !(ExtraCycle == 0);

#ifdef DEBUG_MODE
      if (Walking && !PrevWalking) {
        DBGSerial.println(F("Walking: Start"));
        PrevWalking = true;
      }
      else if (!Walking) {
        DBGSerial.println(F("Walking: Finish"));
        PrevWalking = false;
      }
#endif

      //Get end time and calculate wait time
      uint8_t TimeCycle = (millis() - TimeStart);

      //Wait for previous commands to be completed while walking
      delay(PrevSSCTime - TimeCycle);
    }

    //Commit servo positions
    ServoDriverCommit();
  }
  else if (PrevHexOn) {
    //Free servo positions
    ServoDriverFree();
  }

  //Store current state
  PrevSSCTime = SSCTime;
  PrevHexOn = HexOn;
}
