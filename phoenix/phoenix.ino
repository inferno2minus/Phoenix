/**
 * Project Lynxmotion Phoenix
 * Description: Phoenix software
 * Version: v2.0
 * Programmer: Jeroen Janssen (aka Xan)
 *             Kompanets Konstantin (aka I2M)
 *
 * Hardware setup: Arduino, SSC32 V2
 */

#include <Arduino.h>
#include <MiniTone.h>
#include <SoftwareSerial.h>
#include "phoenix_cfg.h"
#include "phoenix.h"

//NrLiftedPos    — Number of positions that a single leg is lifted (1-3)
//FrontDownPos   — Where the leg should be put down to ground
//LiftDivFactor  — Default: 2, when NrLiftedPos = 5: 4
//HalfLiftHeight — How high to lift at halfway up
//TLDivFactor    — Number of steps that a leg is on the floor while walking
//StepsInGait    — Number of steps in gait
//NomGaitSpeed   — Nominal speed of the gait
//GaitLegNr[6]   — Init position of the leg

const gait Gaits[] = { 
  { 3,  2,  2,  3,  8, 12, 70, {  1,  3,  5,  7,  9, 11 } }, //Ripple 12 steps
  { 2,  1,  2,  1,  4,  6, 60, {  4,  1,  1,  1,  4,  4 } }, //Tripod 6 steps
  { 3,  2,  2,  3,  4,  8, 70, {  5,  1,  1,  1,  5,  5 } }, //Tripod 8 steps
  { 3,  2,  2,  3,  8, 12, 60, { 11,  3,  4,  5,  9, 10 } }, //Tripod 12 steps
  { 5,  3,  4,  1, 10, 16, 60, { 14,  4,  5,  6, 12, 13 } }, //Tripod 16 steps
  { 3,  2,  2,  3, 20, 24, 70, {  1, 21,  5, 13,  9, 17 } }  //Wave 24 steps
};

const byte GaitsNumber = sizeof(Gaits)/sizeof(Gaits[0]) - 1;

void setup() {
  SSCSerial.begin(SSC_BAUD);

#ifdef DEBUG_MODE
  DBGSerial.begin(DBG_BAUD);
  DBGSerial.println(" _____                       _ ");
  DBGSerial.println("|  |  |___ _ _ ___ ___ ___ _| |");
  DBGSerial.println("|     | -_|_'_| .'| . | . | . |");
  DBGSerial.println("|__|__|___|_,_|__,|  _|___|___|");
  DBGSerial.println("       Lynxmotion |_| Phoenix  ");
  DBGSerial.println();
  DBGSerial.println("Press 'Start' to initialize...");
  DBGSerial.println();
#endif

#ifdef SOUND_MODE
  Sound.begin(BUZZER);
#endif

  //Setup init positions
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

  //Read controller
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
        LegPosY[LegIndex] - (short)pgm_read_word(&InitPosY[LegIndex]) + GaitPosY[LegIndex],
        LegPosZ[LegIndex] + GaitPosZ[LegIndex], LegIndex);
      }
      else {
        //Balance calculations for all left legs
        BalanceLeg(LegPosX[LegIndex] + GaitPosX[LegIndex],
        LegPosY[LegIndex] - (short)pgm_read_word(&InitPosY[LegIndex]) + GaitPosY[LegIndex],
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

      LegIK(LegPosX[LegIndex] - BodyPosX + BodyFKPosX - GaitPosX[LegIndex] - TotalTransX,
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
      Sound.play(3, 1661, 60, 2217, 80, 2794, 100);
#endif
    }

    //Set SSC time
    if ((abs(TravelLengthX) > TRAVEL_DEADZONE) || (abs(TravelLengthZ) > TRAVEL_DEADZONE) || (abs(TravelLengthY * 2) > TRAVEL_DEADZONE)) {
      SSCTime = GaitCurrent.NomGaitSpeed + (InputTimeDelay * 2) + SpeedControl;

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
        ExtraCycle = GaitCurrent.NrLiftedPos + 1; //For making sure that we are using timed move until all legs are down
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
    Sound.play(3, 2794, 100, 2217, 80, 1661, 60);
#endif
    delay(600);
  }
  else {
    ServoDriverFree();
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
  if (GaitType < GaitsNumber) { 
    GaitCurrent = Gaits[GaitType];
  }
}

void GaitSequence() {
  //Check if the gait is in motion
  TravelRequest = (abs(TravelLengthX) > TRAVEL_DEADZONE) || (abs(TravelLengthZ) > TRAVEL_DEADZONE) || (abs(TravelLengthY) > TRAVEL_DEADZONE) || Walking;

  //Clear values under the TRAVEL_DEADZONE
  if (!TravelRequest) {
    TravelLengthX = 0;
    TravelLengthZ = 0;
    TravelLengthY = 0;
  }

  //Calculate gait sequence
  for (byte LegIndex = 0; LegIndex <= 5; LegIndex++) {
    Gait(LegIndex);
  }

  //Advance to the next step
  GaitStep++;
  if (GaitStep > GaitCurrent.StepsInGait) {
    GaitStep = 1;
  }
}

void Gait(byte LegIndex) {
  //Try to reduce the number of time we look at GaitLegNr and GaitStep
  short LegStep = GaitStep - GaitCurrent.GaitLegNr[LegIndex];

  //Leg middle up position
  if ((TravelRequest && (GaitCurrent.NrLiftedPos & 1) && (LegStep == 0)) || (!TravelRequest && (LegStep == 0) &&
    ((abs(GaitPosX[LegIndex]) > 2) || (abs(GaitPosZ[LegIndex]) > 2) || (abs(GaitRotY[LegIndex]) > 2)))) {
    GaitPosX[LegIndex] = 0;
    GaitPosY[LegIndex] = -LegLiftHeight;
    GaitPosZ[LegIndex] = 0;
    GaitRotY[LegIndex] = 0;
  }

  //Optional half height Rear (2, 3, 5 lifted positions)
  else if ((((GaitCurrent.NrLiftedPos == 2) && (LegStep == 0)) || ((GaitCurrent.NrLiftedPos >= 3) && ((LegStep == -1) || (LegStep == (GaitCurrent.StepsInGait - 1))))) && TravelRequest) {
    GaitPosX[LegIndex] = -TravelLengthX / GaitCurrent.LiftDivFactor;
    GaitPosY[LegIndex] = -3 * LegLiftHeight / (3 + GaitCurrent.HalfLiftHeight); //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[LegIndex] = -TravelLengthZ / GaitCurrent.LiftDivFactor;
    GaitRotY[LegIndex] = -TravelLengthY / GaitCurrent.LiftDivFactor;
  }

  //Optional half height Front (2, 3, 5 lifted positions)
  else if ((GaitCurrent.NrLiftedPos >= 2) && ((LegStep == 1) || (LegStep == -(GaitCurrent.StepsInGait - 1))) && TravelRequest) {
    GaitPosX[LegIndex] = TravelLengthX / GaitCurrent.LiftDivFactor;
    GaitPosY[LegIndex] = -3 * LegLiftHeight / (3 + GaitCurrent.HalfLiftHeight); //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[LegIndex] = TravelLengthZ / GaitCurrent.LiftDivFactor;
    GaitRotY[LegIndex] = TravelLengthY / GaitCurrent.LiftDivFactor;
  }

  //Optional half height Rear (5 lifted positions)
  else if ((GaitCurrent.NrLiftedPos == 5) && (LegStep == -2) && TravelRequest) {
    GaitPosX[LegIndex] = -TravelLengthX / 2;
    GaitPosY[LegIndex] = -LegLiftHeight / 2;
    GaitPosZ[LegIndex] = -TravelLengthZ / 2;
    GaitRotY[LegIndex] = -TravelLengthY / 2;
  }

  //Optional half height Front (5 lifted positions)
  else if ((GaitCurrent.NrLiftedPos == 5) && ((LegStep == 2) || (LegStep == -(GaitCurrent.StepsInGait - 2))) && TravelRequest) {
    GaitPosX[LegIndex] = TravelLengthX / 2;
    GaitPosY[LegIndex] = -LegLiftHeight / 2;
    GaitPosZ[LegIndex] = TravelLengthZ / 2;
    GaitRotY[LegIndex] = TravelLengthY / 2;
  }

  //Leg front down position
  else if (((LegStep == GaitCurrent.FrontDownPos) || (LegStep == -(GaitCurrent.StepsInGait - GaitCurrent.FrontDownPos))) && (GaitPosY[LegIndex] < 0)) {
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

angle GetSinCos(short AngleDeg) {
  //Get the absolute value of AngleDeg
  angle Angle;
  short ABSAngleDeg = abs(AngleDeg);

  //Shift rotation to a full circle of 360 deg
  if (AngleDeg < 0) { //Negative values
    AngleDeg = 360 - (ABSAngleDeg - (360 * (ABSAngleDeg / 360)));
  }
  else { //Positive values
    AngleDeg = ABSAngleDeg - (360 * (ABSAngleDeg / 360));
  }

  if (AngleDeg < 180) { //Angle between 0 and 180
    //Subtract 90 to shift range
    AngleDeg = AngleDeg - 90;
    Angle.Sin = cos(radians(AngleDeg));
    Angle.Cos = -sin(radians(AngleDeg));
  }
  else { //Angle between 180 and 360
    //Subtract 270 to shift range
    AngleDeg = AngleDeg - 270;
    Angle.Sin = -cos(radians(AngleDeg));
    Angle.Cos = sin(radians(AngleDeg));
  }
  return Angle;
}

void BodyFK(short PosX, short PosY, short PosZ, short RotY, byte LegIndex) {
  //Calculating totals from center of the body to the feet
  short TotalX = (short)pgm_read_word(&OffsetX[LegIndex]) + PosX;
  short TotalY = PosY;
  short TotalZ = (short)pgm_read_word(&OffsetZ[LegIndex]) + PosZ;

  //First calculate sinus and cosinus for each rotation
  angle G = GetSinCos(BodyRotX + TotalBalX);
  angle B = GetSinCos(BodyRotZ + TotalBalZ);
  angle A = GetSinCos(BodyRotY + TotalBalY + RotY);

  //Calculation of rotation matrix
  BodyFKPosX = TotalX - (TotalX * A.Cos * B.Cos - TotalZ * B.Cos * A.Sin + TotalY * B.Sin);
  BodyFKPosY = TotalY - (TotalX * A.Sin * G.Sin - TotalX * A.Cos * G.Cos * B.Sin + TotalZ * A.Cos * G.Sin + TotalZ * G.Cos * A.Sin * B.Sin + TotalY * B.Cos * G.Cos);
  BodyFKPosZ = TotalZ - (TotalX * G.Cos * A.Sin + TotalX * A.Cos * B.Sin * G.Sin + TotalZ * A.Cos * G.Cos - TotalZ * A.Sin * B.Sin * G.Sin - TotalY * B.Cos * G.Sin);
}

void LegIK(short PosX, short PosY, short PosZ, byte LegIndex) {
  //Length between the coxa and feet
  float PosXZ = sqrt(pow(PosX, 2) + pow(PosZ, 2));

  //Length between shoulder and wrist
  float IKSW = sqrt(pow(PosXZ - CoxaLength, 2) + pow(PosY, 2));

  //Angle of the line SW with respect to the ground in radians
  float IKA1 = atan2(PosXZ - CoxaLength, PosY);

  //Angle of the line SW with respect to the femur in radians
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

void ServoDriverFree() {
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
