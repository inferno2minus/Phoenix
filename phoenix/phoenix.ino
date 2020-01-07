/**
 * Project:     Lynxmotion Phoenix
 * Description: Phoenix software
 * Version:     v2.6
 * Author:      Jeroen Janssen (aka Xan)
 *              Kåre Halvorsen (aka Zenta)
 *              Kompanets Konstantin (aka I2M)
 */

#include "phoenix.h"

void setup() {
  Servo.begin(SSC_BAUD);

#ifdef DEBUG_MODE
  DBGSerial.begin(DBG_BAUD);
#endif

#ifdef SOUND_MODE
  Sound.begin(BUZZER);
#endif

  DebugPrint(F("Ver: %s %s %s\n"), VERSION, __DATE__, __TIME__);
  DebugPrint(F(" _____                       _ \n"));
  DebugPrint(F("|  |  |___ _ _ ___ ___ ___ _| |\n"));
  DebugPrint(F("|     | -_|_'_| .'| . | . | . |\n"));
  DebugPrint(F("|__|__|___|_,_|__,|  _|___|___|\n"));
  DebugPrint(F("       Lynxmotion |_| Phoenix  \n\n"));
  DebugPrint(F("Press 'Start' to initialize... \n\n"));

  //Initialize leg positions
  InitLegPosition();

  //Initialize gait
  InitGait();

  //Initialize controller
  InitControl();
}

void loop() {
  //Start time
  StartTime = millis();

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
  for (uint8_t LegIndex = 0; LegIndex < 6; LegIndex++) {
    //All legs to the init position
    LegPos[LegIndex].X = (int16_t)pgm_read_word(&InitPosX[LegIndex]);
    LegPos[LegIndex].Y = (int16_t)pgm_read_word(&InitPosY[LegIndex]);
    LegPos[LegIndex].Z = (int16_t)pgm_read_word(&InitPosZ[LegIndex]);
  }
}

void InitGait() {
  if (GaitType <= GaitsNumber - 1) {
    GaitCurrent = Gaits[GaitType];
  }
}

void SingleLegControl() {
  //Check if all legs are down
  bool AllDown = LegPos[RF].Y == (int16_t)pgm_read_word(&InitPosY[RF]) &&
                 LegPos[RM].Y == (int16_t)pgm_read_word(&InitPosY[RM]) &&
                 LegPos[RR].Y == (int16_t)pgm_read_word(&InitPosY[RR]) &&
                 LegPos[LR].Y == (int16_t)pgm_read_word(&InitPosY[LR]) &&
                 LegPos[LM].Y == (int16_t)pgm_read_word(&InitPosY[LM]) &&
                 LegPos[LF].Y == (int16_t)pgm_read_word(&InitPosY[LF]);

  if (SelectedLeg < 6) {
    if (SelectedLeg != PrevSelectedLeg) {
      if (AllDown) {
        //Lift leg a bit when it got selected
        LegPos[SelectedLeg].Y = (int16_t)pgm_read_word(&InitPosY[SelectedLeg]) - 20;
        //Store previous state
        PrevSelectedLeg = SelectedLeg;
      }
      else {
        //Return previous leg back to the init position
        LegPos[PrevSelectedLeg].X = (int16_t)pgm_read_word(&InitPosX[PrevSelectedLeg]);
        LegPos[PrevSelectedLeg].Y = (int16_t)pgm_read_word(&InitPosY[PrevSelectedLeg]);
        LegPos[PrevSelectedLeg].Z = (int16_t)pgm_read_word(&InitPosZ[PrevSelectedLeg]);
      }
    }
    else if (!SingleLegHold) {
      LegPos[SelectedLeg].X = (int16_t)pgm_read_word(&InitPosX[SelectedLeg]) + SingleLegPos.X;
      LegPos[SelectedLeg].Y = (int16_t)pgm_read_word(&InitPosY[SelectedLeg]) + SingleLegPos.Y;
      LegPos[SelectedLeg].Z = (int16_t)pgm_read_word(&InitPosZ[SelectedLeg]) + SingleLegPos.Z;
    }
  }
  else if (!AllDown) {
    InitLegPosition();
  }
  else if (PrevSelectedLeg != NOT_SELECTED) {
    PrevSelectedLeg = NOT_SELECTED;
  }
}

void GaitCalc(uint8_t LegIndex) {
  int16_t LegStep = GaitStep - GaitCurrent.GaitLegNr[LegIndex];
  uint8_t NrLiftedPos = GaitCurrent.NrLiftedPos;
  uint8_t FrontDownPos = GaitCurrent.FrontDownPos;
  uint8_t LiftDivFactor = GaitCurrent.LiftDivFactor;
  uint8_t HalfLiftHeight = GaitCurrent.HalfLiftHeight;
  uint8_t TLDivFactor = GaitCurrent.TLDivFactor;
  uint8_t StepsInGait = GaitCurrent.StepsInGait;

  //Leg middle up position
  if (TravelRequest && NrLiftedPos & 1 && LegStep == 0 ||
     !TravelRequest && LegStep == 0 && (abs(Gait[LegIndex].Pos.X) > 2 || abs(Gait[LegIndex].Pos.Z) > 2 || abs(Gait[LegIndex].Rot.Y) > 2))
  {
    Gait[LegIndex].Pos.X = 0;
    Gait[LegIndex].Pos.Y = -LegLiftHeight;
    Gait[LegIndex].Pos.Z = 0;
    Gait[LegIndex].Rot.Y = 0;
  }

  //Optional half height rear (2, 3, 5 lifted positions)
  else if ((NrLiftedPos == 2 && LegStep == 0 || NrLiftedPos >= 3 && (LegStep == -1 || LegStep == StepsInGait - 1)) && TravelRequest) {
    Gait[LegIndex].Pos.X = -TravelLength.X / LiftDivFactor;
    Gait[LegIndex].Pos.Y = -3 * LegLiftHeight / (3 + HalfLiftHeight);
    Gait[LegIndex].Pos.Z = -TravelLength.Z / LiftDivFactor;
    Gait[LegIndex].Rot.Y = -TravelLength.Y / LiftDivFactor;
  }

  //Optional half height front (2, 3, 5 lifted positions)
  else if (NrLiftedPos >= 2 && (LegStep == 1 || LegStep == -(StepsInGait - 1)) && TravelRequest) {
    Gait[LegIndex].Pos.X = TravelLength.X / LiftDivFactor;
    Gait[LegIndex].Pos.Y = -3 * LegLiftHeight / (3 + HalfLiftHeight);
    Gait[LegIndex].Pos.Z = TravelLength.Z / LiftDivFactor;
    Gait[LegIndex].Rot.Y = TravelLength.Y / LiftDivFactor;
  }

  //Optional half height rear (5 lifted positions)
  else if (NrLiftedPos == 5 && LegStep == -2 && TravelRequest) {
    Gait[LegIndex].Pos.X = -TravelLength.X / 2;
    Gait[LegIndex].Pos.Y = -LegLiftHeight / 2;
    Gait[LegIndex].Pos.Z = -TravelLength.Z / 2;
    Gait[LegIndex].Rot.Y = -TravelLength.Y / 2;
  }

  //Optional half height front (5 lifted positions)
  else if (NrLiftedPos == 5 && (LegStep == 2 || LegStep == -(StepsInGait - 2)) && TravelRequest) {
    Gait[LegIndex].Pos.X = TravelLength.X / 2;
    Gait[LegIndex].Pos.Y = -LegLiftHeight / 2;
    Gait[LegIndex].Pos.Z = TravelLength.Z / 2;
    Gait[LegIndex].Rot.Y = TravelLength.Y / 2;
  }

  //Leg front down position
  else if ((LegStep == FrontDownPos || LegStep == -(StepsInGait - FrontDownPos)) && Gait[LegIndex].Pos.Y < 0) {
    Gait[LegIndex].Pos.X = TravelLength.X / 2;
    Gait[LegIndex].Pos.Y = 0;
    Gait[LegIndex].Pos.Z = TravelLength.Z / 2;
    Gait[LegIndex].Rot.Y = TravelLength.Y / 2;
  }

  //Move body forward
  else {
    Gait[LegIndex].Pos.X -= TravelLength.X / TLDivFactor;
    Gait[LegIndex].Pos.Y = 0;
    Gait[LegIndex].Pos.Z -= TravelLength.Z / TLDivFactor;
    Gait[LegIndex].Rot.Y -= TravelLength.Y / TLDivFactor;
  }
}

void GaitSequence() {
  //Check if the gait is in motion
  TravelRequest = WalkStatus ||
                 abs(TravelLength.X) > TRAVEL_DEADZONE ||
                 abs(TravelLength.Z) > TRAVEL_DEADZONE ||
                 abs(TravelLength.Y) > TRAVEL_DEADZONE;

  //Clear values under the TRAVEL_DEADZONE
  if (!TravelRequest) {
    TravelLength = {0};
  }

  //Calculate gait sequence
  for (uint8_t LegIndex = 0; LegIndex < 6; LegIndex++) {
    GaitCalc(LegIndex);
  }

  //Advance to the next step
  if (GaitStep < GaitCurrent.StepsInGait) {
    GaitStep++;
  }
  else {
    GaitStep = 1;
  }
}

void BalanceLeg(point3d Pos, uint8_t LegIndex) {
  //Calculating totals from center of the body to the feet
  point3d Total;
  Total.X = (int16_t)pgm_read_word(&OffsetX[LegIndex]) + Pos.X;
  Total.Y = 150 + Pos.Y; //Using the value 150 to lower the center point of rotation
  Total.Z = (int16_t)pgm_read_word(&OffsetZ[LegIndex]) + Pos.Z;

  TotalTranslate.X += Total.X;
  TotalTranslate.Y += Pos.Y;
  TotalTranslate.Z += Total.Z;

  TotalBalance.X += atan2(Total.Y, Total.Z) * RAD_IN_DEG - 90; //Rotate balance circle 90 deg
  TotalBalance.Y += atan2(Total.Z, Total.X) * RAD_IN_DEG;
  TotalBalance.Z += atan2(Total.Y, Total.X) * RAD_IN_DEG - 90; //Rotate balance circle 90 deg
}

void BalanceBody() {
  TotalTranslate.X /= 6;
  TotalTranslate.Y /= 6;
  TotalTranslate.Z /= 6;

  //Rotate balance circle by +/- 180 deg
  if (TotalBalance.Y > 0) {
    TotalBalance.Y -= 180;
  }
  else {
    TotalBalance.Y += 180;
  }

  //Compensate for extreme balance positions that causes overflow
  if (TotalBalance.Z < -180) {
    TotalBalance.Z += 360;
  }

  //Compensate for extreme balance positions that causes overflow
  if (TotalBalance.X < -180) {
    TotalBalance.X += 360;
  }

  //Balance rotation
  TotalBalance.X = -TotalBalance.X / 6;
  TotalBalance.Y = -TotalBalance.Y / 6;
  TotalBalance.Z =  TotalBalance.Z / 6;
}

void BalanceCalc() {
  //Reset values used for calculation of balance
  TotalBalance = {0};
  TotalTranslate = {0};

  if (BalanceMode) {
    for (uint8_t LegIndex = 0; LegIndex < 6; LegIndex++) {
      int8_t Sign = SIGN(LegIndex);
      
      //Balance calculations for all legs
      point3d Position;
      Position.X = Sign * LegPos[LegIndex].X + Gait[LegIndex].Pos.X;
      Position.Y = LegPos[LegIndex].Y - (int16_t)pgm_read_word(&InitPosY[LegIndex]) + Gait[LegIndex].Pos.Y;
      Position.Z = LegPos[LegIndex].Z + Gait[LegIndex].Pos.Z;
      BalanceLeg(Position, LegIndex);
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

void BodyFKCalc(point3d Pos, ordinate Rot, uint8_t LegIndex) {
  //Calculating totals from center of the body to the feet
  point3d Total;
  Total.X = (int16_t)pgm_read_word(&OffsetX[LegIndex]) + Pos.X;
  Total.Y = Pos.Y;
  Total.Z = (int16_t)pgm_read_word(&OffsetZ[LegIndex]) + Pos.Z;

  //First calculate sinus and cosinus for each rotation
  trig G = GetSinCos(BodyRot.X + TotalBalance.X);
  trig B = GetSinCos(BodyRot.Z + TotalBalance.Z);
  trig A = GetSinCos(BodyRot.Y + TotalBalance.Y + Rot.Y);

  //Calculation of rotation matrix
  BodyFKPos.X = Total.X -
               (Total.X * A.Cos * B.Cos -
                Total.Z * B.Cos * A.Sin +
                Total.Y * B.Sin);

  BodyFKPos.Y = Total.Y -
               (Total.X * A.Sin * G.Sin -
                Total.X * A.Cos * G.Cos * B.Sin +
                Total.Z * A.Cos * G.Sin +
                Total.Z * G.Cos * A.Sin * B.Sin +
                Total.Y * B.Cos * G.Cos);

  BodyFKPos.Z = Total.Z -
               (Total.X * G.Cos * A.Sin +
                Total.X * A.Cos * B.Sin * G.Sin +
                Total.Z * A.Cos * G.Cos -
                Total.Z * A.Sin * B.Sin * G.Sin -
                Total.Y * B.Cos * G.Sin);
}

void LegIKCalc(point3d Pos, uint8_t LegIndex) {
  //Length between the coxa and feet
  float PosXZ = sqrt(pow(Pos.X, 2) + pow(Pos.Z, 2));

  //Length between shoulder and wrist
  float IKSW = sqrt(pow(PosXZ - CoxaLength, 2) + pow(Pos.Y, 2));

  //Angle of the line SW with respect to the ground in radians
  float IKA1 = atan2(PosXZ - CoxaLength, Pos.Y);

  //Angle of the line SW with respect to the femur in radians
  float IKA2 = acos((pow(FemurLength, 2) - pow(TibiaLength, 2) + pow(IKSW, 2)) / (2 * FemurLength * IKSW));

  CoxaAngle[LegIndex] = atan2(Pos.Z, Pos.X) * RAD_IN_DEG + (int16_t)pgm_read_word(&LegAngle[LegIndex]);
  FemurAngle[LegIndex] = -(IKA1 + IKA2) * RAD_IN_DEG + 90;
  TibiaAngle[LegIndex] = -(90 - acos((pow(FemurLength, 2) + pow(TibiaLength, 2) - pow(IKSW, 2)) / (2 * FemurLength * TibiaLength)) * RAD_IN_DEG);
}

void KinematicCalc() {
  for (uint8_t LegIndex = 0; LegIndex < 6; LegIndex++) {
    int8_t Sign = SIGN(LegIndex);

    //Kinematic calculations for all legs
    point3d BodyPosition;
    BodyPosition.X = Sign * (LegPos[LegIndex].X + BodyPos.X) + Gait[LegIndex].Pos.X - TotalTranslate.X;
    BodyPosition.Y = LegPos[LegIndex].Y + BodyPos.Y + Gait[LegIndex].Pos.Y - TotalTranslate.Y;
    BodyPosition.Z = LegPos[LegIndex].Z + BodyPos.Z + Gait[LegIndex].Pos.Z - TotalTranslate.Z;

    BodyFKCalc(BodyPosition, Gait[LegIndex].Rot, LegIndex);

    point3d LegPosition;
    LegPosition.X = LegPos[LegIndex].X + Sign * (BodyPos.X - BodyFKPos.X + Gait[LegIndex].Pos.X) - TotalTranslate.X;
    LegPosition.Y = LegPos[LegIndex].Y + BodyPos.Y - BodyFKPos.Y + Gait[LegIndex].Pos.Y - TotalTranslate.Y;
    LegPosition.Z = LegPos[LegIndex].Z + BodyPos.Z - BodyFKPos.Z + Gait[LegIndex].Pos.Z - TotalTranslate.Z;

    LegIKCalc(LegPosition, LegIndex);
  }
}

void CheckAngles() {
  for (uint8_t LegIndex = 0; LegIndex < 6; LegIndex++) {
    CoxaAngle[LegIndex] = min(max(CoxaAngle[LegIndex], (int16_t)pgm_read_word(&CoxaMin[LegIndex])),
      (int16_t)pgm_read_word(&CoxaMax[LegIndex]));
    FemurAngle[LegIndex] = min(max(FemurAngle[LegIndex], (int16_t)pgm_read_word(&FemurMin[LegIndex])),
      (int16_t)pgm_read_word(&FemurMax[LegIndex]));
    TibiaAngle[LegIndex] = min(max(TibiaAngle[LegIndex], (int16_t)pgm_read_word(&TibiaMin[LegIndex])),
      (int16_t)pgm_read_word(&TibiaMax[LegIndex]));
  }
}

void ServoDriverUpdate() {
  for (uint8_t LegIndex = 0; LegIndex < 6; LegIndex++) {
    int8_t Sign = SIGN(LegIndex);
    //Update all legs
    uint16_t CoxaPWM = (Sign * CoxaAngle[LegIndex] + 90) / PWM_FACTOR + PWM_OFFSET;
    uint16_t FemurPWM = (Sign * FemurAngle[LegIndex] + 90) / PWM_FACTOR + PWM_OFFSET;
    uint16_t TibiaPWM = (Sign * TibiaAngle[LegIndex] + 90) / PWM_FACTOR + PWM_OFFSET;

#ifdef DEBUG_MODE
    if (DebugOutput) {
      DebugPrint(F("%d: %04d %04d %04d"), LegIndex + 1, CoxaPWM, FemurPWM, TibiaPWM);
      if (LegIndex != 5) {
        DebugPrint(F(" | "));
      }
      else {
        DebugPrint(F("\n"));
      }
    }
#endif

    Servo.write(pgm_read_byte(&CoxaPin[LegIndex] + 0x80), CoxaPWM);
    Servo.write(pgm_read_byte(&FemurPin[LegIndex] + 0x80), FemurPWM);
    Servo.write(pgm_read_byte(&TibiaPin[LegIndex] + 0x80), TibiaPWM);
  }
}

void ServoDriver() {
  if (HexOn) {
    //Update servo positions without committing
    ServoDriverUpdate();

    //Finding any the biggest value for GaitPos/Rot
    for (uint8_t LegIndex = 0; LegIndex < 6; LegIndex++) {
      if (Gait[LegIndex].Pos.X > 2 || Gait[LegIndex].Pos.X < -2 ||
          Gait[LegIndex].Pos.Z > 2 || Gait[LegIndex].Pos.Z < -2 ||
          Gait[LegIndex].Rot.Y > 2 || Gait[LegIndex].Rot.Y < -2) {

        //For making sure that we are using timed move until all legs are down
        ExtraCycle = GaitCurrent.NrLiftedPos + 1;
        break;
      }
    }

    if (ExtraCycle > 0) {
      ExtraCycle--;
      WalkStatus = !(ExtraCycle == 0);

#ifdef DEBUG_MODE
      if (WalkStatus && !PrevWalkStatus) {
        DebugPrint(F("WalkStatus: Start\n"));
        PrevWalkStatus = true;
      }
      else if (!WalkStatus) {
        DebugPrint(F("WalkStatus: Stop\n"));
        PrevWalkStatus = false;
      }
#endif

      //Calculate cycle time
      uint8_t CycleTime = millis() - StartTime;

      //Wait for previous commands to be completed while walking
      delay(abs(PrevSSCTime - CycleTime));
    }

    //Commit servo positions
    Servo.commit(SSCTime);
  }
  else if (PrevHexOn) {
    //Turn off the hexapod
    ServoDriverUpdate();
    Servo.commit(600);
    delay(600);

    //Free servo positions
    Servo.free();
  }

  //Store previous state
  PrevSSCTime = SSCTime;
  PrevHexOn = HexOn;
}
