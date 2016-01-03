/**
 * Project:     Lynxmotion Phoenix
 * Description: Phoenix control file
 * Version:     v2.5
 * Author:      Jeroen Janssen (aka Xan)
 *              Kåre Halvorsen (aka Zenta)
 *              Kompanets Konstantin (aka I2M)
 */

#include <PS2X.h>

//Control mode
#define WALK_MODE         0
#define ROTATE_MODE       1
#define TRANSLATE_MODE    2
#define SINGLELEG_MODE    3

PS2X    PS2;
bool    DoubleHeight;
bool    DoubleTravel;
bool    WalkMethod;
uint8_t ControlMode;
int16_t BodyYOffset;
int16_t BodyYShift;

void SoundEvent(uint8_t SoundType) {
#ifdef SOUND_MODE
  switch (SoundType) {
  case 0:
    Sound.play(3, 1568, 60, 2093, 80, 2794, 100);
    break;
  case 1:
    Sound.play(3, 2794, 100, 2093, 80, 1568, 60);
    break;
  case 2:
    Sound.play(2093, 40);
    break;
  case 3:
    Sound.play(1568, 80);
    break;
  }
#endif
}

void InitControl() {
  PS2.ConfigGamepad(PS2_DAT, PS2_CMD, PS2_ATT, PS2_CLK);
}

void TurnRobotOn() {
  SoundEvent(0);
#ifdef DEBUG_MODE
  DBGSerial.printf(F("TurnRobot: On\n"));
#endif
  HexOn = true;
}

void TurnRobotOff() {
  SoundEvent(1);
#ifdef DEBUG_MODE
  DBGSerial.printf(F("TurnRobot: Off\n"));
#endif
  HexOn = false;
  SingleLegHold = false;
  BodyPosX = 0;
  BodyPosY = 0;
  BodyPosZ = 0;
  BodyRotX = 0;
  BodyRotY = 0;
  BodyRotZ = 0;
  BodyYOffset = 0;
  BodyYShift = 0;
}

void ReadControl() {
  if (PS2.ReadGamepad()) {
    //Switch bot on/off
    if (PS2.ButtonPressed(PSB_START) && !GaitInMotion) { //Start button
      if (HexOn) {
        TurnRobotOff();
      }
      else {
        TurnRobotOn();
      }
    }

    if (HexOn) {
      //Translate mode
      if (PS2.ButtonPressed(PSB_L1) && !GaitInMotion) { //L1 button
        SoundEvent(2);
        if (ControlMode != TRANSLATE_MODE) {
          ControlMode = TRANSLATE_MODE;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("ControlMode: Translate\n"));
#endif
        }
        else if (SelectedLeg == NOT_SELECTED) {
          ControlMode = WALK_MODE;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("ControlMode: Walk\n"));
#endif
        }
        else {
          ControlMode = SINGLELEG_MODE;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("ControlMode: SingleLeg\n"));
#endif
        }
      }

      //Rotate mode
      if (PS2.ButtonPressed(PSB_L2) && !GaitInMotion) { //L2 button
        SoundEvent(2);
        if (ControlMode != ROTATE_MODE) {
          ControlMode = ROTATE_MODE;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("ControlMode: Rotate\n"));
#endif
        }
        else if (SelectedLeg == NOT_SELECTED) {
          ControlMode = WALK_MODE;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("ControlMode: Walk\n"));
#endif
        }
        else {
          ControlMode = SINGLELEG_MODE;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("ControlMode: SingleLeg\n"));
#endif
        }
      }

#ifdef DEBUG_MODE
      if (PS2.ButtonPressed(PSB_L3) && !GaitInMotion) { //L3 button
        DebugOutput = !DebugOutput;
        if (DebugOutput) {
          SoundEvent(2);
        }
        else {
          SoundEvent(3);
        }
      }
#endif

      //Single leg mode
      if (PS2.ButtonPressed(PSB_CIRCLE) && !GaitInMotion) { //Circle button
        SoundEvent(2);
        if (ControlMode != SINGLELEG_MODE) {
          ControlMode = SINGLELEG_MODE;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("ControlMode: SingleLeg\n"));
#endif
          if (SelectedLeg == NOT_SELECTED) {
            SelectedLeg = RR; //Start leg
          }
        }
        else {
          ControlMode = WALK_MODE;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("ControlMode: Walk\n"));
#endif
          SelectedLeg = NOT_SELECTED;
        }
      }

      //[Common functions]
      //Switch balance mode on/off
      if (PS2.ButtonPressed(PSB_SQUARE) && !GaitInMotion) { //Square button
        BalanceMode = !BalanceMode;
        if (BalanceMode) {
          SoundEvent(2);
#ifdef DEBUG_MODE
          DBGSerial.printf(F("BalanceMode: On\n"));
#endif
        }
        else {
          SoundEvent(3);
#ifdef DEBUG_MODE
          DBGSerial.printf(F("BalanceMode: Off\n"));
#endif
        }
      }

      //Stand up, sit down
      if (PS2.ButtonPressed(PSB_TRIANGLE)) { //Triangle button
        if (BodyYOffset > 0) {
          BodyYOffset = 0;
        }
        else {
          BodyYOffset = 40;
        }
#ifdef DEBUG_MODE
        DBGSerial.printf(F("BodyYOffset: %d\n"), BodyYOffset);
#endif
      }

      //Stand up
      if (PS2.ButtonPressed(PSB_PAD_UP)) { //D-Up button
        if (BodyYOffset < 100) {
          BodyYOffset += 10;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("BodyYOffset: %d\n"), BodyYOffset);
#endif
        }
      }

      //Sit down
      if (PS2.ButtonPressed(PSB_PAD_DOWN)) { //D-Down button
        if (BodyYOffset > 0) {
          BodyYOffset -= 10;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("BodyYOffset: %d\n"), BodyYOffset);
#endif
        }
      }

      //Speed up, previous speed
      if (PS2.ButtonPressed(PSB_CROSS)) { //Cross button
        if (SpeedControl > 0) {
          PrevSpeedControl = SpeedControl;
          SpeedControl = 0;
        }
        else {
          SpeedControl = PrevSpeedControl;
        }
#ifdef DEBUG_MODE
        DBGSerial.printf(F("SpeedControl: %d\n"), SpeedControl);
#endif
      }

      //Slow down
      if (PS2.ButtonPressed(PSB_PAD_RIGHT)) { //D-Right button
        if (SpeedControl < 1000) {
          SoundEvent(2);
          SpeedControl += 50;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("SpeedControl: %d\n"), SpeedControl);
#endif
        }
      }

      //Speed up
      if (PS2.ButtonPressed(PSB_PAD_LEFT)) { //D-Left button
        if (SpeedControl > 0) {
          SoundEvent(2);
          SpeedControl -= 50;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("SpeedControl: %d\n"), SpeedControl);
#endif
        }
      }

      //[Walk functions]
      if (ControlMode == WALK_MODE) {
        //Switch gates
        if (PS2.ButtonPressed(PSB_SELECT) && !GaitInMotion) { //Select button
          if (GaitType < GaitsLength - 1) {
            SoundEvent(2);
            GaitType++;
          }
          else {
            SoundEvent(3);
            GaitType = 0;
          }
#ifdef DEBUG_MODE
          DBGSerial.printf(F("GaitType: "));
          switch (GaitType) {
          case 0:
            DBGSerial.printf(F("Ripple 12\n"));
            break;
          case 1:
            DBGSerial.printf(F("Tripod 6\n"));
            break;
          case 2:
            DBGSerial.printf(F("Tripod 8\n"));
            break;
          case 3:
            DBGSerial.printf(F("Tripod 12\n"));
            break;
          case 4:
            DBGSerial.printf(F("Tripod 16\n"));
            break;
          case 5:
            DBGSerial.printf(F("Wave 24\n"));
            break;
          }
#endif
          InitGait();
        }

        //Double leg lift height
        if (PS2.ButtonPressed(PSB_R1) && !GaitInMotion) { //R1 button
          DoubleHeight = !DoubleHeight;
          if (DoubleHeight) {
            SoundEvent(2);
            LegLiftHeight = 80;
#ifdef DEBUG_MODE
            DBGSerial.printf(F("DoubleHeight: On\n"));
#endif
          }
          else {
            SoundEvent(3);
            LegLiftHeight = 50;
#ifdef DEBUG_MODE
            DBGSerial.printf(F("DoubleHeight: Off\n"));
#endif
          }
        }

        //Double travel length
        if (PS2.ButtonPressed(PSB_R2) && !GaitInMotion) { //R2 button
          DoubleTravel = !DoubleTravel;
          if (DoubleTravel) {
            SoundEvent(2);
#ifdef DEBUG_MODE
            DBGSerial.printf(F("DoubleTravel: On\n"));
#endif
          }
          else {
            SoundEvent(3);
#ifdef DEBUG_MODE
            DBGSerial.printf(F("DoubleTravel: Off\n"));
#endif
          }
        }

        //Switch between Walk method YZ and Walk method XYZ
        if (PS2.ButtonPressed(PSB_R3) && !GaitInMotion) { //R3 button
          WalkMethod = !WalkMethod;
          if (WalkMethod) {
            SoundEvent(2);
#ifdef DEBUG_MODE
            DBGSerial.printf(F("WalkMethod: YZ\n"));
#endif
          }
          else {
            SoundEvent(3);
#ifdef DEBUG_MODE
            DBGSerial.printf(F("WalkMethod: XYZ\n"));
#endif
          }
        }

        if (WalkMethod) {
          TravelLengthZ = -(PS2.Analog(PSS_RY) - 128);
        }
        else {
          TravelLengthX = (PS2.Analog(PSS_LX) - 128);
          TravelLengthZ = -(PS2.Analog(PSS_LY) - 128);
        }

        if (!DoubleTravel) {
          TravelLengthX /= 2;
          TravelLengthZ /= 2;
        }

        TravelLengthY = -(PS2.Analog(PSS_RX) - 128) / 4;
      }

      //[Translate functions]
      else if (ControlMode == TRANSLATE_MODE) {
        BodyPosX = (PS2.Analog(PSS_LX) - 128) / 2;
        BodyPosZ = -(PS2.Analog(PSS_LY) - 128) / 3;
        BodyRotY = (PS2.Analog(PSS_RX) - 128) / 6;
        BodyYShift = -(PS2.Analog(PSS_RY) - 128) / 2;
      }

      //[Rotate functions]
      else if (ControlMode == ROTATE_MODE) {
        BodyRotX = (PS2.Analog(PSS_LY) - 128) / 8;
        BodyRotY = (PS2.Analog(PSS_RX) - 128) / 6;
        BodyRotZ = (PS2.Analog(PSS_LX) - 128) / 8;
        BodyYShift = -(PS2.Analog(PSS_RY) - 128) / 2;
      }

      //[Single leg functions]
      else if (ControlMode == SINGLELEG_MODE) {
        //Switch leg for single leg control
        if (PS2.ButtonPressed(PSB_SELECT)) { //Select button
          if (SelectedLeg < 5) {
            SoundEvent(2);
            SelectedLeg++;
          }
          else {
            SoundEvent(3);
            SelectedLeg = 0;
          }
#ifdef DEBUG_MODE
          DBGSerial.printf(F("SelectedLeg: "));
          switch (SelectedLeg) {
          case 0:
            DBGSerial.printf(F("RR\n"));
            break;
          case 1:
            DBGSerial.printf(F("RM\n"));
            break;
          case 2:
            DBGSerial.printf(F("RF\n"));
            break;
          case 3:
            DBGSerial.printf(F("LR\n"));
            break;
          case 4:
            DBGSerial.printf(F("LM\n"));
            break;
          case 5:
            DBGSerial.printf(F("LF\n"));
            break;
          }
#endif
        }

        //Hold single leg in place
        if (PS2.ButtonPressed(PSB_R2)) { //R2 button
          SingleLegHold = !SingleLegHold;
          if (SingleLegHold) {
            SoundEvent(2);
#ifdef DEBUG_MODE
            DBGSerial.printf(F("SingleLegHold: On\n"));
#endif
          }
          else {
            SoundEvent(3);
#ifdef DEBUG_MODE
            DBGSerial.printf(F("SingleLegHold: Off\n"));
#endif
          }
        }

        SingleLegX = (PS2.Analog(PSS_LX) - 128) / 2;
        SingleLegY = (PS2.Analog(PSS_RY) - 128) / 10;
        SingleLegZ = (PS2.Analog(PSS_LY) - 128) / 2;
      }

      //Reset BodyYShift
      if (ControlMode != TRANSLATE_MODE && ControlMode != ROTATE_MODE) {
        BodyYShift = 0;
      }

      //Calculate BodyPosY
      BodyPosY = min(max(BodyYOffset + BodyYShift, 0), 100);

      //Calculate InputTimeDelay
      InputTimeDelay = 128 - max(max(abs(PS2.Analog(PSS_LX) - 128),
        abs(PS2.Analog(PSS_LY) - 128)), abs(PS2.Analog(PSS_RX) - 128));

      //Calculate SSCTime
      if ((abs(TravelLengthX) > TRAVEL_DEADZONE) ||
          (abs(TravelLengthZ) > TRAVEL_DEADZONE) ||
          (abs(TravelLengthY) > TRAVEL_DEADZONE / 2)) {
        SSCTime = GaitCurrent.NomGaitSpeed + (InputTimeDelay * 2) + SpeedControl;
        //Add additional delay when balance mode is on
        if (BalanceMode) {
          SSCTime += 100;
        }
      }
      else {
        SSCTime = 200 + SpeedControl;
      }
    }
  }
  else if (HexOn) {
    TurnRobotOff();
  }
}
