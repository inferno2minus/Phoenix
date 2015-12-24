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
#define WALKMODE         0
#define TRANSLATEMODE    1
#define ROTATEMODE       2
#define SINGLELEGMODE    3

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
  case 1:
    Sound.play(3, 1568, 60, 2093, 80, 2794, 100);
    break;
  case 2:
    Sound.play(3, 2794, 100, 2093, 80, 1568, 60);
    break;
  case 3:
    Sound.play(2093, 40);
    break;
  case 4:
    Sound.play(1568, 80);
    break;
  }
#endif
}

void InitControl() {
  PS2.ConfigGamepad(PS2_DAT, PS2_CMD, PS2_ATT, PS2_CLK);
}

void TurnRobotOn() {
  SoundEvent(1);
#ifdef DEBUG_MODE
  DBGSerial.printf(F("Power: Turn on\n"));
#endif
  HexOn = true;
}

void TurnRobotOff() {
  SoundEvent(2);
#ifdef DEBUG_MODE
  DBGSerial.printf(F("Power: Turn off\n"));
#endif
  HexOn = false;
  SLHold = false;
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
        SoundEvent(3);
        if (ControlMode != TRANSLATEMODE) {
          ControlMode = TRANSLATEMODE;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("ControlMode: TRANSLATEMODE\n"));
#endif
        }
        else if (SelectedLeg == NOT_SELECTED) {
          ControlMode = WALKMODE;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("ControlMode: WALKMODE\n"));
#endif
        }
        else {
          ControlMode = SINGLELEGMODE;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("ControlMode: SINGLELEGMODE\n"));
#endif
        }
      }

      //Rotate mode
      if (PS2.ButtonPressed(PSB_L2) && !GaitInMotion) { //L2 button
        SoundEvent(3);
        if (ControlMode != ROTATEMODE) {
          ControlMode = ROTATEMODE;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("ControlMode: ROTATEMODE\n"));
#endif
        }
        else if (SelectedLeg == NOT_SELECTED) {
          ControlMode = WALKMODE;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("ControlMode: WALKMODE\n"));
#endif
        }
        else {
          ControlMode = SINGLELEGMODE;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("ControlMode: SINGLELEGMODE\n"));
#endif
        }
      }

#ifdef DEBUG_MODE
      if (PS2.ButtonPressed(PSB_L3) && !GaitInMotion) { //L3 button
        DebugOutput = !DebugOutput;
        if (DebugOutput) {
          SoundEvent(3);
        }
        else {
          SoundEvent(4);
        }
      }
#endif

      //Single leg mode
      if (PS2.ButtonPressed(PSB_CIRCLE) && !GaitInMotion) { //Circle button
        SoundEvent(3);
        if (ControlMode != SINGLELEGMODE) {
          ControlMode = SINGLELEGMODE;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("ControlMode: SINGLELEGMODE\n"));
#endif
          if (SelectedLeg == NOT_SELECTED) {
            SelectedLeg = RR; //Start leg
          }
        }
        else {
          ControlMode = WALKMODE;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("ControlMode: WALKMODE\n"));
#endif
          SelectedLeg = NOT_SELECTED;
        }
      }

      //[Common functions]
      //Switch balance mode on/off
      if (PS2.ButtonPressed(PSB_SQUARE) && !GaitInMotion) { //Square button
        BalanceMode = !BalanceMode;
        if (BalanceMode) {
          SoundEvent(3);
#ifdef DEBUG_MODE
          DBGSerial.printf(F("BalanceMode: On\n"));
#endif
        }
        else {
          SoundEvent(4);
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
          SoundEvent(3);
          SpeedControl += 50;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("SpeedControl: %d\n"), SpeedControl);
#endif
        }
      }

      //Speed up
      if (PS2.ButtonPressed(PSB_PAD_LEFT)) { //D-Left button
        if (SpeedControl > 0) {
          SoundEvent(3);
          SpeedControl -= 50;
#ifdef DEBUG_MODE
          DBGSerial.printf(F("SpeedControl: %d\n"), SpeedControl);
#endif
        }
      }

      //[Walk functions]
      if (ControlMode == WALKMODE) {
        //Switch gates
        if (PS2.ButtonPressed(PSB_SELECT) && !GaitInMotion) { //Select button
          if (GaitType < GaitsLength - 1) {
            SoundEvent(3);
            GaitType++;
          }
          else {
            SoundEvent(4);
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
          GaitSelect();
        }

        //Double leg lift height
        if (PS2.ButtonPressed(PSB_R1) && !GaitInMotion) { //R1 button
          DoubleHeight = !DoubleHeight;
          if (DoubleHeight) {
            SoundEvent(3);
            LegLiftHeight = 80;
#ifdef DEBUG_MODE
            DBGSerial.printf(F("DoubleHeight: On\n"));
#endif
          }
          else {
            SoundEvent(4);
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
            SoundEvent(3);
#ifdef DEBUG_MODE
            DBGSerial.printf(F("DoubleTravel: On\n"));
#endif
          }
          else {
            SoundEvent(4);
#ifdef DEBUG_MODE
            DBGSerial.printf(F("DoubleTravel: Off\n"));
#endif
          }
        }

        //Switch between Walk method 1 and Walk method 2
        if (PS2.ButtonPressed(PSB_R3) && !GaitInMotion) { //R3 button
          WalkMethod = !WalkMethod;
          if (WalkMethod) {
            SoundEvent(3);
#ifdef DEBUG_MODE
            DBGSerial.printf(F("WalkMethod: 1\n"));
#endif
          }
          else {
            SoundEvent(4);
#ifdef DEBUG_MODE
            DBGSerial.printf(F("WalkMethod: 2\n"));
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
      else if (ControlMode == TRANSLATEMODE) {
        BodyPosX = (PS2.Analog(PSS_LX) - 128) / 2;
        BodyPosZ = -(PS2.Analog(PSS_LY) - 128) / 3;
        BodyRotY = (PS2.Analog(PSS_RX) - 128) / 6;
        BodyYShift = -(PS2.Analog(PSS_RY) - 128) / 2;
      }

      //[Rotate functions]
      else if (ControlMode == ROTATEMODE) {
        BodyRotX = (PS2.Analog(PSS_LY) - 128) / 8;
        BodyRotY = (PS2.Analog(PSS_RX) - 128) / 6;
        BodyRotZ = (PS2.Analog(PSS_LX) - 128) / 8;
        BodyYShift = -(PS2.Analog(PSS_RY) - 128) / 2;
      }

      //[Single leg functions]
      else if (ControlMode == SINGLELEGMODE) {
        //Switch leg for single leg control
        if (PS2.ButtonPressed(PSB_SELECT)) { //Select button
          if (SelectedLeg < 5) {
            SoundEvent(3);
            SelectedLeg++;
          }
          else {
            SoundEvent(4);
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
          SLHold = !SLHold;
          if (SLHold) {
            SoundEvent(3);
#ifdef DEBUG_MODE
            DBGSerial.printf(F("SLHold: On\n"));
#endif
          }
          else {
            SoundEvent(4);
#ifdef DEBUG_MODE
            DBGSerial.printf(F("SLHold: Off\n"));
#endif
          }
        }

        SLLegX = (PS2.Analog(PSS_LX) - 128) / 2;
        SLLegY = (PS2.Analog(PSS_RY) - 128) / 10;
        SLLegZ = (PS2.Analog(PSS_LY) - 128) / 2;
      }

      //Reset BodyYShift
      if (ControlMode != TRANSLATEMODE && ControlMode != ROTATEMODE) {
        BodyYShift = 0;
      }

      //Calculate BodyPosY
      BodyPosY = min(max(BodyYOffset + BodyYShift, 0), 100);

      //Calculate walking time delay
      InputTimeDelay = 128 - max(max(abs(PS2.Analog(PSS_LX) - 128),
        abs(PS2.Analog(PSS_LY) - 128)), abs(PS2.Analog(PSS_RX) - 128));

      //Calculate servo move time
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
