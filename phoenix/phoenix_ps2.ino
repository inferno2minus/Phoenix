/**
 * Project Lynxmotion Phoenix
 * Description: Phoenix control file
 * Version: v2.0
 * Programmer: Jeroen Janssen (aka Xan)
 *             Kompanets Konstantin (aka I2M)
 *
 * Hardware setup: Playstation 2 gamepad
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
  switch(SoundType) {
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
  PS2.config_gamepad(PS2_DAT, PS2_CMD, PS2_ATT, PS2_CLK);
}

void TurnRobotOn() {
  SoundEvent(1);
#ifdef DEBUG_MODE
  DBGSerial.println(F("Power: Turn on"));
#endif
  HexOn = true;
}

void TurnRobotOff() {
  SoundEvent(2);
#ifdef DEBUG_MODE
  DBGSerial.println(F("Power: Turn off"));
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
  if (PS2.read_gamepad()) {
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
          DBGSerial.println(F("ControlMode: TRANSLATEMODE"));
#endif
        }
        else if (SelectedLeg == NOT_SELECTED) {
          ControlMode = WALKMODE;
#ifdef DEBUG_MODE
          DBGSerial.println(F("ControlMode: WALKMODE"));
#endif
        }
        else {
          ControlMode = SINGLELEGMODE;
#ifdef DEBUG_MODE
          DBGSerial.println(F("ControlMode: SINGLELEGMODE"));
#endif
        }
      }

      //Rotate mode
      if (PS2.ButtonPressed(PSB_L2) && !GaitInMotion) { //L2 button
        SoundEvent(3);
        if (ControlMode != ROTATEMODE) {
          ControlMode = ROTATEMODE;
#ifdef DEBUG_MODE
          DBGSerial.println(F("ControlMode: ROTATEMODE"));
#endif
        }
        else if (SelectedLeg == NOT_SELECTED) {
          ControlMode = WALKMODE;
#ifdef DEBUG_MODE
          DBGSerial.println(F("ControlMode: WALKMODE"));
#endif
        }
        else {
          ControlMode = SINGLELEGMODE;
#ifdef DEBUG_MODE
          DBGSerial.println(F("ControlMode: SINGLELEGMODE"));
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
          DBGSerial.println(F("ControlMode: SINGLELEGMODE"));
#endif
          if (SelectedLeg == NOT_SELECTED) {
            SelectedLeg = RR; //Start leg
          }
        }
        else {
          ControlMode = WALKMODE;
#ifdef DEBUG_MODE
          DBGSerial.println(F("ControlMode: WALKMODE"));
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
          DBGSerial.println(F("BalanceMode: On"));
#endif
        }
        else {
          SoundEvent(4);
#ifdef DEBUG_MODE
          DBGSerial.println(F("BalanceMode: Off"));
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
        DBGSerial.print(F("BodyYOffset: "));
        DBGSerial.println(BodyYOffset, DEC);
#endif
      }

      //Stand up
      if (PS2.ButtonPressed(PSB_PAD_UP)) { //D-Up button
        if (BodyYOffset < 100) {
          BodyYOffset += 10;
#ifdef DEBUG_MODE
          DBGSerial.print(F("BodyYOffset: "));
          DBGSerial.println(BodyYOffset, DEC);
#endif
        }
      }

      //Sit down
      if (PS2.ButtonPressed(PSB_PAD_DOWN)) { //D-Down button
        if (BodyYOffset > 0) {
          BodyYOffset -= 10;
#ifdef DEBUG_MODE
          DBGSerial.print(F("BodyYOffset: "));
          DBGSerial.println(BodyYOffset, DEC);
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
        DBGSerial.print(F("SpeedControl: "));
        DBGSerial.println(SpeedControl, DEC);
#endif
      }

      //Slow down
      if (PS2.ButtonPressed(PSB_PAD_RIGHT)) { //D-Right button
        if (SpeedControl < 1000) {
          SoundEvent(3);
          SpeedControl += 50;
#ifdef DEBUG_MODE
          DBGSerial.print(F("SpeedControl: "));
          DBGSerial.println(SpeedControl, DEC);
#endif
        }
      }

      //Speed up
      if (PS2.ButtonPressed(PSB_PAD_LEFT)) { //D-Left button
        if (SpeedControl > 0) {
          SoundEvent(3);
          SpeedControl -= 50;
#ifdef DEBUG_MODE
          DBGSerial.print(F("SpeedControl: "));
          DBGSerial.println(SpeedControl, DEC);
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
          DBGSerial.print(F("GaitType: "));
          switch(GaitType) {
          case 0:
            DBGSerial.println(F("Ripple 12"));
            break;
          case 1:
            DBGSerial.println(F("Tripod 6"));
            break;
          case 2:
            DBGSerial.println(F("Tripod 8"));
            break;
          case 3:
            DBGSerial.println(F("Tripod 12"));
            break;
          case 4:
            DBGSerial.println(F("Tripod 16"));
            break;
          case 5:
            DBGSerial.println(F("Wave 24"));
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
            DBGSerial.println(F("DoubleHeight: On"));
#endif
          }
          else {
            SoundEvent(4);
            LegLiftHeight = 50;
#ifdef DEBUG_MODE
            DBGSerial.println(F("DoubleHeight: Off"));
#endif
          }
        }

        //Double travel length
        if (PS2.ButtonPressed(PSB_R2) && !GaitInMotion) { //R2 button
          DoubleTravel = !DoubleTravel;
          if (DoubleTravel) {
            SoundEvent(3);
#ifdef DEBUG_MODE
            DBGSerial.println(F("DoubleTravel: On"));
#endif
          }
          else {
            SoundEvent(4);
#ifdef DEBUG_MODE
            DBGSerial.println(F("DoubleTravel: Off"));
#endif
          }
        }

        //Switch between Walk method 1 and Walk method 2
        if (PS2.ButtonPressed(PSB_R3) && !GaitInMotion) { //R3 button
          WalkMethod = !WalkMethod;
          if (WalkMethod) {
            SoundEvent(3);
#ifdef DEBUG_MODE
            DBGSerial.println(F("WalkMethod: 1"));
#endif
          }
          else {
            SoundEvent(4);
#ifdef DEBUG_MODE
            DBGSerial.println(F("WalkMethod: 2"));
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
          DBGSerial.print(F("SelectedLeg: "));
          switch(SelectedLeg) {
          case 0:
            DBGSerial.println(F("RR"));
            break;
          case 1:
            DBGSerial.println(F("RM"));
            break;
          case 2:
            DBGSerial.println(F("RF"));
            break;
          case 3:
            DBGSerial.println(F("LR"));
            break;
          case 4:
            DBGSerial.println(F("LM"));
            break;
          case 5:
            DBGSerial.println(F("LF"));
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
            DBGSerial.println(F("SLHold: On"));
#endif
          }
          else {
            SoundEvent(4);
#ifdef DEBUG_MODE
            DBGSerial.println(F("SLHold: Off"));
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
