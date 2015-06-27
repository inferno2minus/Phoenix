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

void InitControl() {
  PS2.config_gamepad(PS2_DAT, PS2_CMD, PS2_ATT, PS2_CLK);
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
#ifdef SOUND_MODE
        Sound.play(2217, 40);
#endif
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
#ifdef SOUND_MODE
        Sound.play(2217, 40);
#endif
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
#ifdef SOUND_MODE
        Sound.play(2217, 40);
#endif
        DebugOutput = !DebugOutput;
      }
#endif

      //Single leg mode
      if (PS2.ButtonPressed(PSB_CIRCLE) && !GaitInMotion) { //Circle button
#ifdef SOUND_MODE
        Sound.play(2217, 40);
#endif
        if (ControlMode != SINGLELEGMODE) {
          ControlMode = SINGLELEGMODE;
#ifdef DEBUG_MODE
          DBGSerial.println(F("ControlMode: SINGLELEGMODE"));
#endif
          if (SelectedLeg == NOT_SELECTED) {
            SelectedLeg = LR; //Start leg
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
#ifdef SOUND_MODE
        Sound.play(2217, 40);
#endif
        BalanceMode = !BalanceMode;
#ifdef DEBUG_MODE
        if (BalanceMode) {
          DBGSerial.println(F("BalanceMode: On"));
        }
        else {
          DBGSerial.println(F("BalanceMode: Off"));
        }
#endif
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
#ifdef SOUND_MODE
          Sound.play(2217, 40);
#endif
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
#ifdef SOUND_MODE
          Sound.play(2217, 40);
#endif
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
#ifdef SOUND_MODE
            Sound.play(2217, 40);
#endif
            GaitType++;
          }
          else {
#ifdef SOUND_MODE
            Sound.play(2794, 80);
#endif
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
#ifdef SOUND_MODE
          Sound.play(2217, 40);
#endif
          DoubleHeight = !DoubleHeight;
          if (DoubleHeight) {
            LegLiftHeight = 80;
#ifdef DEBUG_MODE
            DBGSerial.println(F("DoubleHeight: On"));
#endif
          }
          else {
            LegLiftHeight = 50;
#ifdef DEBUG_MODE
            DBGSerial.println(F("DoubleHeight: Off"));
#endif
          }
        }

        //Double travel length
        if (PS2.ButtonPressed(PSB_R2) && !GaitInMotion) { //R2 button
#ifdef SOUND_MODE
          Sound.play(2217, 40);
#endif
          DoubleTravel = !DoubleTravel;
#ifdef DEBUG_MODE
          if (DoubleTravel) {
            DBGSerial.println(F("DoubleTravel: On"));
          }
          else {
            DBGSerial.println(F("DoubleTravel: Off"));
          }
#endif
        }

        //Switch between Walk method 1 and Walk method 2
        if (PS2.ButtonPressed(PSB_R3) && !GaitInMotion) { //R3 button
#ifdef SOUND_MODE
          Sound.play(2217, 40);
#endif
          WalkMethod = !WalkMethod;
#ifdef DEBUG_MODE
          if (WalkMethod) {
            DBGSerial.println(F("WalkMethod: 1"));
          }
          else {
            DBGSerial.println(F("WalkMethod: 2"));
          }
#endif
        }

        if (WalkMethod) {
          TravelLengthZ = (PS2.Analog(PSS_RY) - 128);
        }
        else {
          TravelLengthX = -(PS2.Analog(PSS_LX) - 128);
          TravelLengthZ = (PS2.Analog(PSS_LY) - 128);
        }

        if (!DoubleTravel) {
          TravelLengthX /= 2;
          TravelLengthZ /= 2;
        }

        TravelLengthY = -(PS2.Analog(PSS_RX) - 128) / 4;
      }

      //[Translate functions]
      if (ControlMode == TRANSLATEMODE) {
        BodyPosX = (PS2.Analog(PSS_LX) - 128) / 2;
        BodyPosZ = -(PS2.Analog(PSS_LY) - 128) / 3;
        BodyRotY = (PS2.Analog(PSS_RX) - 128) / 6;
        BodyYShift = -(PS2.Analog(PSS_RY) - 128) / 2;
      }

      //[Rotate functions]
      if (ControlMode == ROTATEMODE) {
        BodyRotX = (PS2.Analog(PSS_LY) - 128) / 8;
        BodyRotY = (PS2.Analog(PSS_RX) - 128) / 6;
        BodyRotZ = (PS2.Analog(PSS_LX) - 128) / 8;
        BodyYShift = -(PS2.Analog(PSS_RY) - 128) / 2;
      }

      //[Single leg functions]
      if (ControlMode == SINGLELEGMODE) {
        //Switch leg for single leg control
        if (PS2.ButtonPressed(PSB_SELECT)) { //Select button
#ifdef SOUND_MODE
          Sound.play(2217, 40);
#endif
          if (SelectedLeg < 5) {
            SelectedLeg++;
          }
          else {
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
#ifdef SOUND_MODE
          Sound.play(2217, 40);
#endif
          SLHold = !SLHold;
#ifdef DEBUG_MODE
          if (SLHold) {
            DBGSerial.println(F("SLHold: On"));
          }
          else {
            DBGSerial.println(F("SLHold: Off"));
          }
#endif
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
    }
  }
  else if (HexOn) {
    TurnRobotOff();
  }
}

void TurnRobotOn() {
#ifdef SOUND_MODE
  Sound.play(3, 1661, 60, 2217, 80, 2794, 100);
#endif
#ifdef DEBUG_MODE
  DBGSerial.println(F("Power: Turn on"));
#endif
  HexOn = true;
}

void TurnRobotOff() {
#ifdef SOUND_MODE
  Sound.play(3, 2794, 100, 2217, 80, 1661, 60);
#endif
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
