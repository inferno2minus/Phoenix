/**
 * Project:     Lynxmotion Phoenix
 * Description: Phoenix control file
 * Version:     v2.6
 * Author:      Jeroen Janssen (aka Xan)
 *              Kåre Halvorsen (aka Zenta)
 *              Kompanets Konstantin (aka I2M)
 */

#include "phoenix_ps2.h"

void InitControl() {
  PS2.ConfigGamepad(PS2_DAT, PS2_CMD, PS2_ATT, PS2_CLK);
}

void SoundEvent(uint8_t SoundType) {
#ifdef SOUND_MODE
  switch (SoundType) {
  case 0:
    Sound.play(3, 1568, 80, 2093, 80, 2794, 80);
    break;
  case 1:
    Sound.play(3, 2794, 80, 2093, 80, 1568, 80);
    break;
  case 2:
    Sound.play(2093, 80);
    break;
  case 3:
    Sound.play(1568, 80);
    break;
  }
#endif
}

void PowerSwitch() {
  HexOn = !HexOn;
  if (HexOn) {
    SoundEvent(0);
    DebugPrint(F("PowerSwitch: On\n"));
  }
  else {
    SoundEvent(1);
    DebugPrint(F("PowerSwitch: Off\n"));
    SingleLegHold = false;
    BodyPosX = 0;
    BodyPosY = 0;
    BodyPosZ = 0;
    BodyRotX = 0;
    BodyRotY = 0;
    BodyRotZ = 0;
    BodyOffsetY = 0;
    BodyShiftY = 0;
  }
}

void ReadControl() {
  if (PS2.ReadGamepad()) {
    //Switch hexapod on/off
    if (PS2.ButtonPressed(PSB_START) && !GaitInMotion) { //Start button
      PowerSwitch();
    }

    if (HexOn) {
      //Translate mode
      if (PS2.ButtonPressed(PSB_L1) && !GaitInMotion) { //L1 button
        SoundEvent(2);
        if (ControlMode != TRANSLATE_MODE) {
          ControlMode = TRANSLATE_MODE;
          DebugPrint(F("ControlMode: Translate\n"));
        }
        else if (SelectedLeg == NOT_SELECTED) {
          ControlMode = WALK_MODE;
          DebugPrint(F("ControlMode: Walk\n"));
        }
        else {
          ControlMode = SINGLELEG_MODE;
          DebugPrint(F("ControlMode: SingleLeg\n"));
        }
      }

      //Rotate mode
      if (PS2.ButtonPressed(PSB_L2) && !GaitInMotion) { //L2 button
        SoundEvent(2);
        if (ControlMode != ROTATE_MODE) {
          ControlMode = ROTATE_MODE;
          DebugPrint(F("ControlMode: Rotate\n"));
        }
        else if (SelectedLeg == NOT_SELECTED) {
          ControlMode = WALK_MODE;
          DebugPrint(F("ControlMode: Walk\n"));
        }
        else {
          ControlMode = SINGLELEG_MODE;
          DebugPrint(F("ControlMode: SingleLeg\n"));
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
          DebugPrint(F("ControlMode: SingleLeg\n"));
          if (SelectedLeg == NOT_SELECTED) {
            SelectedLeg = RR; //Start leg
          }
        }
        else {
          ControlMode = WALK_MODE;
          DebugPrint(F("ControlMode: Walk\n"));
          SelectedLeg = NOT_SELECTED;
        }
      }

      //[Common functions]
      //Switch balance mode on/off
      if (PS2.ButtonPressed(PSB_SQUARE) && !GaitInMotion) { //Square button
        BalanceMode = !BalanceMode;
        if (BalanceMode) {
          SoundEvent(2);
          DebugPrint(F("BalanceMode: On\n"));
        }
        else {
          SoundEvent(3);
          DebugPrint(F("BalanceMode: Off\n"));
        }
      }

      //Stand up, sit down
      if (PS2.ButtonPressed(PSB_TRIANGLE)) { //Triangle button
        if (BodyOffsetY > 0) {
          BodyOffsetY = 0;
        }
        else {
          BodyOffsetY = 40;
        }
        DebugPrint(F("BodyOffsetY: %d\n"), BodyOffsetY);
      }

      //Stand up
      if (PS2.ButtonPressed(PSB_PAD_UP)) { //D-Up button
        if (BodyOffsetY < 100) {
          BodyOffsetY += 10;
          DebugPrint(F("BodyOffsetY: %d\n"), BodyOffsetY);
        }
      }

      //Sit down
      if (PS2.ButtonPressed(PSB_PAD_DOWN)) { //D-Down button
        if (BodyOffsetY > 0) {
          BodyOffsetY -= 10;
          DebugPrint(F("BodyOffsetY: %d\n"), BodyOffsetY);
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
        DebugPrint(F("SpeedControl: %d\n"), SpeedControl);
      }

      //Slow down
      if (PS2.ButtonPressed(PSB_PAD_RIGHT)) { //D-Right button
        if (SpeedControl < 1000) {
          SoundEvent(2);
          SpeedControl += 50;
          DebugPrint(F("SpeedControl: %d\n"), SpeedControl);
        }
      }

      //Speed up
      if (PS2.ButtonPressed(PSB_PAD_LEFT)) { //D-Left button
        if (SpeedControl > 0) {
          SoundEvent(2);
          SpeedControl -= 50;
          DebugPrint(F("SpeedControl: %d\n"), SpeedControl);
        }
      }

      //[Walk functions]
      if (ControlMode == WALK_MODE) {
        //Switch gates
        if (PS2.ButtonPressed(PSB_SELECT) && !GaitInMotion) { //Select button
          if (GaitType < GaitsNumber - 1) {
            SoundEvent(2);
            GaitType++;
          }
          else {
            SoundEvent(3);
            GaitType = 0;
          }
#ifdef DEBUG_MODE
          DebugPrint(F("GaitType: "));
          switch (GaitType) {
          case 0:
            DebugPrint(F("Ripple 12\n"));
            break;
          case 1:
            DebugPrint(F("Tripod 6\n"));
            break;
          case 2:
            DebugPrint(F("Tripod 8\n"));
            break;
          case 3:
            DebugPrint(F("Tripod 12\n"));
            break;
          case 4:
            DebugPrint(F("Tripod 16\n"));
            break;
          case 5:
            DebugPrint(F("Wave 24\n"));
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
            DebugPrint(F("DoubleHeight: On\n"));
          }
          else {
            SoundEvent(3);
            LegLiftHeight = 50;
            DebugPrint(F("DoubleHeight: Off\n"));
          }
        }

        //Double travel length
        if (PS2.ButtonPressed(PSB_R2) && !GaitInMotion) { //R2 button
          DoubleTravel = !DoubleTravel;
          if (DoubleTravel) {
            SoundEvent(2);
            DebugPrint(F("DoubleTravel: On\n"));
          }
          else {
            SoundEvent(3);
            DebugPrint(F("DoubleTravel: Off\n"));
          }
        }

        //Switch between walk method YZ and XYZ
        if (PS2.ButtonPressed(PSB_R3) && !GaitInMotion) { //R3 button
          WalkMethod = !WalkMethod;
          if (WalkMethod) {
            SoundEvent(2);
            DebugPrint(F("WalkMethod: YZ\n"));
          }
          else {
            SoundEvent(3);
            DebugPrint(F("WalkMethod: XYZ\n"));
          }
        }

        if (!WalkMethod) {
          TravelLengthX =  (PS2.Analog(PSS_LX) - 128);
          TravelLengthZ = -(PS2.Analog(PSS_LY) - 128);
        }
        else {
          TravelLengthZ = -(PS2.Analog(PSS_RY) - 128);
        }

        if (!DoubleTravel) {
          TravelLengthX = TravelLengthX / 2;
          TravelLengthZ = TravelLengthZ / 2;
        }

        TravelLengthY = -(PS2.Analog(PSS_RX) - 128) / 4;
      }

      //[Rotate functions]
      else if (ControlMode == ROTATE_MODE) {
        BodyRotX = (PS2.Analog(PSS_LY) - 128) / 8;
        BodyRotY = (PS2.Analog(PSS_RX) - 128) / 6;
        BodyRotZ = (PS2.Analog(PSS_LX) - 128) / 8;
        BodyShiftY = -(PS2.Analog(PSS_RY) - 128) / 2;
      }

      //[Translate functions]
      else if (ControlMode == TRANSLATE_MODE) {
        BodyPosX =  (PS2.Analog(PSS_LX) - 128) / 2;
        BodyPosZ = -(PS2.Analog(PSS_LY) - 128) / 3;
        BodyRotY =  (PS2.Analog(PSS_RX) - 128) / 6;
        BodyShiftY = -(PS2.Analog(PSS_RY) - 128) / 2;
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
          DebugPrint(F("SelectedLeg: "));
          switch (SelectedLeg) {
          case 0:
            DebugPrint(F("RR\n"));
            break;
          case 1:
            DebugPrint(F("RM\n"));
            break;
          case 2:
            DebugPrint(F("RF\n"));
            break;
          case 3:
            DebugPrint(F("LR\n"));
            break;
          case 4:
            DebugPrint(F("LM\n"));
            break;
          case 5:
            DebugPrint(F("LF\n"));
            break;
          }
#endif
        }

        //Hold single leg in place
        if (PS2.ButtonPressed(PSB_R2)) { //R2 button
          SingleLegHold = !SingleLegHold;
          if (SingleLegHold) {
            SoundEvent(2);
            DebugPrint(F("SingleLegHold: On\n"));
          }
          else {
            SoundEvent(3);
            DebugPrint(F("SingleLegHold: Off\n"));
          }
        }

        SingleLegX = (PS2.Analog(PSS_LX) - 128) / 2;
        SingleLegY = (PS2.Analog(PSS_RY) - 128) / 10;
        SingleLegZ = (PS2.Analog(PSS_LY) - 128) / 2;
      }

      //Reset BodyShiftY
      if (ControlMode != TRANSLATE_MODE && ControlMode != ROTATE_MODE) {
        BodyShiftY = 0;
      }

      //Calculate BodyPosY
      BodyPosY = min(max(BodyOffsetY + BodyShiftY, 0), 100);

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
    DebugPrint(F("PS2 controller is not detected!\n"));
    PowerSwitch();
  }
}
