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

#define WALKMODE           0
#define TRANSLATEMODE      1
#define ROTATEMODE         2
#define SINGLELEGMODE      3

PS2X    PS2;
short   BodyYShift;
short   BodyYOffset;
byte    ControlMode;
bool    DoubleHeight;
bool    DoubleTravel;
bool    WalkMethod;

void InitControl() {
  PS2.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT);

  ControlMode = WALKMODE;
  DoubleHeight = false;
  DoubleTravel = false;
  WalkMethod = false;

  SpeedControl = 100;
}

void ReadControl() {
  if (PS2.read_gamepad()) {
    //Switch bot on/off
    if (PS2.ButtonPressed(PSB_START) && !GaitInMotion) { //Start button
      if (HexOn) {
        TurnRobotOff(); //Turn off
      }
      else {
        HexOn = true; //Turn on
#ifdef DEBUG_MODE
        DBGSerial.println("Power: Turn on");
#endif
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
          DBGSerial.println("ControlMode: TRANSLATEMODE");
#endif
        }
        else if (SelectedLeg == 255) {
          ControlMode = WALKMODE;
#ifdef DEBUG_MODE
          DBGSerial.println("ControlMode: WALKMODE");
#endif
        }
        else {
          ControlMode = SINGLELEGMODE;
#ifdef DEBUG_MODE
          DBGSerial.println("ControlMode: SINGLELEGMODE");
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
          DBGSerial.println("ControlMode: ROTATEMODE");
#endif
        }
        else if (SelectedLeg == 255) {
          ControlMode = WALKMODE;
#ifdef DEBUG_MODE
          DBGSerial.println("ControlMode: WALKMODE");
#endif
        }
        else {
          ControlMode = SINGLELEGMODE;
#ifdef DEBUG_MODE
          DBGSerial.println("ControlMode: SINGLELEGMODE");
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
          DBGSerial.println("ControlMode: SINGLELEGMODE");
#endif
          if (SelectedLeg == 255) {
            SelectedLeg = RF; //Start leg
          }
        }
        else {
          ControlMode = WALKMODE;
#ifdef DEBUG_MODE
          DBGSerial.println("ControlMode: WALKMODE");
#endif
          SelectedLeg = 255;
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
          DBGSerial.println("BalanceMode: On");
        }
        else {
          DBGSerial.println("BalanceMode: Off");
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
        DBGSerial.print("BodyYOffset: ");
        DBGSerial.println(BodyYOffset, DEC);
#endif
      }

      //Stand up
      if (PS2.ButtonPressed(PSB_PAD_UP)) { //D-Up button
        if (BodyYOffset < 100) {
          BodyYOffset += 10;
#ifdef DEBUG_MODE
          DBGSerial.print("BodyYOffset: ");
          DBGSerial.println(BodyYOffset, DEC);
#endif
        }
      }

      //Sit down
      if (PS2.ButtonPressed(PSB_PAD_DOWN)) { //D-Down button
        if (BodyYOffset > 0) {
          BodyYOffset -= 10;
#ifdef DEBUG_MODE
          DBGSerial.print("BodyYOffset: ");
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
        DBGSerial.print("SpeedControl: ");
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
          DBGSerial.print("SpeedControl: ");
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
          DBGSerial.print("SpeedControl: ");
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
          DBGSerial.print("GaitType: ");
          switch(GaitType) {
          case 0:
            DBGSerial.println("Ripple 12");
            break;
          case 1:
            DBGSerial.println("Tripod 6");
            break;
          case 2:
            DBGSerial.println("Tripod 8");
            break;
          case 3:
            DBGSerial.println("Tripod 12");
            break;
          case 4:
            DBGSerial.println("Tripod 16");
            break;
          case 5:
            DBGSerial.println("Wave 24");
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
            DBGSerial.println("DoubleHeight: On");
#endif
          }
          else {
            LegLiftHeight = 50;
#ifdef DEBUG_MODE
            DBGSerial.println("DoubleHeight: Off");
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
            DBGSerial.println("DoubleTravel: On");
          }
          else {
            DBGSerial.println("DoubleTravel: Off");
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
            DBGSerial.println("WalkMethod: 1");
          }
          else {
            DBGSerial.println("WalkMethod: 2");
          }
#endif
        }

        //Walking
        if (WalkMethod) { //Walk method
          TravelLengthZ = (PS2.Analog(PSS_RY) - 128); //Right stick Up/Down
        }
        else {
          TravelLengthX = -(PS2.Analog(PSS_LX) - 128);
          TravelLengthZ = (PS2.Analog(PSS_LY) - 128);
        }

        if (!DoubleTravel) { //Double travel length
          TravelLengthX /= 2;
          TravelLengthZ /= 2;
        }

        TravelLengthY = -(PS2.Analog(PSS_RX) - 128) / 4; //Right stick Left/Right
      }

      //[Translate functions]
      BodyYShift = 0;
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
          DBGSerial.print("SelectedLeg: ");
          switch(SelectedLeg) {
          case 0:
            DBGSerial.println("RR");
            break;
          case 1:
            DBGSerial.println("RM");
            break;
          case 2:
            DBGSerial.println("RF");
            break;
          case 3:
            DBGSerial.println("LR");
            break;
          case 4:
            DBGSerial.println("LM");
            break;
          case 5:
            DBGSerial.println("LF");
            break;
          }
#endif
        }

        SLLegX = (PS2.Analog(PSS_LX) - 128) / 2; //Left stick Right/Left
        SLLegY = (PS2.Analog(PSS_RY) - 128) / 10; //Right stick Up/Down
        SLLegZ = (PS2.Analog(PSS_LY) - 128) / 2; //Left stick Up/Down

        //Hold single leg in place
        if (PS2.ButtonPressed(PSB_R2)) { //R2 button
#ifdef SOUND_MODE
          Sound.play(2217, 40);
#endif
          SLHold = !SLHold;
#ifdef DEBUG_MODE
          if (SLHold) {
            DBGSerial.println("SLHold: On");
          }
          else {
            DBGSerial.println("SLHold: Off");
          }
#endif
        }
      }

      //Calculate walking time delay
      InputTimeDelay = 128 - max(max(abs(PS2.Analog(PSS_LX) - 128),
        abs(PS2.Analog(PSS_LY) - 128)), abs(PS2.Analog(PSS_RX) - 128));
    }

    //Calculate BodyPosY
    BodyPosY = min(max(BodyYOffset + BodyYShift, 0), 100);
  }
  else if (HexOn) {
    TurnRobotOff();
  }
}

void TurnRobotOff() {
#ifdef DEBUG_MODE
  DBGSerial.println("Power: Turn off");
#endif
  BodyPosX = 0;
  BodyPosY = 0;
  BodyPosZ = 0;
  BodyRotX = 0;
  BodyRotY = 0;
  BodyRotZ = 0;
  TravelLengthX = 0;
  TravelLengthY = 0;
  TravelLengthZ = 0;
  BodyYOffset = 0;
  BodyYShift = 0;
  SelectedLeg = 255;
  HexOn = false;
}
