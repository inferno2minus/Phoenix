/**
 * Project Lynxmotion Phoenix
 * Description: Phoenix control file
 * Version: v1.0
 * Programmer: Jeroen Janssen (aka Xan)
 * Porting: Kompanets Konstantin (aka I2M)
 *
 * Hardware setup: Playstation 2 gamepad
 */

#include <PS2X_lib.h>

//[CONSTANTS]
#define WALKMODE         0
#define TRANSLATEMODE    1
#define ROTATEMODE       2
#define SINGLELEGMODE    3
//#define GPPLAYERMODE   4

#define cGaitsNumber     5
#define cMaxPS2Error     5 //How many times through the loop will we go before shutting off robot?
#define cMaxBodyY        100

//[VARIABLES]
PS2X    PS2;
short   PS2ErrorCount;
short   BodyYOffset;
short   BodyYShift;
byte    ControlMode;
bool    DoubleHeightOn;
bool    DoubleTravelOn;
bool    WalkMethod;

#ifdef cBUZZER
extern void MSound(byte cNotes, ...);
#endif

//[InitController] Initialize the PS2 controller
void InitController() {
  PS2.config_gamepad(cPS2_CLK, cPS2_CMD, cPS2_SEL, cPS2_DAT);

  PS2ErrorCount = 0;
  BodyYOffset = 0;
  BodyYShift = 0;

  ControlMode = WALKMODE;
  DoubleHeightOn = false;
  DoubleTravelOn = false;
  WalkMethod = false;

  SpeedControl = 100;
}

//[ControlInput] Reads the input data from the PS2 controller and processes the data to the parameters.
void ControlInput() {
  PS2.read_gamepad();

  //Wish the library had a valid way to verify that the read_gamepad succeeded... Will hack for now
  if ((PS2.Analog(1) & 0xf0) == 0x70) {

    PS2ErrorCount = 0; //Clear out error count...

    //Switch bot on/off
    if (PS2.ButtonPressed(PSB_START)) { //Start Button Test
      if (HexOn) {
        PS2TurnRobotOff(); //Turn off
      }
      else {
        HexOn = 1; //Turn on
      }
    }

    if (HexOn) {

      //Translate mode
      if (PS2.ButtonPressed(PSB_L1)) { //L1 Button Test
#ifdef cBUZZER
        MSound(1, 50, 2000);
#endif
        if (ControlMode != TRANSLATEMODE) {
          ControlMode = TRANSLATEMODE;
        }
        else if (SelectedLeg == 255) {
          ControlMode = WALKMODE;
        }
        else {
          ControlMode = SINGLELEGMODE;
        }
      }

      //Rotate mode
      if (PS2.ButtonPressed(PSB_L2)) { //L2 Button Test
#ifdef cBUZZER
        MSound(1, 50, 2000);
#endif
        if (ControlMode != ROTATEMODE) {
          ControlMode = ROTATEMODE;
        }
        else if (SelectedLeg == 255) {
          ControlMode = WALKMODE;
        }
        else {
          ControlMode = SINGLELEGMODE;
        }
      }

      //Single leg mode
      if (PS2.ButtonPressed(PSB_CIRCLE)) { //Circle Button Test
        if (abs(TravelLengthX) < cTravelDeadZone &&
          abs(TravelLengthZ) < cTravelDeadZone &&
          abs(TravelRotationY * 2) < cTravelDeadZone) {
#ifdef cBUZZER
          MSound(1, 50, 2000);
#endif
          if (ControlMode != SINGLELEGMODE) {
            ControlMode = SINGLELEGMODE;
            if (SelectedLeg == 255) { //Select leg if none is selected
              SelectedLeg = cRF; //Startleg
            }
          }
          else {
            ControlMode = WALKMODE;
            SelectedLeg = 255;
          }
        }
      }

      //[Common functions]
      //Switch Balance mode on/off 
      if (PS2.ButtonPressed(PSB_SQUARE)) { //Square Button Test
        BalanceMode = !BalanceMode;
        if (BalanceMode) {
#ifdef cBUZZER
          MSound(1, 250, 1500);
#endif
        }
        else {
#ifdef cBUZZER
          MSound(2, 100, 2000, 50, 4000);
#endif
        }
      }

      //Stand up, sit down 
      if (PS2.ButtonPressed(PSB_TRIANGLE)) { //Triangle Button Test
        if (BodyYOffset > 0) {
          BodyYOffset = 0;
        }
        else {
          BodyYOffset = 35;
        }
      }

      if (PS2.ButtonPressed(PSB_PAD_UP)) { //D-Up Button Test
        if (BodyYOffset < cMaxBodyY) {
          BodyYOffset += 10;
        }
        else {
          BodyYOffset = cMaxBodyY;
        }
      }

      if (PS2.ButtonPressed(PSB_PAD_DOWN)) { //D-Down Button Test
        if (BodyYOffset > 10) {
          BodyYOffset -= 10;
        }
        else {
          BodyYOffset = 0;
        }
      }

      if (PS2.ButtonPressed(PSB_PAD_RIGHT)) { //D-Right Button Test
        if (SpeedControl > 0) {
          SpeedControl -= 50;
#ifdef cBUZZER
          MSound(1, 50, 2000);
#endif
        }
      }

      if (PS2.ButtonPressed(PSB_PAD_LEFT)) { //D-Left Button Test
        if (SpeedControl < 2000) {
          SpeedControl += 50;
#ifdef cBUZZER
          MSound(1, 50, 2000);
#endif
        }
      }

      //[Walk functions]
      if (ControlMode == WALKMODE) {

        //Switch gates
        if (PS2.ButtonPressed(PSB_SELECT) && //Select Button Test
        abs(TravelLengthX) < cTravelDeadZone && //No movement
        abs(TravelLengthZ) < cTravelDeadZone &&
          abs(TravelRotationY * 2) < cTravelDeadZone) {
          if (GaitType < cGaitsNumber - 1) {
#ifdef cBUZZER
            MSound(1, 50, 2000);
#endif
            GaitType++;
          }
          else {
#ifdef cBUZZER
            MSound(2, 50, 2000, 50, 2250);
#endif
            GaitType = 0;
          }
          GaitSelect();
        }

        //Double leg lift height
        if (PS2.ButtonPressed(PSB_R1)) { //R1 Button Test
#ifdef cBUZZER
          MSound(1, 50, 2000);
#endif
          DoubleHeightOn = !DoubleHeightOn;
          if (DoubleHeightOn) {
            LegLiftHeight = 80;
          }
          else {
            LegLiftHeight = 50;
          }
        }

        //Double Travel Length
        if (PS2.ButtonPressed(PSB_R2)) { //R2 Button Test
#ifdef cBUZZER
          MSound(1, 50, 2000);
#endif
          DoubleTravelOn = !DoubleTravelOn;
        }

        //Switch between Walk method 1 and Walk method 2
        if (PS2.ButtonPressed(PSB_R3)) { //R3 Button Test
#ifdef cBUZZER
          MSound(1, 50, 2000);
#endif
          WalkMethod = !WalkMethod;
        }

        //Walking
        if (WalkMethod) { //Walk Method
          TravelLengthZ = (PS2.Analog(PSS_RY) - 128); //Right Stick Up/Down
        }
        else {
          TravelLengthX = -(PS2.Analog(PSS_LX) - 128);
          TravelLengthZ = (PS2.Analog(PSS_LY) - 128);
        }

        if (!DoubleTravelOn) { //Double travel length
          TravelLengthX /= 2;
          TravelLengthZ /= 2;
        }

        TravelRotationY = -(PS2.Analog(PSS_RX) - 128) / 4; //Right Stick Left/Right
      }

      //[Translate functions]
      //BodyYShift = 0;
      if (ControlMode == TRANSLATEMODE) {
        BodyPosX = (PS2.Analog(PSS_LX) - 128) / 2;
        BodyPosZ = -(PS2.Analog(PSS_LY) - 128) / 3;
        BodyRotY = (PS2.Analog(PSS_RX) - 128) * 2;
        BodyYShift = (-(PS2.Analog(PSS_RY) - 128) / 2);
      }

      //[Rotate functions]
      if (ControlMode == ROTATEMODE) {
        BodyRotX = (PS2.Analog(PSS_LY) - 128);
        BodyRotY = (PS2.Analog(PSS_RX) - 128) * 2;
        BodyRotZ = (PS2.Analog(PSS_LX) - 128);
        BodyYShift = (-(PS2.Analog(PSS_RY) - 128) / 2);
      }

      //[Single leg functions]
      if (ControlMode == SINGLELEGMODE) {

        //Switch leg for single leg control
        if (PS2.ButtonPressed(PSB_SELECT)) { //Select Button Test
#ifdef cBUZZER
          MSound(1, 50, 2000);
#endif
          if (SelectedLeg < 5) {
            SelectedLeg++;
          }
          else {
            SelectedLeg = 0;
          }
        }

        SLLegX = (PS2.Analog(PSS_LX) - 128) / 2; //Left Stick Right/Left
        SLLegY = (PS2.Analog(PSS_RY) - 128) / 10; //Right Stick Up/Down
        SLLegZ = (PS2.Analog(PSS_LY) - 128) / 2; //Left Stick Up/Down

        //Hold single leg in place
        if (PS2.ButtonPressed(PSB_R2)) { //R2 Button Test
#ifdef cBUZZER
          MSound(1, 50, 2000);
#endif
          SLHold = !SLHold;
        }
      }

      //Calculate walking time delay
      InputTimeDelay = 128 - max(max(abs(PS2.Analog(PSS_LX) - 128), abs(PS2.Analog(PSS_LY) - 128)), abs(PS2.Analog(PSS_RX) - 128));
    }

    //Calculate BodyPosY
    BodyPosY = min(max(BodyYOffset + BodyYShift, 0), cMaxBodyY);
  }
  else if (PS2ErrorCount < cMaxPS2Error) {
    PS2ErrorCount++;
  }
  else if (HexOn) {
    PS2TurnRobotOff();
  }
}

void PS2TurnRobotOff() {
  BodyPosX = 0;
  BodyPosY = 0;
  BodyPosZ = 0;
  BodyRotX = 0;
  BodyRotY = 0;
  BodyRotZ = 0;
  TravelLengthX = 0;
  TravelLengthZ = 0;
  TravelRotationY = 0;
  BodyYOffset = 0;
  BodyYShift = 0;
  SelectedLeg = 255;
  HexOn = 0;
}

