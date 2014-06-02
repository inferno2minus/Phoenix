/**
 * Project Lynxmotion Phoenix
 * Description: Phoenix configuration file
 * Version: v1.0
 * Programmer: Jeroen Janssen (aka Xan)
 * Porting: Kompanets Konstantin (aka I2M)
 *
 * Hardware setup: Arduino, SSC32 V2, (See further for connections)
 */

#ifndef PHOENIX_CFG_H
#define PHOENIX_CFG_H

//[CONDITIONAL COMPILING]
#define DEBUG_MODE
#define SOUND_MODE

//[ARDUINO PIN NUMBERS]
#define cPS2_DAT           6  //PS2 Controller DAT
#define cPS2_CMD           7  //PS2 Controller CMD
#define cPS2_SEL           8  //PS2 Controller SEL
#define cPS2_CLK           9  //PS2 Controller CLK
#define cSSC_TX            10 //(SSC32 TX -> Arduino RX)
#define cSSC_RX            11 //(SSC32 RX <- Arduino TX)
#define cBUZZER            12 //Buzzer module

//[SERIAL BAUD RATES]
#define cDBG_BAUD          115200 //DEBUG BAUD rate
#define cSSC_BAUD          115200 //SSC32 BAUD rate

//[SERIAL CONNECTIONS]
#define DBGSerial          Serial

//Add support for running on non-mega Arduino boards as well
#ifndef UBRR1H
SoftwareSerial SSCSerial(cSSC_TX, cSSC_RX);
#else
#define SSCSerial          Serial1
#endif

//[SSC PIN NUMBERS]
#define cRRCoxaPin         0  //Rear Right leg Hip Horizontal
#define cRRFemurPin        1  //Rear Right leg Hip Vertical
#define cRRTibiaPin        2  //Rear Right leg Knee

#define cRMCoxaPin         4  //Middle Right leg Hip Horizontal
#define cRMFemurPin        5  //Middle Right leg Hip Vertical
#define cRMTibiaPin        6  //Middle Right leg Knee

#define cRFCoxaPin         8  //Front Right leg Hip Horizontal
#define cRFFemurPin        9  //Front Right leg Hip Vertical
#define cRFTibiaPin        10 //Front Right leg Knee

#define cLRCoxaPin         16 //Rear Left leg Hip Horizontal
#define cLRFemurPin        17 //Rear Left leg Hip Vertical
#define cLRTibiaPin        18 //Rear Left leg Knee

#define cLMCoxaPin         20 //Middle Left leg Hip Horizontal
#define cLMFemurPin        21 //Middle Left leg Hip Vertical
#define cLMTibiaPin        22 //Middle Left leg Knee

#define cLFCoxaPin         24 //Front Left leg Hip Horizontal
#define cLFFemurPin        25 //Front Left leg Hip Vertical
#define cLFTibiaPin        26 //Front Left leg Knee

//[MIN/MAX ANGLES]
#define cRRCoxaMin        -260 //Mechanical limits of the Right Rear Leg, decimals = 1
#define cRRCoxaMax         740
#define cRRFemurMin       -1010
#define cRRFemurMax        950
#define cRRTibiaMin       -1060
#define cRRTibiaMax        770

#define cRMCoxaMin        -530 //Mechanical limits of the Right Middle Leg, decimals = 1
#define cRMCoxaMax         530
#define cRMFemurMin       -1010
#define cRMFemurMax        950
#define cRMTibiaMin       -1060
#define cRMTibiaMax        770

#define cRFCoxaMin        -580 //Mechanical limits of the Right Front Leg, decimals = 1
#define cRFCoxaMax         740
#define cRFFemurMin       -1010
#define cRFFemurMax        950
#define cRFTibiaMin       -1060
#define cRFTibiaMax        770

#define cLRCoxaMin        -740 //Mechanical limits of the Left Rear Leg, decimals = 1
#define cLRCoxaMax         260
#define cLRFemurMin       -950
#define cLRFemurMax        1010
#define cLRTibiaMin       -770
#define cLRTibiaMax        1060

#define cLMCoxaMin        -530 //Mechanical limits of the Left Middle Leg, decimals = 1
#define cLMCoxaMax         530
#define cLMFemurMin       -950
#define cLMFemurMax        1010
#define cLMTibiaMin       -770
#define cLMTibiaMax        1060

#define cLFCoxaMin        -740 //Mechanical limits of the Left Front Leg, decimals = 1
#define cLFCoxaMax         580
#define cLFFemurMin       -950
#define cLFFemurMax        1010
#define cLFTibiaMin       -770
#define cLFTibiaMax        1060

//[BODY DIMENSIONS]
#define cCoxaLength        29  //Length of the Coxa [mm]
#define cFemurLength       76  //Length of the Femur [mm]
#define cTibiaLength       106 //Length of the Tibia [mm]

#define cRRCoxaAngle      -600 //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle       0   //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle       600 //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle      -600 //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle       0   //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle       600 //Default Coxa setup angle, decimals = 1

#define cRROffsetX        -43  //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ         82  //Distance Z from center of the body to the Right Rear coxa
#define cRMOffsetX        -63  //Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ         0   //Distance Z from center of the body to the Right Middle coxa
#define cRFOffsetX        -43  //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ        -82  //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX         43  //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ         82  //Distance Z from center of the body to the Left Rear coxa
#define cLMOffsetX         63  //Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ         0   //Distance Z from center of the body to the Left Middle coxa
#define cLFOffsetX         43  //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ        -82  //Distance Z from center of the body to the Left Front coxa

//[START POSITIONS FEET]
#define cHexInitXZ         105
#define cHexInitXZCos60    53
#define cHexInitXZSin60    91
#define cHexInitY          25

#define cRRInitPosX        cHexInitXZCos60 //Start positions of the Right Rear leg
#define cRRInitPosY        cHexInitY
#define cRRInitPosZ        cHexInitXZSin60

#define cRMInitPosX        cHexInitXZ      //Start positions of the Right Middle leg
#define cRMInitPosY        cHexInitY
#define cRMInitPosZ        0

#define cRFInitPosX        cHexInitXZCos60 //Start positions of the Right Front leg
#define cRFInitPosY        cHexInitY
#define cRFInitPosZ       -cHexInitXZSin60

#define cLRInitPosX        cHexInitXZCos60 //Start positions of the Left Rear leg
#define cLRInitPosY        cHexInitY
#define cLRInitPosZ        cHexInitXZSin60

#define cLMInitPosX        cHexInitXZ      //Start positions of the Left Middle leg
#define cLMInitPosY        cHexInitY
#define cLMInitPosZ        0

#define cLFInitPosX        cHexInitXZCos60 //Start positions of the Left Front leg
#define cLFInitPosY        cHexInitY
#define cLFInitPosZ       -cHexInitXZSin60

#endif
