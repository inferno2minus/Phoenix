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

//[SERIAL CONNECTIONS]
#define cSSC_BAUD          38400  //SSC32 BAUD rate
#define cDBG_BAUD          115200 //DEBUG BAUD rate

//[DEBUG CONNECTIONS]
#define DBGSerial          Serial

//[ARDUINO PIN NUMBERS]
#define cPS2_DAT           6  //PS2 Controller DAT
#define cPS2_CMD           7  //PS2 Controller CMD
#define cPS2_SEL           8  //PS2 Controller SEL
#define cPS2_CLK           9  //PS2 Controller CLK
#define cSSC_TX            10 //(SSC32 TX -> Arduino RX)
#define cSSC_RX            11 //(SSC32 RX <- Arduino TX)
#define cBUZZER            12 //Buzzer module

//Add support for running on non-mega Arduino boards as well.
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
#define cRRCoxaMin1       -260 //Mechanical limits of the Right Rear Leg, decimals = 1
#define cRRCoxaMax1        740
#define cRRFemurMin1      -1010
#define cRRFemurMax1       950
#define cRRTibiaMin1      -1060
#define cRRTibiaMax1       770

#define cRMCoxaMin1       -530 //Mechanical limits of the Right Middle Leg, decimals = 1
#define cRMCoxaMax1        530
#define cRMFemurMin1      -1010
#define cRMFemurMax1       950
#define cRMTibiaMin1      -1060
#define cRMTibiaMax1       770

#define cRFCoxaMin1       -580 //Mechanical limits of the Right Front Leg, decimals = 1
#define cRFCoxaMax1        740
#define cRFFemurMin1      -1010
#define cRFFemurMax1       950
#define cRFTibiaMin1      -1060
#define cRFTibiaMax1       770

#define cLRCoxaMin1       -740 //Mechanical limits of the Left Rear Leg, decimals = 1
#define cLRCoxaMax1        260
#define cLRFemurMin1      -950
#define cLRFemurMax1       1010
#define cLRTibiaMin1      -770
#define cLRTibiaMax1       1060

#define cLMCoxaMin1       -530 //Mechanical limits of the Left Middle Leg, decimals = 1
#define cLMCoxaMax1        530
#define cLMFemurMin1      -950
#define cLMFemurMax1       1010
#define cLMTibiaMin1      -770
#define cLMTibiaMax1       1060

#define cLFCoxaMin1       -740 //Mechanical limits of the Left Front Leg, decimals = 1
#define cLFCoxaMax1        580
#define cLFFemurMin1      -950
#define cLFFemurMax1       1010
#define cLFTibiaMin1      -770
#define cLFTibiaMax1       1060

//[BODY DIMENSIONS]
#define cCoxaLength        29  //Length of the Coxa [mm]
#define cFemurLength       76  //Length of the Femur [mm]
#define cTibiaLength       106 //Length of the Tibia [mm]

#define cRRCoxaAngle1     -600 //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1      0   //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1      600 //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1     -600 //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1      0   //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1      600 //Default Coxa setup angle, decimals = 1

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
#define CHexInitXZCos60    53
#define CHexInitXZSin60    91
#define CHexInitY          25

#define cRRInitPosX        CHexInitXZCos60 //Start positions of the Right Rear leg
#define cRRInitPosY        CHexInitY
#define cRRInitPosZ        CHexInitXZSin60

#define cRMInitPosX        cHexInitXZ      //Start positions of the Right Middle leg
#define cRMInitPosY        CHexInitY
#define cRMInitPosZ        0

#define cRFInitPosX        CHexInitXZCos60 //Start positions of the Right Front leg
#define cRFInitPosY        CHexInitY
#define cRFInitPosZ       -CHexInitXZSin60

#define cLRInitPosX        CHexInitXZCos60 //Start positions of the Left Rear leg
#define cLRInitPosY        CHexInitY
#define cLRInitPosZ        CHexInitXZSin60

#define cLMInitPosX        cHexInitXZ      //Start positions of the Left Middle leg
#define cLMInitPosY        CHexInitY
#define cLMInitPosZ        0

#define cLFInitPosX        CHexInitXZCos60 //Start positions of the Left Front leg
#define cLFInitPosY        CHexInitY
#define cLFInitPosZ       -CHexInitXZSin60

#endif
