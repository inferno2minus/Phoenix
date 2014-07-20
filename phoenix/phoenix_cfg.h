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
#define PS2_DAT            6  //PS2 controller DAT
#define PS2_CMD            7  //PS2 controller CMD
#define PS2_SEL            8  //PS2 controller SEL
#define PS2_CLK            9  //PS2 controller CLK
#define SSC_TX             10 //(SSC32 TX -> Arduino RX)
#define SSC_RX             11 //(SSC32 RX <- Arduino TX)
#define BUZZER             12 //Buzzer module

//[SERIAL BAUD RATES]
#define DBG_BAUD           115200 //Debug baud rate
#define SSC_BAUD           115200 //SSC32 baud rate

//[SERIAL CONNECTIONS]
#define DBGSerial          Serial

//Add support for running on non-mega Arduino boards as well
#ifndef UBRR1H
SoftwareSerial SSCSerial(SSC_TX, SSC_RX);
#else
#define SSCSerial          Serial1
#endif

//[ANALOG INPUT]
#define TravelDeadZone     4 //The deadzone for the analog input from the remote

//[CONSTANTS LEGS]
#define RR                 0
#define RM                 1
#define RF                 2
#define LR                 3
#define LM                 4
#define LF                 5

//[SSC PIN NUMBERS]
#define RRCoxaPin          0  //Rear Right leg hip horizontal
#define RRFemurPin         1  //Rear Right leg hip vertical
#define RRTibiaPin         2  //Rear Right leg knee

#define RMCoxaPin          4  //Middle Right leg hip horizontal
#define RMFemurPin         5  //Middle Right leg hip vertical
#define RMTibiaPin         6  //Middle Right leg knee

#define RFCoxaPin          8  //Front Right leg hip horizontal
#define RFFemurPin         9  //Front Right leg hip vertical
#define RFTibiaPin         10 //Front Right leg knee

#define LRCoxaPin          16 //Rear Left leg hip horizontal
#define LRFemurPin         17 //Rear Left leg hip vertical
#define LRTibiaPin         18 //Rear Left leg knee

#define LMCoxaPin          20 //Middle Left leg hip horizontal
#define LMFemurPin         21 //Middle Left leg hip vertical
#define LMTibiaPin         22 //Middle Left leg knee

#define LFCoxaPin          24 //Front Left leg hip horizontal
#define LFFemurPin         25 //Front Left leg hip vertical
#define LFTibiaPin         26 //Front Left leg knee

//[MIN/MAX ANGLES]
#define RRCoxaMin         -260 //Mechanical limits of the Right Rear leg, decimals = 1
#define RRCoxaMax          740
#define RRFemurMin        -1010
#define RRFemurMax         950
#define RRTibiaMin        -1060
#define RRTibiaMax         770

#define RMCoxaMin         -530 //Mechanical limits of the Right Middle leg, decimals = 1
#define RMCoxaMax          530
#define RMFemurMin        -1010
#define RMFemurMax         950
#define RMTibiaMin        -1060
#define RMTibiaMax         770

#define RFCoxaMin         -580 //Mechanical limits of the Right Front leg, decimals = 1
#define RFCoxaMax          740
#define RFFemurMin        -1010
#define RFFemurMax         950
#define RFTibiaMin        -1060
#define RFTibiaMax         770

#define LRCoxaMin         -740 //Mechanical limits of the Left Rear leg, decimals = 1
#define LRCoxaMax          260
#define LRFemurMin        -950
#define LRFemurMax         1010
#define LRTibiaMin        -770
#define LRTibiaMax         1060

#define LMCoxaMin         -530 //Mechanical limits of the Left Middle leg, decimals = 1
#define LMCoxaMax          530
#define LMFemurMin        -950
#define LMFemurMax         1010
#define LMTibiaMin        -770
#define LMTibiaMax         1060

#define LFCoxaMin         -740 //Mechanical limits of the Left Front leg, decimals = 1
#define LFCoxaMax          580
#define LFFemurMin        -950
#define LFFemurMax         1010
#define LFTibiaMin        -770
#define LFTibiaMax         1060

//[COXA ANGLES]
#define RRLegAngle        -600 //Default Coxa setup angle, decimals = 1
#define RMLegAngle         0
#define RFLegAngle         600
#define LRLegAngle        -600
#define LMLegAngle         0
#define LFLegAngle         600

//[BODY DIMENSIONS]
#define CoxaLength         29  //Length of the coxa
#define FemurLength        83  //Length of the femur
#define TibiaLength        106 //Length of the tibia

#define RROffsetX         -40  //Distance X from center of the body to the Right Rear coxa
#define RROffsetZ          75  //Distance Z from center of the body to the Right Rear coxa
#define RMOffsetX         -65  //Distance X from center of the body to the Right Middle coxa
#define RMOffsetZ          0   //Distance Z from center of the body to the Right Middle coxa
#define RFOffsetX         -40  //Distance X from center of the body to the Right Front coxa
#define RFOffsetZ         -75  //Distance Z from center of the body to the Right Front coxa

#define LROffsetX          40  //Distance X from center of the body to the Left Rear coxa
#define LROffsetZ          75  //Distance Z from center of the body to the Left Rear coxa
#define LMOffsetX          65  //Distance X from center of the body to the Left Middle coxa
#define LMOffsetZ          0   //Distance Z from center of the body to the Left Middle coxa
#define LFOffsetX          40  //Distance X from center of the body to the Left Front coxa
#define LFOffsetZ         -75  //Distance Z from center of the body to the Left Front coxa

//[START POSITIONS FEET]
#define InitXZ             105
#define InitXZCos60        53
#define InitXZSin60        91
#define InitY              25

#define RRInitPosX         InitXZCos60 //Start positions of the Right Rear leg
#define RRInitPosY         InitY
#define RRInitPosZ         InitXZSin60

#define RMInitPosX         InitXZ      //Start positions of the Right Middle leg
#define RMInitPosY         InitY
#define RMInitPosZ         0

#define RFInitPosX         InitXZCos60 //Start positions of the Right Front leg
#define RFInitPosY         InitY
#define RFInitPosZ        -InitXZSin60

#define LRInitPosX         InitXZCos60 //Start positions of the Left Rear leg
#define LRInitPosY         InitY
#define LRInitPosZ         InitXZSin60

#define LMInitPosX         InitXZ      //Start positions of the Left Middle leg
#define LMInitPosY         InitY
#define LMInitPosZ         0

#define LFInitPosX         InitXZCos60 //Start positions of the Left Front leg
#define LFInitPosY         InitY
#define LFInitPosZ        -InitXZSin60

#endif
