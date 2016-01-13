/**
 * Project:     Lynxmotion Phoenix
 * Description: Phoenix configuration file
 * Version:     v2.5
 * Author:      Jeroen Janssen (aka Xan)
 *              Kåre Halvorsen (aka Zenta)
 *              Kompanets Konstantin (aka I2M)
 */

#ifndef PHOENIX_CFG_H
#define PHOENIX_CFG_H

//[SOFTWARE VERSION]
#define VERSION            "2.5.5"

//[COMPILATION OPTIONS]
#define DEBUG_MODE
#define SOUND_MODE

//[ARDUINO PIN NUMBERS]
#define PS2_DAT            6  //PS2 controller data
#define PS2_CMD            7  //PS2 controller command
#define PS2_ATT            8  //PS2 controller attention
#define PS2_CLK            9  //PS2 controller clock

#define SSC_RX             10 //Arduino RX <- SSC32 TX
#define SSC_TX             11 //Arduino TX -> SSC32 RX

#define BUZZER             12 //Buzzer module

//[SERIAL BAUD RATES]
#define DBG_BAUD           115200 //Debug baud rate
#define SSC_BAUD           115200 //SSC32 baud rate

//[ANALOG INPUT]
#define TRAVEL_DEADZONE    4 //Dead zone for the analog input

//[SSC PIN NUMBERS]
#define RRCoxaPin          0  //Right Rear leg hip horizontal
#define RRFemurPin         1  //Right Rear leg hip vertical
#define RRTibiaPin         2  //Right Rear leg knee

#define RMCoxaPin          4  //Right Middle leg hip horizontal
#define RMFemurPin         5  //Right Middle leg hip vertical
#define RMTibiaPin         6  //Right Middle leg knee

#define RFCoxaPin          8  //Right Front leg hip horizontal
#define RFFemurPin         9  //Right Front leg hip vertical
#define RFTibiaPin         10 //Right Front leg knee

#define LRCoxaPin          16 //Left Rear leg hip horizontal
#define LRFemurPin         17 //Left Rear leg hip vertical
#define LRTibiaPin         18 //Left Rear leg knee

#define LMCoxaPin          20 //Left Middle leg hip horizontal
#define LMFemurPin         21 //Left Middle leg hip vertical
#define LMTibiaPin         22 //Left Middle leg knee

#define LFCoxaPin          24 //Left Front leg hip horizontal
#define LFFemurPin         25 //Left Front leg hip vertical
#define LFTibiaPin         26 //Left Front leg knee

//[MIN/MAX ANGLES]
#define RRCoxaMin         -26 //Mechanical limits of the Right Rear leg
#define RRCoxaMax          74
#define RRFemurMin        -101
#define RRFemurMax         95
#define RRTibiaMin        -106
#define RRTibiaMax         77

#define RMCoxaMin         -53 //Mechanical limits of the Right Middle leg
#define RMCoxaMax          53
#define RMFemurMin        -101
#define RMFemurMax         95
#define RMTibiaMin        -106
#define RMTibiaMax         77

#define RFCoxaMin         -58 //Mechanical limits of the Right Front leg
#define RFCoxaMax          74
#define RFFemurMin        -101
#define RFFemurMax         95
#define RFTibiaMin        -106
#define RFTibiaMax         77

#define LRCoxaMin         -74 //Mechanical limits of the Left Rear leg
#define LRCoxaMax          26
#define LRFemurMin        -95
#define LRFemurMax         101
#define LRTibiaMin        -77
#define LRTibiaMax         106

#define LMCoxaMin         -53 //Mechanical limits of the Left Middle leg
#define LMCoxaMax          53
#define LMFemurMin        -95
#define LMFemurMax         101
#define LMTibiaMin        -77
#define LMTibiaMax         106

#define LFCoxaMin         -74 //Mechanical limits of the Left Front leg
#define LFCoxaMax          58
#define LFFemurMin        -95
#define LFFemurMax         101
#define LFTibiaMin        -77
#define LFTibiaMax         106

//[COXA ANGLES]
#define RRLegAngle        -60 //Default coxa setup angle
#define RMLegAngle         0
#define RFLegAngle         60
#define LRLegAngle        -60
#define LMLegAngle         0
#define LFLegAngle         60

//[LEG DIMENSIONS]
#define CoxaLength         30  //Length of the coxa
#define FemurLength        85  //Length of the femur
#define TibiaLength        125 //Length of the tibia

//[BODY DIMENSIONS]
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

//[START POSITIONS]
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
