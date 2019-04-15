/*
 * General.h
 *
 * Created: 02/04/2019 11:11:56
 * Author: Marek Kujawa
 */ 

#ifndef GENERAL_H
#define GENERAL_H

/* General Defines */
#define OFF 0
#define ON 1
#define NO 0
#define YES 1
#define CCW 0
#define CW 1
#define LEFT 0
#define RIGHT 1
#define DOWN 0
#define UP 1

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(Byte)  \
	(Byte & 0x80 ? '1' : ' '), \
	(Byte & 0x40 ? '1' : ' '), \
	(Byte & 0x20 ? '1' : ' '), \
	(Byte & 0x10 ? '1' : ' '), \
	(Byte & 0x08 ? '1' : ' '), \
	(Byte & 0x04 ? '1' : ' '), \
	(Byte & 0x02 ? '1' : ' '), \
	(Byte & 0x01 ? '1' : ' ')

//Beginning of Auto generated function prototypes by Atmel Studio
int sumBytes(uint8_t* bytes, uint8_t parsNo);
void writeServo(uint8_t pcktID, uint8_t pcktCmnd, uint8_t* pcktPars, uint8_t parsNo);
void writeServo(uint8_t pcktID, uint8_t pcktCmnd, uint8_t pcktPar);
void moveSpeed(uint8_t id, int Position, int Speed);
void setServo(uint8_t mode, int rotate);
void rotateOn(int r);
void rotateOff(int r);
void downOn();
void downOff();
void upOn();
void upOff();
void printBuffer();
void printDataLCD();
void printBufferLCD();
void printSerial(String title, int value);
void setID(uint8_t newID);
void move(uint8_t id, int Position);
void reset(uint8_t id);
void reset();
void setMode(uint8_t id, uint8_t mode);
void setModeWheel(uint8_t id);
void setModeJoint(uint8_t id);
void setModeMultiTurn(uint8_t id);
int getData(uint8_t id, uint8_t ctrlData);
void turn(uint8_t id, bool side, int Speed);
void setMaxTorque(uint8_t id, int MaxTorque);
void setTorqueEnable(uint8_t id, bool status);
void readServo(uint8_t pcktID, uint8_t pcktCmnd, uint8_t resLength);
void readServo(uint8_t pcktID, uint8_t pcktCmnd);
//End of Auto generated function prototypes by Atmel Studio

#endif /* GENERAL_H_ */
