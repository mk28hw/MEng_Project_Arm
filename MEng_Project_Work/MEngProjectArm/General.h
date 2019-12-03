/*
 * General.h
 *
 * Created: 02/04/2019 11:11:56
 * Author: Marek Kujawa
 */ 

/* Integer Ranges
 *
 * uint8_t	= 0 to                        255	[0x00               to               0xFF]
 * uint16_t = 0 to                     65,535	[0x0000             to             0xFFFF]
 * uint32_t = 0 to              4,294,967,295	[0x00000000         to         0xFFFFFFFF]
 * uint64_t = 0 to 18,446,744,073,709,551,615	[0x0000000000000000 to 0xFFFFFFFFFFFFFFFF]
 *
 * int8_t	=                       -128 to                       127
 * int16_t	=                    -32,768 to                    32,767
 * int32_t	=             -2,147,483,648 to             2,147,483,647
 * int64_t	= -9,223,372,036,854,775,808 to 9,223,372,036,854,775,807
 */

#ifndef GENERAL_H
#define GENERAL_H

String padNumber(int value, uint8_t padding);
int combineBytes(uint8_t byte_L, uint8_t byte_H);
int sumBytes(uint8_t* bytes, uint8_t parsNo);
String combineToCSV(int* data, uint8_t number);
void dumpDataToSerial(int* data, uint8_t number);
uint32_t timeIt(void (*functionToBeTimed) ());
void printSerial(String title, int value);
	
/* General Defines */
#define OFF 0
#define ON 1
#define NO 0
#define YES 1
#define CCW 0
#define CW 1
#define LEFT 0
#define RIGHT 1
#define UP 1
#define DOWN 0

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

#endif /* GENERAL_H_ */
//
