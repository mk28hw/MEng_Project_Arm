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

#endif
