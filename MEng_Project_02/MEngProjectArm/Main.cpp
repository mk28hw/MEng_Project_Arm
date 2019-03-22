/* 
 * This code is a mess and not everything works, 
 * The code is compatible with Arduino Mega ATmega2560 (it may works with other boards).
 * It uses two hardware serial interfaces:
 *   Serial (Port 0) for monitoring and troubleshooting
 *   Serial 1 (Port 1) for communication with MX-64ARs
 * 
 * Version 0.21 (20/03/2019):
 * 	 Control Buttons(3) added full rotation (1-2-3-4-5) between all the servos with using [motorSelect] button
 * 	 Tide up of the Code and moved the MX-64AR defines as a separate header file MX-64AR.h
 * Version 0.2:
 * 	 Control Buttons(3) for adjusting the servos: [motorSelect][[down][up]
 * 
 * Date: 20/03/2019 
 */

 /*Begining of Auto generated code by Atmel studio */
 #include <Arduino.h>
 #include <avr/io.h>
 #include <avr/interrupt.h>
 /*End of auto generated code by Atmel studio */
 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* Local Files Includes */
#include "MX-64AR.h"

//Beginning of Auto generated function prototypes by Atmel Studio
int sumBytes(byte* bytes, byte parsNo);
void writeServo(byte pcktID, byte pcktCmnd, byte* pcktPars, byte parsNo);
void writeServo(byte pcktID, byte pcktCmnd, byte pcktPar);
void moveSpeed(unsigned char id, int Position, int Speed);
void setServo(unsigned char mode, int rotate);
void rotateOn(int r);
void rotateOff(int r);
void downOn();
void downOff();
void upOn();
void upOff();
void printBuffer();
void printDataLCD();
void printBufferLCD();
void setID(byte newID);
void move(unsigned char id, int Position);
void reset(unsigned char id);
void reset();
void setMode(unsigned char id, unsigned char mode);
void setModeWheel(unsigned char id);
void setModeJoint(unsigned char id);
void setModeMultiTurn(unsigned char id);
void turn(byte id, bool side, int Speed);
void setMaxTorque(unsigned char id, int MaxTorque);
void setTorqueEnable(unsigned char id, bool status);
void readServo(byte pcktID, byte pcktCmnd, byte resLength);
void readServo(byte pcktID, byte pcktCmnd);
//End of Auto generated function prototypes by Atmel Studio

/* General Defines */
#define OFF 0
#define ON 1
#define NO 0
#define YES 1
#define CCW 0
#define CW 1
#define LEFT 0
#define RIGHT 1

/* LCD Related Defines */
#define LCD_ADDRESS 0x27
#define LCD_ROWS 20
#define LCD_COLS 4
#define LCD_COL1 4
#define LCD_COL2 10
/* LCD Characters. Use (char)CH_XXX to display */
#define CH_ARL 0b01111111
#define CH_ARR 0b01111110
#define CH_DEG 0b11011111

/* Setting both of these two pins to: (a) HIGH - enable TX and disable RX, (b) LOW - enable RX and disable TX */
#define RS485_RX_EN_PIN 2 // this is actually disable pin (not enable) for RX
#define RS485_TX_EN_PIN 3 // enable pin for TX

#define RS485_TX_ON \
digitalWrite(RS485_RX_EN_PIN, HIGH);	/* Notify max485 transceiver to accept tx */ \
digitalWrite(RS485_TX_EN_PIN, HIGH);	/* Notify max485 transceiver to accept tx */ \
delay(1);								/* Allow this to take effect */ \
Serial1.write(MX_START);				/* These 2 bytes are 'start message' */ \
Serial1.write(MX_START);

#define RS485_RX_ON \
Serial1.flush(); \
digitalWrite(RS485_RX_EN_PIN, LOW);		/* Notify MAX485 transceiver to receive */ \
digitalWrite(RS485_TX_EN_PIN, LOW);		/* Notify MAX485 transceiver to receive */ \
delay(1);

#define ARM_ID4_ANGLE_MIN 740	// exactly 752 = 65 degrees
#define ARM_ID4_ANGLE_MAX 1700	// exactly 1762 = 157 degrees

#define ARM_ID5_ANGLE_MIN 740	// exactly 749 = 65 degrees
#define ARM_ID5_ANGLE_MAX 1700	// exactly 1744 = 155 degrees
	
#define BUTTON_1_PRESSED (PINB & (1<<PINB0)) // PIN 53
#define BUTTON_2_PRESSED (PINB & (1<<PINB1)) // PIN 52
#define BUTTON_3_PRESSED (PINB & (1<<PINB2)) // PIN 51
#define LED_ON PORTB |= (1<<PINB7)
#define LED_OFF PORTB &= ~(1<<PINB7)
#define LED_TOGGLE PINB |= (1<<PINB7)
#define PIN53_PRESSED (PINB & (1<<PINB0))
#define PIN52_PRESSED (PINB & (1<<PINB1))

#define TIME_OUT 10

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
	(byte & 0x80 ? '1' : ' '), \
	(byte & 0x40 ? '1' : ' '), \
	(byte & 0x20 ? '1' : ' '), \
	(byte & 0x10 ? '1' : ' '), \
	(byte & 0x08 ? '1' : ' '), \
	(byte & 0x04 ? '1' : ' '), \
	(byte & 0x02 ? '1' : ' '), \
	(byte & 0x01 ? '1' : ' ')
#define MAX_TORQUE 0x3FF
//-----------------------------------------------------------------
int position_old = 0;
int rotations = 0;
byte id = 5;
int angle = 1500;
int currPos = angle;
int currSpeed;
byte error_byte_old;
int error_counter = 0;
int cycle_counter = 0;
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_ROWS, LCD_COLS);  // set the LCD address to 0x27 for a 20 chars and 4 line display

/* LCD Helping to print Function */ 
void printLCD(byte col, byte row, int value, byte padding) {
	char buffer[padding];
	char tmp[5];
	sprintf(tmp, "%%%dd", padding);
	sprintf(buffer, tmp, value);
	lcd.setCursor(col, row);
	lcd.print(buffer);
}
/* Setup the Switches (Pin Change Interrupts) */
void setupSwitches() {
	DDRB = 0xFF;
	DDRB &= ~((1<<DDB0) | (1<<DDB1) | (1<<DDB2)); // clear DDB0 and DDB4 bits in DDRB. PIN B0 and B4 is set to INPUT (BUTTON)
	/* Pin Change Mask Register 0. Enable Interrupt on PIN 19, 20 and 21 */
	PCMSK0 |= (1<<PCINT0) | (1<<PCINT1) | (1<<PCINT2);  // set Pin Change Mask 0 bit PCINT0.
	PCICR  |= (1<<PCIE0); // Pin Change Interrupt Control Register. Activate PCINT7:0
	sei(); // enable global interrupts
}

/*  Buttons Interrupts */
ISR(PCINT0_vect) {
	if (BUTTON_1_PRESSED) { // Speed CHange
		LED_ON;
		//setEndless(id, OFF);
		//setMaxTorque(id,1023);
		while (BUTTON_1_PRESSED) { 
			moveSpeed(id, currPos-1, 20); 
			//move(id, currPos<ARM_ID5_ANGLE_MAX ? currPos-1 : ARM_ID5_ANGLE_MAX);
			delay(200);
		}
		LED_OFF;
	}
	if (BUTTON_2_PRESSED) {
		LED_ON;
		while (BUTTON_2_PRESSED) { 
			moveSpeed(id, currPos+1, 20);
			//move(id, currPos<ARM_ID5_ANGLE_MAX ? currPos+1 : ARM_ID5_ANGLE_MAX);
			delay(200);
		}
		LED_OFF;
	}
	if (BUTTON_3_PRESSED) { // this switch is not working all the time
		LED_TOGGLE;
		delay(20);
		while (BUTTON_3_PRESSED) delay(1);
		id = id > 4 ? 1 : id + 1;
		if (id>3) setModeJoint(id);
		else if (id==3) setModeMultiTurn(id);
		else setModeWheel(id);
	}
}

/* New shorter functions - START */

void writeServo(byte pcktID, byte pcktCmnd, byte* pcktPars, byte parsNo) {
	char Checksum = ~lowByte(pcktID + parsNo + 3 + MX_INSTRUCTION_WRITE_DATA + pcktCmnd + sumBytes(pcktPars, parsNo));

	RS485_TX_ON
	Serial1.write(pcktID);						// Servo ID
	Serial1.write(parsNo + 3);					// Length of message (number of Parameters + 3 (1 Command + 2))
	Serial1.write(MX_INSTRUCTION_WRITE_DATA);	// Write message type (write)
	Serial1.write(pcktCmnd);					// Write Command
	Serial1.write(pcktPars, parsNo);			// Write Parameters
	Serial1.write(Checksum);					// Write Checksum
	RS485_RX_ON
}

/* Write to Servo a command with only one Parameter */
void writeServo(byte pcktID, byte pcktCmnd, byte pcktPar) { writeServo(pcktID, pcktCmnd, &pcktPar, 1); }

/* Reset the Servo with given ID */
void resetServo(unsigned char id) {
	char Checksum = ~lowByte(id + MX_INSTRUCTION_RESET_LENGTH + MX_INSTRUCTION_RESET);

	RS485_TX_ON
	Serial1.write(id);							// ID is 0xFE, which broadcast mode (all Servos hear this message)
	Serial1.write(MX_INSTRUCTION_RESET_LENGTH);	// Length of message (number of Parameters + 3 (1 Command + 2))
	Serial1.write(MX_INSTRUCTION_RESET);		// Write message type (write)
	Serial1.write(Checksum);					// Write Checksum
	RS485_RX_ON
}

/* Reset All Servos */
void resetServo() { resetServo(MX_ALL_SERVOS); }
	
/* Request message from Servo using a command with a Parameter */
void readServo(byte pcktID, byte pcktCmnd, byte resLength) {
	char Checksum = ~lowByte(pcktID + 4 + MX_INSTRUCTION_READ_DATA + pcktCmnd + resLength);
	RS485_TX_ON
	Serial1.write(pcktID);					// Servo ID
	Serial1.write(4);						// Length of message (number of Parameters + 3 (1 Command + 2))
	Serial1.write(MX_INSTRUCTION_READ_DATA);// Write message type (read)
	Serial1.write(pcktCmnd);				// Write Command
	Serial1.write(resLength);				// Response Length (usually 1 but for some might be 2 or more (more data))
	Serial1.write(Checksum);				// Write Checksum
	RS485_RX_ON
}
void readServo(byte pcktID, byte pcktCmnd) { readServo(pcktID, pcktCmnd, 1); }
	
void moveSpeed(byte id, int Position, int Speed) {
	if (id == 4) {
		Position = Position > ARM_ID4_ANGLE_MAX 
			? ARM_ID4_ANGLE_MAX : Position < ARM_ID4_ANGLE_MIN 
			? ARM_ID4_ANGLE_MIN : Position;
	} else if (id == 5) {
		Position = Position > ARM_ID5_ANGLE_MAX 
			? ARM_ID5_ANGLE_MAX : Position < ARM_ID5_ANGLE_MIN 
			? ARM_ID5_ANGLE_MIN : Position;
	}
  byte parsNo = 4;
  byte pcktPars[parsNo] = {Position, Position >> 8, Speed, Speed >> 8};
  writeServo(id, MX_GOAL_POSITION_L, pcktPars, parsNo);
  currPos = Position;
}
/* 0 ~ 4095 => 0 ~ 360° */
void move(unsigned char id, int Position) {
	byte parsNo = 2;
	byte pcktPars[parsNo] = {Position, Position >> 8};
	writeServo(id, MX_GOAL_POSITION_L, pcktPars, parsNo);
	currPos = Position;
}
/*	Joint / Multi-Turn Mode: 0 ~ 1023 (0x3FF), 1 => 0.114rpm
		0 => Maximum rpm is used without controlling the speed
		0< ~ 1023 (0x3FF) => 0 ~ 117.07rpm 
	Wheel Mode: 0 ~ 2047 (0x7FF), 1 => 0.114rpm
		0 ~ 1023 (0x3FF) => 0 ~ 117.07rpm CCW 
		1024 ~ 2047 (0x7FF) => 0 ~ 117.07rpm CW */
void turn(byte id, bool side, int Speed) {
	byte parsNo = 2;
	byte pcktPars[parsNo] = {Speed, (Speed >> 8) + side ? 0 : 4};
	writeServo(id, MX_MOVING_SPEED_L, pcktPars, parsNo);
	currSpeed = Speed;
	// Return the read error
} 

void setTorqueEnable(unsigned char id, bool status) { writeServo(id, MX_TORQUE_ENABLE, status); }

/*
Goal Position: 0 ~ 4095 (0x000 ~ 0xFFF) = 0° ~ 360°, 1 ~ 0.088° 
Moving Speed:	0~2047 (0x000~0x7FF), (Joint Mode), 1 ~ 0.114rpm
				0 -> uses max rpm, no speed control
				1023 (0x3FF) -> ~ 117.07rpm
				0~1023 (0x000~0x3FF) -> CCW direction
				1024~2047 (0x400~0x7FF) -> CW direction
*/
void setMaxTorque(unsigned char id, int MaxTorque) {
	byte parsNo = 2;
	byte pcktPars[parsNo] = { MaxTorque, MaxTorque >> 8};
	writeServo(id, MX_MAX_TORQUE_L, pcktPars, parsNo);
}

void setTorqueLimit(unsigned char id, int TorqueLimit) {
	byte parsNo = 2;
	byte pcktPars[parsNo] = { TorqueLimit, TorqueLimit >> 8};
	writeServo(id, MX_TORQUE_LIMIT_L, pcktPars, parsNo);
}
void setID(byte newID) { writeServo(MX_ALL_SERVOS, MX_ID, newID); }
	
/* Set the servo with given ID to endless (wheel mode) or clear it (joint or multi-turn mode) */   
void setMode(unsigned char id, unsigned char mode) {
	byte parsNo = 4;
	if (mode == 1) {			// Wheel Mode
		byte pcktPars[parsNo] = {0, 0, 0, 0};
		writeServo(id, MX_CW_ANGLE_LIMIT_L, pcktPars, parsNo);
	} else if (mode == 2) {		// Multi-turn Mode (both 4095)
		byte pcktPars[parsNo] = {0xFF, 0x0F, 0xFF, 0x0F};
		writeServo(id, MX_CW_ANGLE_LIMIT_L, pcktPars, parsNo);
	} else {					// Joint Mode
		byte pcktPars[parsNo] = {ARM_ID5_ANGLE_MIN, ARM_ID5_ANGLE_MIN >> 8, ARM_ID5_ANGLE_MAX, ARM_ID5_ANGLE_MAX >> 8};
		writeServo(id, MX_CW_ANGLE_LIMIT_L, pcktPars, parsNo);
	}
}
/* Set the servo with given ID to endless => wheel mode */ 
void setModeWheel(unsigned char id) { setMode(id, MX_MODE_WHEEL); }
void setModeJoint(unsigned char id) { setMode(id, MX_MODE_JOINT); }
void setModeMultiTurn(unsigned char id) { setMode(id, MX_MODE_MULTI); }
		
/* New shorter functions - END */

/* Some old (but shortened) function from previous dissertation */
void setServo(unsigned char mode, int rotate) {
	setMaxTorque(0, 1023);
	setMode(0, mode);
	turn(0, rotate, 100);
}
void rotateOn(int r) { setServo(MX_MODE_WHEEL, r); }	
void rotateOff(int r) { setServo(MX_MODE_JOINT, r); }	
void downOn() { setServo(MX_MODE_WHEEL, LEFT); }
void downOff() { setServo(MX_MODE_JOINT, LEFT); }
void upOn() { setServo(MX_MODE_WHEEL, RIGHT); }
void upOff() { setServo(MX_MODE_JOINT, RIGHT); }

/* Servo error decoding function */ 
String* error_decode(byte error_code) {
	String errors[8] = { 
		(error_code & (1<0) ? "Input Voltage" : ""),	// 1
		(error_code & (1<1) ? "Angle Limit" : ""),	// 2
		(error_code & (1<2) ? "Overheating" : ""),	// 4
		(error_code & (1<3) ? "Range" : ""),			// 8
		(error_code & (1<4) ? "Checksum" : ""),		// 16
		(error_code & (1<5) ? "Overload" : ""),		// 32
		(error_code & (1<6) ? "Instruction" : "") };	// 64
	return errors;
}
/* 
 *  |0xFF|0xFF|ID|LENGTH|INSTRUCTION|PARAM_1|...|PARAM_N|CHECKSUM
 */
void printBuffer() {
	//delay(20);
	//byte start_1, start_2, servo_id, msg_length, er_byte, chck_sum;
	Serial.print("## Start: ");
	Serial.print(Serial1.available());
	Serial.println(" ####################");
	while(Serial1.available()){
		
		Serial.print(Serial1.available());
		Serial.print(", ");
		Serial.println(Serial1.read());
		delay(1);
	}
	Serial.println("## End #####################");
}

void printSerial(String title, int value) {
	Serial.print(title);
	Serial.print(": ");
	Serial.println(value);
}
void printDataLCD() {
	delay(10);
	int data;
	bool msgStarted;
	byte servoID, msgLength, error_byte;
	int position;
	int speed;
	int load;
	int voltage;
	int temperature;
	int registered;
	int moving;
	int lock;
	int punch;
	int current;
	bool speedDirection, loadDirection;
	byte i = 0;
	int available = Serial1.available();
	if (available > 0){
		msgStarted = Serial1.read() == 0xFF ?  YES : NO;
		if (msgStarted && (Serial1.read() == 0xFF)) {
			servoID = Serial1.read(); // servo ID
			msgLength = Serial1.read(); // msg Length
			error_byte = Serial1.read();
			error_byte_old = error_byte ? error_byte : error_byte_old;
			
			lcd.setCursor(7,0);
			if (error_byte || error_counter) {	
				error_counter = error_counter > 10 ? 0 : error_counter + 1;
				lcd.print("ERROR:");
				//char buff[7];
				//sprintf(buff, "BYTE_TO_BINARY_PATTERN", BYTE_TO_BINARY(error_byte));
				lcd.print(error_byte_old);
				lcd.print("   ");
			} else { // No Error :)
				lcd.print("          ");
				if (msgLength > 2) {
					position = Serial1.read();
					position = (Serial1.read() * 255) + position;
					rotations = position_old > position ? rotations + 1 : rotations;
					position_old = position;
								
					//Serial.println(position);
					speed = Serial1.read();
					speed = (Serial1.read() * 255) + speed;
					//Serial.println(speed);
					load = Serial1.read();
					load = (Serial1.read() * 255) + load;
					voltage = Serial1.read();
					temperature = Serial1.read();
					registered = Serial1.read();
					moving = Serial1.read();
					lock = Serial1.read();
					punch = Serial1.read();
					punch = (Serial1.read() * 255) + punch;
					current = Serial1.read();
					current = (Serial1.read() * 255) + current;
					//Serial.println(load);
					if (speed > 0x3FF) {
						speedDirection = CW;
						speed = speed - 0x400;
						} else {
						speedDirection = CCW;
					}
					if (load > 0x3FF) {
						loadDirection = CW;
						load = load - 0x400;
						} else {
						loadDirection = CCW;
					}

					//Serial.println("================");
					//lcd.clear();
					//char buffer[16];
					//sprintf(buffer, "Servo ID: %d", servoID);
					printLCD(LCD_COL1, 0, servoID, 1);
					printLCD(LCD_COL1, 1, position * MX_PRESENT_POSITION_DEGREE, 3);
					lcd.print((char)CH_DEG);
					printLCD(LCD_COL2, 1, position, 4);
					printLCD(LCD_COL2+5, 1, currPos, 4);
					// 			lcd.setCursor(LCD_COL2 ,1);
					// 			lcd.print((int)(currPos));
					//lcd.setCursor(10,1);
					//lcd.print(rotations);
					printLCD(LCD_COL1, 2, speed, 3);
					lcd.print(speedDirection ? (char)CH_ARR : (char)CH_ARL);
					printLCD(LCD_COL1, 3, load, 3);
					//lcd.setCursor(LCD_COL1, 3);
					//lcd.print(load);
					lcd.print(loadDirection ? (char)CH_ARR : (char)CH_ARL);
					//printLCD3(LCD_COL2, 3, (45*(current-2048)));
				}
			}
			while(Serial1.available() > 0) {
				//Serial.println(Serial1.read());
				Serial1.read();
			}
			
			delay(1);
			cycle_counter = cycle_counter > 9999 ? 0 : cycle_counter + 1;
			printLCD(16, 0, cycle_counter, 4);
		}
	}
}
void printBufferLCD() {
	delay(10);
	int data;
	bool msgStarted;
	int reading; 
	
	if (Serial1.available()){
		msgStarted = Serial1.read() == 0xFF ?  YES : NO;
		if (msgStarted && Serial1.read()) {
			Serial1.read(); // servo ID
			Serial1.read(); // msg Length
			Serial1.read(); // zero
			reading = Serial1.read();
			reading = Serial1.available() ? (Serial1.read() * 255) + reading : reading;
			while(Serial1.available()) { Serial1.read(); }
			lcd.clear();
			lcd.setCursor(2,1);
			lcd.print(reading*0.088);
			delay(1);
		}
	}
}

/* Extra functions */
int sumBytes(byte* bytes, byte parsNo) {
	int sum = 0;
	for (byte i=0; i<parsNo; i++) sum +=bytes[i];
	return sum;
}

/************************************************************************/
/* SETUP                                                                */
/************************************************************************/
void setup() {
	/* Main Setup */
	setupSwitches();					// Setup the arm control switches
	lcd.init();							// initialize the lcd
	lcd.backlight();
	lcd.clear();
	pinMode(RS485_RX_EN_PIN, OUTPUT);
	pinMode(RS485_TX_EN_PIN, OUTPUT);
	Serial.begin(57600);
	Serial1.begin(57143);
	Serial1.flush();
	/* Angle Limits Setup (Servo Modes) */
	setModeWheel(1);
	setModeWheel(2);
	setModeMultiTurn(3);
	setModeJoint(4);
	setModeJoint(5);
	/* Enable Toques for Joints (4 and 5) */
	for (unsigned char i=1; i<6; i++) {
		setTorqueLimit(i, MAX_TORQUE);
	}
	/* LCD Setup */
	lcd.setCursor(0,0);
	lcd.print("sID:");
	lcd.setCursor(0,1);
	lcd.print("deg:");
	lcd.setCursor(0,2);
	lcd.print("rpm:");
	lcd.setCursor(0,3);
	lcd.print("ld%:");

	readServo(id, 0x18, 8);
	printBuffer();
	delay(2);
}

/************************************************************************/
/* LOOP                                                                 */
/************************************************************************/
void loop() {

	readServo(id, 0x24, 34);
	printDataLCD();
	//printDataLCD();
	//delay(200);
}