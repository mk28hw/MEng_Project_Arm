/*
 * This code is a mess and not everything works,
 * The code is compatible with Arduino Mega ATmega2560 (it may works with other boards).
 * It uses two hardware serial interfaces:
 *   Serial (Port 0) for monitoring and troubleshooting
 *   Serial 1 (Port 1) for communication with MX-64ARs
 
 * Last Commit:	e608c66f07eb9e3d64fcf5b4aaa1c5a76aa0a600
 * TODO:
 *   1. dump data to serial in csv format for the main data (positions, speeds, loads) 
 *   2. Integral and Differential self adjusting balance (by controlling servo 1 and 2) using load reading from servo 4 and 5

 * Version 0.35 (11/04/2019):
 *   1. turn number when changing ID [DONE]
 *   2. stopping servos (1 and 2) when button 1 pressed [DONE]
 *   3. getting data (load) from ID 4 and 5 at the same time [DONE]
 *   4. rearrange LCD layout (2nd and 3rd row last remove) [DONE]
 *   5. proportional self adjusting balance (by controlling servo 1 and 2) using load reading from servo 4 and 5 [DONE]
 *   6. dump data to serial in csv format for getData function.
 *   7. Adding Manual and Automatic Mode. [DONE]
 *		Long press (1s) [select] button to toggle between Automatic and Manual. [DONE]
 *      It indicates it on LCD (top right corner). [DONE]
 *      Once change from Automatic to Manual both servos 1 and 2 stop. [DONE]
 * Version 0.32 (01/04/2019) - UNSTABLE:
 * 	 Changed global variables to global stucts.
 *   Added getData function for requesting and capturing the data (message) returned from (by) the requested servo.
 * Version 0.3 (22/03/2019):
 * 	 The code got optimized (size-wise), from original 683 lines it got reduced to 542 (excluding MX-64AR.h file but including
 *	 lots of comments). This translates to over 20% reduction. Button [motorSelect] was fixed and now it does not skip servos.
 * Version 0.21 (20/03/2019):
 * 	 Control Buttons(3) added full rotation (1-2-3-4-5) between all the servos with using [motorSelect] button
 * 	 Tide up of the Code and moved the MX-64AR defines as a separate header file MX-64AR.h
 * Version 0.2:
 * 	 Control Buttons(3) for adjusting the servos: [motorSelect][[down][up]
 *
 * Date: 20/03/2019
 */

/* Beginning of Auto generated code by Atmel studio */
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/* End of auto generated code by Atmel studio */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* Local Files Includes */
#include "General.h"
#include "MX-64AR.h"
#include "Arm.h"
#include "LCD.h"

/* Setting both of these two pins to: (a) HIGH - enable TX and disable RX, (b) LOW - enable RX and disable TX */
#define RS485_RX_EN_PIN 2 // this is actually disable pin (not enable) for RX
#define RS485_TX_EN_PIN 3 // enable pin for TX

#define RS485_TX_ON \
while(Serial1.available()) { Serial1.read(); } \
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

#define BUTTON_1_PRESSED (PINB & (1<<PINB0)) // PIN 53
#define BUTTON_2_PRESSED (PINB & (1<<PINB1)) // PIN 52
#define BUTTON_3_PRESSED (PINB & (1<<PINB2)) // PIN 51
#define LED_ON PORTB |= (1<<PINB7)
#define LED_OFF PORTB &= ~(1<<PINB7)
#define LED_TOGGLE PINB |= (1<<PINB7)
#define PIN53_PRESSED (PINB & (1<<PINB0))
#define PIN52_PRESSED (PINB & (1<<PINB1))

#define TIME_OUT 10

#define MAX_TORQUE 0x3FF


//-----------------------------------------------------------------
int position_old = 0;
uint8_t id = 5;
uint8_t servoID_old;
int angle = 1500;
int currPos = angle;
int currSpeed;
uint8_t lastButtonPressed = 0;
bool buttonsFlip[3] = {0, 0, 0};

Arm arm;

uint8_t error_byte_old;
int error_counter = 0;
int cycle_counter = 0;
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);  // set the LCD address to 0x27 for a 20 chars and 4 line display
bool serialWriting = NO;
bool serialReading = NO;

uint16_t load_4, load_5;
bool load_4_dir, load_5_dir;
bool autoBalanceOn = OFF;

/* LCD Helping to print Function */
void printLCD(uint8_t col, uint8_t row, int value, uint8_t padding) {
	char buffer[padding];
	char tmp[5];
	value = (value < pow(10, padding)) ? value : (pow(10, padding)) - 1;
	sprintf(tmp, "%%%dd", padding);
	sprintf(buffer, tmp, value);
	lcd.setCursor(col, row);
	lcd.print(buffer);
}
/* LCD Helping to print Function */
void printLCD(uint8_t col, uint8_t row, const char* text) {
	lcd.setCursor(col, row);
	lcd.print(text);
}
/* Get Real Value from Raw Value. Works for Speed and Load */ 
int getRealValue(int rawValue) { return (rawValue > 0x3FF) ? rawValue - 0x400 : rawValue; }
/* Get Direction from Raw Value. Works for Speed and Load */ 
bool getDirection(int rawValue) { return (rawValue > 0x3FF) ? CW : CCW; }

String padNumber(int value, uint8_t padding) {
	uint8_t response;
	char buffer[padding];
	char tmp[5];
	/* if the value is greater than number of digits it will be maxed out, e.g. pad=2, val=200 => val=99 */
	//value = (value < pow(10, padding)) ? value : (pow(10, padding)) - 1;
	response = sprintf(tmp, "%%%dd", padding);
	response = sprintf(buffer, tmp, value);
	return buffer;
}

int combineBytes(uint8_t byte_1, uint8_t byte_2) { return byte_1 + (byte_2<<8); }

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
	int speed;
	uint8_t id = arm.id;
	uint16_t i;
	if (BUTTON_1_PRESSED) { // Speed CHange
		LED_ON;
		//setEndless(id, OFF);
		//setMaxTorque(id,1023);
		//while (BUTTON_1_PRESSED) delay(1);
		while (BUTTON_1_PRESSED) {
			arm.servos[id].position--;
			if (id < 3) {
				
				speed = (lastButtonPressed == 1) 
					? buttonsFlip[1] 
						? 0
						: arm.servos[id].speed
					: arm.servos[id].speed;
			} else {
				speed = arm.servos[id].speed;
			}
			moveSpeed(id, arm.servos[id].position, speed);
			//move(id, currPos<ARM_ID5_ANGLE_MAX ? currPos-1 : ARM_ID5_ANGLE_MAX);
			delay(200);
			
		}
		buttonsFlip[1] = !buttonsFlip[1];
		lastButtonPressed = 1;
		LED_OFF;
	}
	if (BUTTON_2_PRESSED) {
		LED_ON;
		//while (BUTTON_2_PRESSED) delay(1);
		while (BUTTON_2_PRESSED) {
			arm.servos[id].position++;
			if (id < 3) {
				speed = (lastButtonPressed == 2) 
					? buttonsFlip[2]
						? 0x0400
						: (0x0400 + arm.servos[id].speed) 
					: (0x0400 + arm.servos[id].speed);	
			} else {
				speed = arm.servos[id].speed;
			}
			moveSpeed(id, arm.servos[id].position, speed);
			//move(id, currPos<ARM_ID5_ANGLE_MAX ? currPos+1 : ARM_ID5_ANGLE_MAX);
			delay(200);
			
		}
		buttonsFlip[2] = !buttonsFlip[2];
		lastButtonPressed = 2;
		LED_OFF;
	}
	if (BUTTON_3_PRESSED) {
		LED_ON;
		i = 0;
		while (BUTTON_3_PRESSED) {
			delay(1);
			i++;
		} 
		if (i>1000) { 
			autoBalanceOn = !autoBalanceOn;
		} else {
			if (id < 3) { moveSpeed(id, arm.servos[id].position, 0); }
			id = id > 4 ? 1 : id + 1;
			arm.id = id;
			if (id>3) setModeJoint(id);
			else if (id==3) setModeMultiTurn(id);
			else setModeWheel(id);
		}
		delay(20);
		/* Stop Servo 1 or 2 if you press button 3 */

		buttonsFlip[3] = !buttonsFlip[3];
		lastButtonPressed = 3;
		LED_OFF;
	}
}

/* New shorter functions - START */

void writeServo(uint8_t pcktID, uint8_t pcktCmnd, uint8_t* pcktPars, uint8_t parsNo) {
	uint8_t Checksum = ~lowByte(pcktID + parsNo + 3 + MX_INSTRUCTION_WRITE_DATA + pcktCmnd + sumBytes(pcktPars, parsNo));

	//while (serialWriting || serialReading) { delay(1); }
	serialWriting = YES;
	RS485_TX_ON
	Serial1.write(pcktID);						// Servo ID
	Serial1.write(parsNo + 3);					// Length of message (number of Parameters + 3 (1 Command + 2))
	Serial1.write(MX_INSTRUCTION_WRITE_DATA);	// Write message type (write)
	Serial1.write(pcktCmnd);					// Write Command
	Serial1.write(pcktPars, parsNo);			// Write Parameters
	Serial1.write(Checksum);					// Write Checksum
	RS485_RX_ON
	serialWriting = NO;
}

/* Write to Servo a command with only one Parameter */
void writeServo(uint8_t pcktID, uint8_t pcktCmnd, uint8_t pcktPar) { writeServo(pcktID, pcktCmnd, &pcktPar, 1); }

/* Reset the Servo with given ID */
void resetServo(uint8_t id) {
	uint8_t Checksum = ~lowByte(id + MX_INSTRUCTION_RESET_LENGTH + MX_INSTRUCTION_RESET);
	
	//while (serialWriting || serialReading) { delay(1); }
	serialWriting = YES;
	RS485_TX_ON
	Serial1.write(id);							// ID is 0xFE, which broadcast mode (all Servos hear this message)
	Serial1.write(MX_INSTRUCTION_RESET_LENGTH);	// Length of message (number of Parameters + 3 (1 Command + 2))
	Serial1.write(MX_INSTRUCTION_RESET);		// Write message type (write)
	Serial1.write(Checksum);					// Write Checksum
	RS485_RX_ON
	serialWriting = NO;
}

/* Reset All Servos */
void resetServo() { resetServo(MX_ALL_SERVOS); }

/* Request message from Servo using a command with a Parameter */
void readServo(uint8_t pcktID, uint8_t pcktCmnd, uint8_t resLength) {
	uint8_t Checksum = ~lowByte(pcktID + 4 + MX_INSTRUCTION_READ_DATA + pcktCmnd + resLength);
	
	//while (serialWriting || serialReading) { delay(1); }
	serialWriting = YES;
	RS485_TX_ON
	Serial1.write(pcktID);					// Servo ID
	Serial1.write(4);						// Length of message (number of Parameters + 3 (1 Command + 2))
	Serial1.write(MX_INSTRUCTION_READ_DATA);// Write message type (read)
	Serial1.write(pcktCmnd);				// Write Command
	Serial1.write(resLength);				// Response Length (usually 1 but for some might be 2 or more (more data))
	Serial1.write(Checksum);				// Write Checksum
	RS485_RX_ON
	serialWriting = NO;
}
void readServo(uint8_t pcktID, uint8_t pcktCmnd) { readServo(pcktID, pcktCmnd, 1); }


int checkPosition(uint8_t id, int Position) {
    if (id < 3) { // Wheel Mode Unlimited
	        return Position > ARM_ID1_ANGLE_MAX
	        ? ARM_ID1_ANGLE_MAX
	        : Position < ARM_ID1_ANGLE_MIN
				? ARM_ID1_ANGLE_MIN
				: Position;
	} else if (id == 3) { // Multi-Turn up to 7 turns each direction
		/* -28672 - 28672 */
        return Position > ARM_ID3_ANGLE_MAX
            ? ARM_ID3_ANGLE_MAX
            : Position < ARM_ID3_ANGLE_MIN
                     ? ARM_ID3_ANGLE_MIN
                     : Position;
    } else if (id == 4) { // Joint Mode up one turn
		/* 0 - 4095 */
        return Position > ARM_ID4_ANGLE_MAX
            ? ARM_ID4_ANGLE_MAX
            : Position < ARM_ID4_ANGLE_MIN
                     ? ARM_ID4_ANGLE_MIN
                     : Position;
    } else if (id == 5) { // Joint Mode up one turn
		/* 0 - 4095 */
        return Position > ARM_ID5_ANGLE_MAX
            ? ARM_ID5_ANGLE_MAX
            : Position < ARM_ID5_ANGLE_MIN
                     ? ARM_ID5_ANGLE_MIN
                     : Position;
    } else {
        return -1;
    }
}
/*	Joint Mode:
	Multi-Turn: -28672 ~ 28672 */
void moveSpeed(uint8_t id, int Position, int Speed) {
    Position = checkPosition(id, Position);
    uint8_t parsNo = 4;
    uint8_t pcktPars[parsNo] = {Position, Position >> 8, Speed, Speed >> 8};
    writeServo(id, MX_GOAL_POSITION_L, pcktPars, parsNo); 
    currPos = Position;
    arm.servos[id].position = Position;
    //arm.servos[id].speed = Speed;
}
/* 0 ~ 4095 => 0 ~ 360° */
void move(uint8_t id, int Position) {
    Position = checkPosition(id, Position);
	uint8_t parsNo = 2;
	uint8_t pcktPars[parsNo] = {Position, Position >> 8};
	writeServo(id, MX_GOAL_POSITION_L, pcktPars, parsNo);
	currPos = Position;
	arm.servos[id].position = Position;
}
/*	Joint / Multi-Turn Mode: 0 ~ 1023 (0x3FF), 1 => 0.114rpm
		0 => Maximum rpm is used without controlling the speed
		0< ~ 1023 (0x3FF) => 0 ~ 117.07rpm
	Wheel Mode: 0 ~ 2047 (0x7FF), 1 => 0.114rpm
		0 ~ 1023 (0x3FF) => 0 ~ 117.07rpm CCW
		1024 ~ 2047 (0x7FF) => 0 ~ 117.07rpm CW */
void turn(uint8_t id, bool side, int Speed) {
	uint8_t parsNo = 2;
	uint8_t pcktPars[parsNo] = {Speed, (Speed >> 8) + side ? 0 : 4};
	writeServo(id, MX_MOVING_SPEED_L, pcktPars, parsNo);
	currSpeed = Speed;
	//arm.servos[id].speed = Speed;
	// Return the read error
}

void setTorqueEnable(uint8_t id, bool status) { writeServo(id, MX_TORQUE_ENABLE, status); }

/*
Goal Position: 0 ~ 4095 (0x000 ~ 0xFFF) = 0° ~ 360°, 1 ~ 0.088°
Moving Speed:	0~2047 (0x000~0x7FF), (Joint Mode), 1 ~ 0.114rpm
				0 -> uses max rpm, no speed control
				1023 (0x3FF) -> ~ 117.07rpm
				0~1023 (0x000~0x3FF) -> CCW direction
				1024~2047 (0x400~0x7FF) -> CW direction
*/
void setMaxTorque(uint8_t id, int MaxTorque) {
	uint8_t parsNo = 2;
	uint8_t pcktPars[parsNo] = {MaxTorque, MaxTorque >> 8};
	writeServo(id, MX_MAX_TORQUE_L, pcktPars, parsNo);
}

void setTorqueLimit(uint8_t id, int TorqueLimit) {
	uint8_t parsNo = 2;
	uint8_t pcktPars[parsNo] = {TorqueLimit, TorqueLimit >> 8};
	writeServo(id, MX_TORQUE_LIMIT_L, pcktPars, parsNo);
}
void setID(uint8_t newID) { writeServo(MX_ALL_SERVOS, MX_ID, newID); }

/*  Set the servo with given ID to different modes:
        0 - Joint Mode
        1 - Wheel Modes
        2 - Multi-Turn Mode */
void setMode(uint8_t id, uint8_t mode) {
	uint8_t parsNo = 4;
	if (mode == 1) {			// Wheel Mode
		uint8_t pcktPars[parsNo] = {0, 0, 0, 0};
		writeServo(id, MX_CW_ANGLE_LIMIT_L, pcktPars, parsNo);
	} else if (mode == 2) {		// Multi-turn Mode (both 4095)
		uint8_t pcktPars[parsNo] = {0xFF, 0x0F, 0xFF, 0x0F};
		writeServo(id, MX_CW_ANGLE_LIMIT_L, pcktPars, parsNo);
	} else {					// Joint Mode
		uint8_t pcktPars[parsNo] = {ARM_ID5_ANGLE_MIN, ARM_ID5_ANGLE_MIN >> 8, ARM_ID5_ANGLE_MAX, ARM_ID5_ANGLE_MAX >> 8};
		writeServo(id, MX_CW_ANGLE_LIMIT_L, pcktPars, parsNo);
	}
	if (id<6 && mode<3) { arm.servos[id].mode = mode; }
}
/* Set the servo with given ID to endless => wheel mode */
void setModeWheel(uint8_t id) { setMode(id, MX_MODE_WHEEL); }
/* Set the servo with given ID to joint mode (0 ~ 360°) */
void setModeJoint(uint8_t id) { setMode(id, MX_MODE_JOINT); }
/* Set the servo with given ID to multi-turn mode (-(7*360°) ~ +(7*360°)) */
void setModeMultiTurn(uint8_t id) { setMode(id, MX_MODE_MULTI); }

/* New shorter functions - END */

/* Some old (but shortened) function from previous dissertation */
void setServo(uint8_t mode, int rotate) {
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
String* error_decode(uint8_t error_code) {
	String errors[8] = {
		(error_code & (1<0) ? "Vlt" : ""),	// 1
		(error_code & (1<1) ? "Ang" : ""),	// 2
		(error_code & (1<2) ? "oHt" : ""),	// 4
		(error_code & (1<3) ? "Rng" : ""),	// 8
		(error_code & (1<4) ? "Som" : ""),	// 16
		(error_code & (1<5) ? "oLd" : ""),	// 32
		(error_code & (1<6) ? "Ins" : "") };// 64
	for (uint8_t i=0; i<8; i++) {
		printSerial(errors[i], i);
	}
	return errors;
}
/*
 *  |0xFF|0xFF|ID|LENGTH|INSTRUCTION|PARAM_1|...|PARAM_N|CHECKSUM
 */
void printBuffer() {
	//delay(20);
	//uint8_t start_1, start_2, servo_id, msg_length, er_byte, chck_sum;

	Serial.print("## Start: ");
	Serial.print(Serial1.available());
	Serial.println(" ####################");
	serialReading = YES;
	while(Serial1.available()){

		Serial.print(Serial1.available());
		Serial.print(", ");
		Serial.println(Serial1.read());
		delay(1);
	}
	serialReading = NO;
	Serial.println("## End #####################");
}

void printSerial(String title, int value) {
	Serial.print(title);
	Serial.print(": ");
	Serial.println(value);
}
/* Request and Capture data from servo with given ID */
int getData(uint8_t id, uint8_t ctrlData) {
	uint8_t msgByte;
	bool startOne = NO, startTwo = NO, msgStarted, msgOK = NO;
	uint8_t byteCount = 0;
	uint8_t msgId, msgLength, msgError, msgChecksum, Checksum, msgData_1, msgData_2;
	int msgData;
	String line;
	
	readServo(id, ctrlData, 2);
	delay(10);
	serialReading = YES;
	//do { msgByte = Serial1.read(); } while (msgByte != 0xFF);	// 01 : Start 1/2
	msgByte = Serial1.read();
	startOne = msgByte == 0xFF ? YES : NO;
	msgByte = Serial1.read();									// 02 : Start 2/2
	startTwo = msgByte == 0xFF ? YES : NO;
	//while (msgByte == 0xFF) { msgByte = Serial1.read(); }
	msgStarted = startOne && startTwo ? YES : NO;
	if (msgStarted) {
		msgId = Serial1.read();
		line = "ID:" + padNumber(msgId, 6);							//1
		msgLength = Serial1.read();
		line = line + ", MsgLegth:" + padNumber(msgLength, 6);		//2
		msgError = Serial1.read();
		line = line + ", Error:" + padNumber(msgError, 6);			//3
		msgData_1 = Serial1.read();
		msgData_2 = Serial1.read();
		msgData = combineBytes(msgData_1, msgData_2);
		line = line + ", Data:" + padNumber(msgData, 6);			//4
		msgChecksum = Serial1.read();
		line = line + ", MsgCheckSum:" + padNumber(msgChecksum, 6); //5
		Checksum = ~lowByte(msgId + msgLength + msgError + msgData_1 + msgData_2);
		line = line + ", CalCheckSum:" + padNumber(Checksum, 6);	//6
		msgOK = msgChecksum == Checksum ? YES : NO;
	}
	while(Serial1.available()) { msgByte = Serial1.read(); }
	serialReading = NO;
	Serial.println(line);
    /*  Check if the returned data is not corrupted and there are no errors
            Return -255 if the data is corrupted (Checksum error)
            Return -ErrorCode if there is error from servo */
	return msgOK ? msgError ? -msgError : msgData : -255;
}

void printDataLCD() {
	delay(10);
	int data;
	bool msgStarted;
	uint8_t servoID, msgLength, error_byte;
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
	int8_t rotations = 0;
	bool speedDirection, loadDirection;
	uint8_t i = 0;
	int available = Serial1.available();
	if (available > 0){
		serialReading = YES;
		msgStarted = Serial1.read() == 0xFF ?  YES : NO;
		if (msgStarted && (Serial1.read() == 0xFF)) {
			do { servoID = Serial1.read(); } while (servoID == 0xFF); // this make sure you wait for real data

			msgLength = Serial1.read(); // msg Length
			error_byte = Serial1.read();
			error_byte_old = error_byte ? error_byte : error_byte_old;

			lcd.setCursor(10,0);
			if (error_byte || error_counter) {
				error_counter = error_counter > 10 ? 0 : error_counter + 1;
				String* er;
				er = error_decode(error_byte);
				lcd.print("E:");
				//char buff[7];
				//sprintf(buff, "BYTE_TO_BINARY_PATTERN", BYTE_TO_BINARY(error_byte));
				lcd.print(error_byte_old);
				lcd.print("   ");
			} else { // No Error :)
				lcd.print("     ");
				if (msgLength > 2) {
					position = Serial1.read();
					position = (Serial1.read()<<8) + position;
					if (!arm.servos[servoID].direction) {
						arm.servos[servoID].turns = (arm.servos[servoID].position > (position+100))
							? arm.servos[servoID].turns + 1
							: arm.servos[servoID].turns;
					} else {
						arm.servos[servoID].turns = ((arm.servos[servoID].position+100) < position)
							? arm.servos[servoID].turns - 1
							: arm.servos[servoID].turns;
					}

					speed = Serial1.read();
					speed = (Serial1.read()<<8) + speed;
					
					load = Serial1.read();
					load = (Serial1.read()<<8) + load;
					voltage = Serial1.read();
					temperature = Serial1.read();
					registered = Serial1.read();
					moving = Serial1.read();
					lock = Serial1.read();
					punch = Serial1.read();
					punch = (Serial1.read()<<8) + punch;
					current = Serial1.read();
					current = (Serial1.read()<<8) + current;

					speedDirection = getDirection(speed); 
					speed = getRealValue(speed);
					
					loadDirection = getDirection(load); 
					load = getRealValue(load);
					
					/* Update the Arm Object */
					arm.servos[servoID].position = position;
					arm.servos[servoID].speed = speed;
					arm.servos[servoID].direction = speedDirection;
					arm.servos[servoID].load = load;
					arm.servos[servoID].loadDirection = loadDirection;
		
					/* LCD Line 1 */
					if (arm.id != servoID) {
						printLCD(LCD_COL1, 0, servoID, 1);
						lcd.print(arm.servos[servoID].mode == 1 ? " Whl" : arm.servos[servoID].mode == 2 ? " Mlt" : " Jnt");
					}
					arm.id = servoID;
					/* LCD Line 2 */
					printLCD(LCD_COL1, 1, arm.servos[servoID].position * MX_PRESENT_POSITION_DEGREE, 4);
					lcd.print((char)CH_DEG);
					printLCD(LCD_COL2, 1, arm.servos[servoID].turns,3);
					lcd.print("t");
					/* LCD Line 3 */
					printLCD(LCD_COL1, 2, speed, 4);
					lcd.print(speedDirection ? (char)CH_ARR : (char)CH_ARL);
					printLCD(LCD_COL2, 2, arm.servos[servoID].speed, 4);
					/* LCD Line 4 */
					printLCD(LCD_COL1, 3, load, 4);
					lcd.print(loadDirection ? (char)CH_ARR : (char)CH_ARL);
					//printLCD3(LCD_COL2, 3, (45*(current-2048)));
					/* Update the Global variables tracking the servos */
					currPos = servoID_old == servoID ? currPos : position;
					servoID_old = servoID;
				}
			}
			while(Serial1.available() > 0) { Serial1.read(); }

			delay(1);
			cycle_counter = cycle_counter < 9 ? cycle_counter + 1 : 0;
			printLCD(18, 0, cycle_counter, 1);
		}
	}
	serialReading = NO;
}
void printBufferLCD() {
	delay(10);
	int data;
	bool msgStarted;
	int reading;

	if (Serial1.available()){
		serialReading = YES;
		msgStarted = Serial1.read() == 0xFF ?  YES : NO;
		if (msgStarted && Serial1.read()) {
			Serial1.read(); // servo ID
			Serial1.read(); // msg Length
			Serial1.read(); // zero
			reading = Serial1.read();
			reading = Serial1.available() ? (Serial1.read()<<8) + reading : reading;
			while(Serial1.available()) { Serial1.read(); }
			lcd.clear();
			lcd.setCursor(2,1);
			lcd.print(reading*0.088);
			delay(1);
		}
		serialReading = NO;
	}
}

/* Extra functions */
int sumBytes(uint8_t* bytes, uint8_t parsNo) {
	int sum = 0;
	for (uint8_t i=0; i<parsNo; i++) sum +=bytes[i];
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
	
	arm.id = ARM_ID1;
	/* Set the speeds of the servos */
	arm.servos[1].speed = 500;
	arm.servos[2].speed = 100;
	arm.servos[3].speed = 30;
	arm.servos[4].speed = 15;
	arm.servos[5].speed = 20;
	/* Enable Toques for Joints (4 and 5) */
	for (uint8_t i=1; i<6; i++) {
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
	readServo(arm.id, MX_TORQUE_ENABLE, 8);
	printBuffer();
	delay(2);
	int somthe = getData(5, MX_CURRENT_L);
	printSerial("return: ", getRealValue(somthe));
	printLCD(19, 0, autoBalanceOn ? "A" : "M" );
	//printBuffer();
}

/************************************************************************/
/* LOOP                                                                 */
/************************************************************************/
void loop() {
	int incomingByte = 0;
	readServo(arm.id, MX_PRESENT_POSITION_L, 34);
	printDataLCD();
	int new_reading;
	bool new_direction;
	int speedo;
	/* Load 4  - Reading */
	new_reading = getData(ARM_ID4, MX_PRESENT_LOAD_L);
	if (new_reading > 0) {
		load_4_dir = getDirection(new_reading);
		load_4 = getRealValue(new_reading);
	} else { // use the old values
		load_4_dir = load_4_dir;
		load_4 = load_4; 
	}
	arm.servos[ARM_ID4].load = load_4; 
	arm.servos[ARM_ID4].loadDirection = load_4_dir; 
	/* Load 5 - Reading */
	new_reading = getData(ARM_ID5, MX_PRESENT_LOAD_L);
	if (new_reading > 0) {
		load_5_dir = getDirection(new_reading);
		load_5 = getRealValue(new_reading);
	} else { // use the old values
		load_5_dir = load_5_dir;
		load_5 = load_5;
	}
	arm.servos[ARM_ID5].load = load_5; 
	arm.servos[ARM_ID5].loadDirection = load_5_dir; 
	
	if (autoBalanceOn) {
		/* Load 4  - Controlling */
		speedo = load_4 < ARM_LOAD_MIN_THLD
			? 0
			: load_4 > ARM_LOAD_MAX_THLD
				? ARM_LOAD_MAX_THLD<<1
				: load_4<<1;
		moveSpeed(ARM_ID1, 100, load_4_dir ? speedo : 0x0400 + speedo);

		/* Load 5  - Controlling */
		speedo = load_5 < ARM_LOAD_MIN_THLD
			? 0
			: load_5 > ARM_LOAD_MAX_THLD
				? ARM_LOAD_MAX_THLD<<1
				: load_5<<1;
		moveSpeed(ARM_ID2, 100, load_5_dir ? speedo : 0x0400 + speedo);
	
	} else { /* if (autoBalanceOn) */ }
	if (!arm.autoBalance == autoBalanceOn) {
		printLCD(19, 0, autoBalanceOn ? "A" : "M" );
		if (!autoBalanceOn) { /* Stop servo 1 and 2 when you toggle back to Manual mode */
			moveSpeed(ARM_ID1, 100, load_4_dir ? 0 : 0x0400);
			moveSpeed(ARM_ID2, 100, load_5_dir ? 0 : 0x0400);
		}
		arm.autoBalance = autoBalanceOn;
	}
	printLCD(10, 3, load_4, 4);
	printLCD(15, 3, load_5, 4);
	/* Reading Serial */
	/*
	if (Serial.available() > 0) {
		// read the incoming byte:
		lcd.setCursor(15, 1);
		lcd.print(incomingByte, 10);
		incomingByte = Serial.read();

		// say what you got:
		Serial.print("I received: ");
		Serial.println(incomingByte, DEC);
		lcd.setCursor(15, 2);
		//lcd.print((String)incomingByte);
		lcd.print((unsigned long)incomingByte, 10);
	}
	*/
}
