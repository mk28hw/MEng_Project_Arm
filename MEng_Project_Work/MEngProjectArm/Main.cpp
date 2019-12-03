﻿/*
 * The code is compatible with Arduino Mega ATmega2560 (it may works with other boards).
 * It uses two hardware serial interfaces:
 *   Serial (Port 0) for monitoring and troubleshooting
 *   Serial 1 (Port 1) for communication with MX-64ARs
 *
 * Version 0.4 (19/04/2019) Report Version:
 *
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
#include <math.h> 

/* End of auto generated code by Atmel studio */

#include <Wire.h>
//#include <LiquidCrystal_I2C.h>

/* Local Files Includes */
#include "General.h"
#include "MX-64AR.h"
#include "LCD.h"
#include "Arm.h"


//Beginning of Auto generated function prototypes by Atmel Studio
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
//End of Auto generated function prototypes by Atmel Studio

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
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_ROWS, LCD_COLS);  // set the LCD address to 0x27 for a 20 chars and 4 line display
bool serialWriting = NO;
bool serialReading = NO;

uint16_t load_4, load_5;
bool load_4_dir, load_5_dir;
bool autoBalanceOn = OFF;

uint32_t t_start;
uint8_t AnChannel = 0;
int16_t AnValue[4];
int x_val;
int y_val;
int x, y;
uint32_t t_global_start;
bool fPrintON = true;

/* LCD Helping to print Function */
void printLCD(uint8_t col, uint8_t row, int value, uint8_t padding) {
	char buffer[padding];
	char tmp[5];
	//value = value < 0 ? 0 : value;
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

/* Setup the Switches (Pin Change Interrupts) */
void setupSwitches() {
	DDRB = 0xFF;
	DDRB &= ~((1<<DDB0) | (1<<DDB1) | (1<<DDB2)); // clear DDB0 and DDB4 bits in DDRB. PIN B0 and B4 is set to INPUT (BUTTON)
	/* Pin Change Mask Register 0. Enable Interrupt on PIN 19, 20 and 21 */
	PCMSK0 |= (1<<PCINT0) | (1<<PCINT1) | (1<<PCINT2);  // set Pin Change Mask 0 bit PCINT0.
	PCICR  |= (1<<PCIE0); // Pin Change Interrupt Control Register. Activate PCINT7:0
	sei(); // enable global interrupts
}
void setupJoystick() {	
	DDRL &= ~((1<<DDL1));	
	DDRK &= ~((1<<DDK7) | (1<<DDK6) | (1<<DDK5) | (1<<DDK4));
}
void startConversion(uint8_t channel) {
	ADMUX &= 0xE0; // Clear4s bits MUX4:0
	ADMUX |= channel & 0x07;
	ADCSRB = channel & (1 << 3);
	ADCSRA |= (1 << ADSC);
}
void setupADC() {
	ADMUX &= ~(1 << REFS1);
	ADMUX  |= (1 << REFS0);									// Set the Ref. Voltage to AVcc (+5V)
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);	// Set ADC Pre-scaler to 128
	ADCSRA |= (1 << ADEN);									// Turn on ADC
	ADCSRA |= (1 << ADIE);									// Set the ADC Interrupt
	DIDR0 = (1 << ADC5D);									//

	startConversion(AnChannel + 12);
}

/* Analog input Interrupts */

ISR(ADC_vect) {
	int16_t Reading;
	Reading = ADC < 500 ? ADC - 500 :  ADC > 512 ? ADC - 512 : 0;
	AnValue[AnChannel] = Reading;
	AnChannel = AnChannel > 2 ? 0 : AnChannel + 1;
	startConversion(AnChannel + 12);
}
/*  Buttons Interrupts */
ISR(PCINT0_vect) {
	int speed;
	uint8_t id = arm.id;
	uint16_t i;
	float tempo;
	//int x, y;
	if (BUTTON_1_PRESSED) { // GO DOWN
		LED_ON;
		while (BUTTON_1_PRESSED) {		
			if (id < 3) {
				//arm.servos[id].speed = 500;
				speed = (lastButtonPressed != 1)
					? arm.servos[id].speed_default
					: 0;
				arm.moveSpeed(id, arm.servos[id].position, speed);
			} else {
				speed = arm.servos[id].speed_default;
				arm.moveSpeed(id, arm.servos[id].position-1, speed);
			} 
			delay(200);

		}
		buttonsFlip[1] = !buttonsFlip[1];
		lastButtonPressed = 1;
		LED_OFF;
	}
	if (BUTTON_2_PRESSED) { // GO UP
		LED_ON;
		while (BUTTON_2_PRESSED) {
			if (id < 3) {
				//arm.servos[id].speed = 500;
				speed = (lastButtonPressed != 2)
					? (MX_SPEED_OFFSET + arm.servos[id].speed_default)
					: MX_SPEED_OFFSET;
				arm.moveSpeed(id, arm.servos[id].position, speed);
			} else {
				speed = arm.servos[id].speed_default;
				arm.moveSpeed(id, arm.servos[id].position+1, speed);
			}
			delay(200);

		}
		buttonsFlip[2] = !buttonsFlip[2];
		lastButtonPressed = 2;
		LED_OFF;
	}
	if (BUTTON_3_PRESSED) { // SELECT
		LED_TOGGLE;
		i = 0;
		delay(20);
		while (BUTTON_3_PRESSED) {
			delay(1);
			i++;
		}
		if (i > 1000) {
			autoBalanceOn = !autoBalanceOn;
		} else {
			if (id < 3) { arm.moveSpeed(id, arm.servos[id].position, 0); }
			id = id > 4 ? 1 : id + 1;
			arm.id = id;
			if (id > 3) arm.setModeJoint(id);
			else if (id==3) arm.setModeMultiTurn(id);
			else arm.setModeWheel(id);
		}
		
		/* Stop Servo 1 or 2 if you press button 3 */

		buttonsFlip[3] = !buttonsFlip[3];
		lastButtonPressed = 3;
	}
}

/* New shorter functions - START */


/* Function checking if the Speed is within set limits */
int checkSpeed(uint8_t id, uint16_t Speed) {
	if (id < 3) { // Wheel Mode Unlimited
		return Speed > ARM_ID1_SPEED_MAX
			? ARM_ID1_SPEED_MAX
			: Speed < ARM_ID1_SPEED_MIN
				? ARM_ID1_SPEED_MIN
				: Speed;
	} else if (id == 3) { // Multi-Turn up to 7 turns each direction
		return Speed > ARM_ID3_SPEED_MAX
			? ARM_ID3_SPEED_MAX
			: Speed < ARM_ID3_SPEED_MIN
				? ARM_ID3_SPEED_MIN
				: Speed;
	} else if (id == 4) { // Joint Mode up to one turn
		return Speed > ARM_ID4_SPEED_MAX
			? ARM_ID4_SPEED_MAX
			: Speed < ARM_ID4_SPEED_MIN
				? ARM_ID4_SPEED_MIN
				: Speed;
	} else if (id == 5) { // Joint Mode up to one turn
		return Speed > ARM_ID5_SPEED_MAX
			? ARM_ID5_SPEED_MAX
			: Speed < ARM_ID5_SPEED_MIN
				? ARM_ID5_SPEED_MIN
				: Speed;
	} else {
		return -1;
	}
}

/* New shorter functions - END */

/* Some old (but shortened) function from previous dissertation */
void setServo(uint8_t mode, int rotate) {
	arm.setMaxTorque(0, 1023);
	arm.setMode(0, mode);
	arm.turn(0, rotate, 100);
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

/* */
void printBuffer() {
	//delay(20);
	//uint8_t start_1, start_2, servo_id, msg_length, er_byte, chck_sum;

	Serial.println((String)"## Print Buffer Start: "+Serial1.available()+" ####################");
	serialReading = YES;
	while(Serial1.available()){
		Serial.println((String)"Available: "+Serial1.available()+", Read Data: "+Serial1.read());
		delay(1);
	}
	serialReading = NO;
	Serial.println("## Print Buffer End #####################");
}

/*  Request, Capture data from servo with given ID and Update arm's object fields:
		arm.servos[id].position
		arm.servos[id].speed
		arm.servos[id].speedDirection
		arm.servos[id].load
		arm.servos[id].loadDirection
*/
int getMoreData(uint8_t id, uint8_t ctrlData) {
	uint8_t msgByte;
	bool startOne = NO, startTwo = NO, msgStarted, msgOK = NO;
	uint8_t byteCount = 0;
	uint8_t msgId, msgLength, msgError, msgChecksum, Checksum;
	uint8_t msgData_1, msgData_2,  msgData_3, msgData_4, msgData_5, msgData_6;
	int msgData;
	String line;

	arm.readServo(id, ctrlData, 6);
	delay(10);
	serialReading = YES;

	msgByte = Serial1.read();
	startOne = msgByte == 0xFF ? YES : NO;
	msgByte = Serial1.read();									// 02 : Start 2/2
	startTwo = msgByte == 0xFF ? YES : NO;

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
		line = line + ", Posi:" + padNumber(msgData_1, 6);			//4
		msgData_3 = Serial1.read();
		msgData_4 = Serial1.read();
		line = line + ", Sped:" + padNumber(msgData_3, 6);			//5
		msgData_5 = Serial1.read();
		msgData_6 = Serial1.read();
		line = line + ", Load:" + padNumber(msgData, 6);			//6
		msgChecksum = Serial1.read();
		line = line + ", MsgCheckSum:" + padNumber(msgChecksum, 6); //7
		Checksum = ~lowByte(msgId + msgLength + msgError + msgData_1 + msgData_2 + msgData_3 + msgData_4 + msgData_5 + msgData_6);
		line = line + ", CalCheckSum:" + padNumber(Checksum, 6);	//8
		msgOK = msgChecksum == Checksum ? YES : NO;
		if (msgOK) {
			arm.updateServoPosition(id, msgData_1, msgData_2);
			arm.updateServoSpeed(id, combineBytes(msgData_3, msgData_4));
			arm.updateServoLoad(id, msgData_5, msgData_6);
		}
	}
	while(Serial1.available()) { msgByte = Serial1.read(); }
	serialReading = NO;

    /*  Check if the returned data is not corrupted and there are no errors
            Return -255 if the data is corrupted (Checksum error)
            Return -ErrorCode if there is error from servo */
	return msgOK ? msgError ? -msgError : 0 : -255;
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

//void dumpDataToSerial(char** data, uint8_t number) { Serial.println(combineToCSV(data, number)); }

void dumpDataToSerial(){
	uint8_t number = 14;
	int data[number] = {
		(int)((millis() - t_start)/100),
		arm.servos[4].position,
		arm.servos[4].load,
		arm.servos[4].loadDirection,
		arm.servos[4].current,
		arm.servos[1].speed,
		arm.servos[1].speedDirection,
		arm.servos[5].position,
		arm.servos[5].load,
		arm.servos[5].loadDirection,
		arm.servos[5].current,
		arm.servos[2].speed,
		arm.servos[2].speedDirection,
		arm.servos[3].speed
	};
	dumpDataToSerial(data, number);
}
/* Extra functions */

int rawToMiliAmps(int currentRaw) { return (int) 4.5*(currentRaw-2048); }

uint8_t number = 13;
bool cycle_flag = false;

/************************************************************************/
/* SETUP                                                                */
/************************************************************************/
void setup_old() {
	setupSwitches();					// Setup the arm control switches
	setupJoystick();
	setupADC();
	/* LCD Setup */
	lcd.init();							// initialize the lcd
	uint8_t ch_arrow_down[8]	= {B00100, B00100, B00100, B00100, B10101, B01110, B00100, B00000};
	uint8_t ch_arrow_up[8]		= {B00100, B01110, B10101, B00100, B00100, B00100, B00100, B00000};
	lcd.createChar(0, ch_arrow_down);
	lcd.createChar(1, ch_arrow_up);	
	lcd.backlight();
	lcd.clear();
	/* Pins Setup */
	pinMode(RS485_RX_EN_PIN, OUTPUT);
	pinMode(RS485_TX_EN_PIN, OUTPUT);
	/* Serials Setup */
	Serial.begin(57600);
	Serial1.begin(57143);
	Serial1.flush();
	arm.id = 1;
	/* Angle Limits Setup (Servo Modes) */
	printLCD(0, 0, "Set Angle Limits" );
	
	printLCD(18, 0, "OK" );
	arm.setModeWheel(1);
	arm.setModeWheel(2);
	arm.setModeMultiTurn(3);
	arm.setModeJoint(4);
	arm.setModeJoint(5);
	delay(500);
	printLCD(18, 0, "OK" );

	/* Set the speeds of the servos */
	printLCD(0, 1, "Set Speeds" );
	arm.servos[1].speed_default = 750;
	arm.servos[2].speed_default = 500;
	arm.servos[3].speed_default = 30;
	arm.servos[4].speed_default = 20;
	arm.servos[5].speed_default = 60;
	delay(500);
	printLCD(18, 1, "OK" );

	/* Enable Toques for Joints (4 and 5) */
	printLCD(0, 2, "Set Torques" );
	for (uint8_t i=1; i<6; i++) {
		arm.setTorqueLimit(i, 0x0200);
	}
	delay(500);
	printLCD(18, 2, "OK" );
	printLCD(0, 3, "Update Angles " );
	//arm.servos[4].position = getData(4, MX_PRESENT_POSITION_L);
	printSerial("position 4", arm.servos[4].position);
	delay(100);
	arm.servos[5].position = arm.getData(5, MX_PRESENT_POSITION_L);
	printSerial("position 5", arm.servos[5].position);
	delay(100);
	//moveSpeed(4, arm.servos[4].position, arm.servos[4].speed);
	arm.moveSpeed(5, arm.servos[5].position, arm.servos[5].speed);
	delay(100);
	printLCD(18, 3, "OK" );
	delay(500);

	/* LCD Setup */
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("ID#");
	lcd.setCursor(5,0);
	lcd.print("mA:");
	lcd.setCursor(0,1);
	lcd.print("an:");
	lcd.setCursor(0,2);
	lcd.print("sp:");
	lcd.setCursor(0,3);
	lcd.print("ld:");

	for (uint8_t i=1; i<6; i++) {
		Serial.print(i);
		arm.readServo(i, 0x00, 128);
		printBuffer();
	}

	/*
	moveSpeedEasy(5,110, 0.125);
	printLCD(LCD_COL1, LCD_ROW3, 125, 4);
	printLCD(LCD_COL2, LCD_ROW1, abs(arm.servos[5].current), 4);
	delay(32000);
	moveSpeedEasy(5,68.2, 0.25);
	printLCD(LCD_COL1, LCD_ROW3, 25, 4);
	printLCD(LCD_COL2, LCD_ROW1, abs(arm.servos[5].current), 4);
	delay(16000);
	moveSpeedEasy(5,110, 0.5);
	printLCD(LCD_COL1, LCD_ROW3, 5, 4);
	printLCD(LCD_COL2, LCD_ROW1, abs(arm.servos[5].current), 4);
	delay(8000);
	moveSpeedEasy(5,68.2, 1);
	printLCD(LCD_COL1, LCD_ROW3, 1, 4);
	printLCD(LCD_COL2, LCD_ROW1, abs(arm.servos[5].current), 4);
	delay(4000);
	moveSpeedEasy(5,110, 2);
	printLCD(LCD_COL1, LCD_ROW3, 2, 4);
	printLCD(LCD_COL2, LCD_ROW1, abs(arm.servos[5].current), 4);
	delay(4000);
	moveSpeedEasy(5,68.2, 4);
	printLCD(LCD_COL1, LCD_ROW3, 4, 4);
	printLCD(LCD_COL2, LCD_ROW1, abs(arm.servos[5].current), 4);
	delay(2000);
	moveSpeedEasy(5,110, 10);
	printLCD(LCD_COL1, LCD_ROW3, 10, 4);
	printLCD(LCD_COL2, LCD_ROW1, abs(arm.servos[5].current), 4);
	delay(2000);
	moveSpeedEasy(5,68.2, 20);
	printLCD(LCD_COL1, LCD_ROW3, 20, 4);
	printLCD(LCD_COL2, LCD_ROW1, abs(arm.servos[5].current), 4);
	delay(200); */
	//printLCD(19, 0, autoBalanceOn ? "A" : "M" );
	double time_fkX, time_fkY, time_ikS5, time_ikS4;
	//t_start = millis();
	/* Update Servo 4 */
	getMoreData(4, MX_PRESENT_POSITION_L);

	/* Update Servo 5 */
	getMoreData(5, MX_PRESENT_POSITION_L);
	uint32_t t_now;
	int one, two;
	float three, four;
	
	t_start = millis();
	one = arm.fkX();
	t_now = millis();
	time_fkX = (int)((t_now - t_start)*1000);
	
	t_start = millis();
	two = arm.fkY();
	t_now = millis();
	time_fkY = (int)((t_now - t_start)*1000);
	
	t_start = millis();
	three = arm.ikAngleServo5();
	t_now = millis();
	time_ikS5 = (int)((t_now - t_start)*1000);
	
	t_start = millis();
	four = arm.ikAngleServo4();
	t_now = millis();
	time_ikS4 = (int)((t_now - t_start)*1000);
	
	 int data[6] = {
		 time_fkX,
		 one,
		 time_fkY,
		 two,
		 time_ikS5,
		 time_ikS4
	 };
	 Serial.print(three);
	 Serial.print(four);
	dumpDataToSerial(data, 6);
	delay(20000);
}

void exampleFunc() {
	//int a, b, c;
	//a = 100;
	//b = 345;
	//c = sqrt(a * b) * a / sqrt(b)  ;
	delay(1001);
}
void timeTheRuntime()  {
	int n = 0;
	int result; 
	arm.servos[4].position_rad = 1.2;
	arm.servos[5].position_rad = 2.2;
	uint32_t t_start, t_now, t_ela , max_t_ela = 0, min_t_ela = 0xFFFFFFFF, avg_t_ela = 0;
	uint64_t t_ela_sum = 0;
	Serial.begin(57600);
	Serial.println("ikth2()");
	for (int i = 2; i < 10; i++) {
		for (int j = -8; j < 5; j++) {
			// for fk uncomment next 2 lines
			//updateServoPosition(4,i*10);
			//updateServoPosition(5,j*10);
			
			// for ik uncomment next 2 lines
			arm.toolPoseX = i * 50;
			arm.toolPoseY = j * 50;
			
			Serial.print(n);
			Serial.print(", ");
			//Serial.print(arm.servos[4].position);
			Serial.print(arm.toolPoseX);
			Serial.print(", ");
			//Serial.print(arm.servos[5].position);
			Serial.print(arm.toolPoseY);
			Serial.print(", ");
			
			t_start = micros();
			//result = fkY();
			result = (int) (arm.ikAngleServo4() * CONVERT_RADIANS_DEGREE);
			t_now = micros();
			
			t_ela = t_now - t_start;
			Serial.print(result);
			Serial.print(", ");
			Serial.println(t_ela);
			n = n + 1;
			t_ela_sum = t_ela_sum + t_ela;
			max_t_ela = t_ela > max_t_ela ? t_ela : max_t_ela;
			if (max_t_ela > 1000) delay(2000);
			min_t_ela = t_ela < min_t_ela ? t_ela : min_t_ela;
		}
	}
	avg_t_ela = t_ela_sum / (n+0);
		
	printSerial("Elapsed MIN", min_t_ela);
	printSerial("Elapsed MAX", max_t_ela);
	printSerial("Elapsed AVG", avg_t_ela);
}

/* Timing processes for Optimization Analysis */
void experiment_0(){
	timeTheRuntime();
	arm.toolPoseX = 220;
	arm.toolPoseY = -400;
	float resa2 = arm.ikAngleServo5();
	printSerial("IK theta 2", resa2 * 180 / M_PI);
	float resa1 = arm.ikAngleServo4();
	printSerial("IK theta 1", resa1 * 180 / M_PI);
}

/* Analysis of the changes of load versus changes of Servo 4 position - output to Serial port */
void experiment_1(){
	/* Take end-effector to the very low position slowly and output data to Serial port */
	Serial.println((String)(millis() - t_global_start)+". Go to E=(200,-380) Starts");
	arm.goToXY(200,-380);
	
	for (uint8_t i=0; i<50; i++) {
		getMoreData(4, MX_PRESENT_POSITION_L); /* Update Servo 4 */
		getMoreData(5, MX_PRESENT_POSITION_L); /* Update Servo 5 */
		dumpDataToSerial();
		delay(100);
	}
	Serial.println((String)(millis() - t_global_start)+". Go to E=(200,-380) Ends");
	delay(2000);
	
	/* move Servo 4 up slowly and output data to Serial port */
	Serial.println((String)(millis() - t_global_start)+". Move S5 to 150 Starts");
	arm.moveSpeedEasy(4, 150, 0.5);
	for (uint8_t i=0; i<140; i++) {
		getMoreData(4, MX_PRESENT_POSITION_L); /* Update Servo 4 */
		getMoreData(5, MX_PRESENT_POSITION_L); /* Update Servo 5 */
		dumpDataToSerial();
		delay(100);
	}
	Serial.println((String)(millis() - t_global_start)+". Move S5 to 150 Ends");
	delay(5000);
}

void setup() {
	t_global_start = millis();
	
	setupSwitches();					// Setup the arm control switches
	setupJoystick();
	setupADC();
	
	/* LCD Setup */
	lcd.init();							// initialize the lcd
	uint8_t ch_arrow_down[8]	= {B00100, B00100, B00100, B00100, B10101, B01110, B00100, B00000};
	uint8_t ch_arrow_up[8]		= {B00100, B01110, B10101, B00100, B00100, B00100, B00100, B00000};
	lcd.createChar(0, ch_arrow_down);
	lcd.createChar(1, ch_arrow_up);
	lcd.backlight();
	lcd.clear();
	
	/* Pins Setup */
	pinMode(RS485_RX_EN_PIN, OUTPUT);
	pinMode(RS485_TX_EN_PIN, OUTPUT);
	
	/* Serials Setup */
	Serial.begin(57600);
	Serial1.begin(57143);
	Serial1.flush();
	arm.id = 1;
	
	/* Angle Limits Setup (Servo Modes) */
	printLCD(0, 0, "Set Angle Limits" );
	arm.setModeWheel(1);
	arm.setModeWheel(2);
	arm.setModeMultiTurn(3);
	arm.setModeJoint(4);
	arm.setModeJoint(5);
	delay(500);
	printLCD(18, 0, "OK" );

	/* Set the default speeds of the servos */
	printLCD(0, 1, "Set Speeds" );
	arm.servos[1].speed_default = 1000;
	arm.servos[2].speed_default = 1000;
	arm.servos[3].speed_default = 30;
	arm.servos[4].speed_default = 10;
	arm.servos[5].speed_default = 10;
	delay(500);
	printLCD(18, 1, "OK" );

	/* Enable Toques for Joints (4 and 5) */
	printLCD(0, 2, "Set Torques" );
	arm.setTorqueLimit(1, 0x03FF); // 100%
	arm.setTorqueLimit(2, 0x03FF); // 100%
	arm.setTorqueLimit(3, 0x0200); //  50%
	arm.setTorqueLimit(4, 0x03FF); // 100%
	arm.setTorqueLimit(5, 0x03FF); // 100%
	delay(500);
	printLCD(18, 2, "OK" );
	
	/* Update Angles (Positions) for Joints (3, 4 and 5) */
	printLCD(0, 3, "Update Angles S3-5 " );
	arm.servos[3].position = arm.getData(3, MX_PRESENT_POSITION_L);
	arm.servos[4].position = arm.getData(4, MX_PRESENT_POSITION_L);
	arm.servos[5].position = arm.getData(5, MX_PRESENT_POSITION_L);
	delay(500);
	printLCD(18, 3, "OK" );
	
	delay(500);

	/* LCD Captions Setup */
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("ID#");
	lcd.setCursor(5,0);
	lcd.print("mA:");
	lcd.setCursor(0,1);
	lcd.print("an:");
	lcd.setCursor(0,2);
	lcd.print("sp:");
	lcd.setCursor(0,3);
	lcd.print("ld:");
	Serial.println((String)"Max Torque: "+arm.getData(1, MX_MAX_TORQUE_L)); 
	Serial.println((String)"Goal Torque: "+arm.getData(1, MX_GOAL_TORQUE_L)); 
	/*
	turn(1,0,50);
	delay(2000);
	turn(1,0,750);
	delay(2000);
	turn(1,0,0);
	delay(2000);
	turn(1,1,100);
	delay(2000);
	turn(1,0,0);
	delay(2000);
	*/
	//arm.goHome();
	//delay(3000);
	//arm.goToXY(200,-230);
	//delay(4000);
	//goHome();
	arm.moveSpeed(4, 100*CONVERT_DEGREE_POSITION, 2*MX_CONVERT_PERCENT_SPEED);
	delay(10000);
	fPrintON = false;
	//experiment_1();
}
void notused() {
		/*
	moveSpeedEasy(5,110, 0.5);
	printLCD(LCD_COL1, LCD_ROW3, 5, 4);
	printLCD(LCD_COL2, LCD_ROW1, abs(arm.servos[5].current), 4);
	delay(8000);
	moveSpeedEasy(5,68.2, 1);
	printLCD(LCD_COL1, LCD_ROW3, 1, 4);
	printLCD(LCD_COL2, LCD_ROW1, abs(arm.servos[5].current), 4);
	delay(4000);
	moveSpeedEasy(5,110, 2);
	printLCD(LCD_COL1, LCD_ROW3, 2, 4);
	printLCD(LCD_COL2, LCD_ROW1, abs(arm.servos[5].current), 4);
	delay(4000);
	moveSpeedEasy(5,68.2, 4);
	printLCD(LCD_COL1, LCD_ROW3, 4, 4);
	printLCD(LCD_COL2, LCD_ROW1, abs(arm.servos[5].current), 4);
	delay(2000);
	*/
	//timeTheRuntime();
	//delay(2000);
	// Go Home
		for (uint8_t i=1; i<6; i++) {
			Serial.print(i);
			arm.readServo(i, 0x00, 128);
			printBuffer();
		}
	
	printLCD(LCD_COL2-1, LCD_ROW3, arm.fkX(), 4);
	printLCD(LCD_COL3-1, LCD_ROW3, arm.fkY(), 4);
	Serial.println((String)(millis() - t_global_start)+". Go to XY 1 Ends");
	//goToXY(200,-340);
	for (uint8_t i=0; i<50; i++) {
		getMoreData(4, MX_PRESENT_POSITION_L); /* Update Servo 4 */
		getMoreData(5, MX_PRESENT_POSITION_L); /* Update Servo 5 */
		dumpDataToSerial();
		delay(100);
	}
	printLCD(LCD_COL2-1, LCD_ROW3, arm.fkX(), 4);
	printLCD(LCD_COL3-1, LCD_ROW3, arm.fkY(), 4);
	Serial.println((String)(millis() - t_global_start)+". Go to XY 2 Ends");
	arm.goToXY(200,-300);
	for (uint8_t i=0; i<50; i++) {
		getMoreData(4, MX_PRESENT_POSITION_L); /* Update Servo 4 */
		getMoreData(5, MX_PRESENT_POSITION_L); /* Update Servo 5 */
		dumpDataToSerial();
		delay(100);
	}
	printLCD(LCD_COL2-1, LCD_ROW3, arm.fkX(), 4);
	printLCD(LCD_COL3-1, LCD_ROW3, arm.fkY(), 4);
	Serial.println((String)(millis() - t_global_start)+". Go to XY 3 Ends");
	arm.goToXY(200,-260);
	for (uint8_t i=0; i<50; i++) {
		getMoreData(4, MX_PRESENT_POSITION_L); /* Update Servo 4 */
		getMoreData(5, MX_PRESENT_POSITION_L); /* Update Servo 5 */
		dumpDataToSerial();
		delay(100);
	}
	printLCD(LCD_COL2-1, LCD_ROW3, arm.fkX(), 4);
	printLCD(LCD_COL3-1, LCD_ROW3, arm.fkY(), 4);
}

/************************************************************************/
/* LOOP                                                                 */
/************************************************************************/

void loop() {
    int deadZone = 50;
    int new_reading;
    int speedo;
    uint8_t id;
	
	/************************************************************************/
	/* Auto/Manual Balance Section                                          */
	/************************************************************************/
	
	/* Auto-balance Code */
	if (autoBalanceOn) {    
		/* Servo 4 Load - Controlling: Simple Constant feedback loop correction */
		speedo = arm.servos[4].load < deadZone ? 0 : 500;
		arm.moveSpeed(1, 100, arm.servos[4].loadDirection ? speedo : MX_SPEED_OFFSET + speedo);

		/* Servo 5 Load - Controlling: Simple Constant feedback loop correction */
		speedo = arm.servos[5].load < deadZone ? 0 : 500;
		arm.moveSpeed(2, 100, arm.servos[5].loadDirection ? speedo : MX_SPEED_OFFSET + speedo);
	}
	    
	/* Stop Servo 1 and 2 when arm mode change from Auto to Manual */
	if (!arm.autoBalance == autoBalanceOn && !autoBalanceOn) {
		Serial.println((String)(millis() - t_global_start)+". Stopping Servo 1 and 2");
		arm.moveSpeed(1, 100, load_4_dir ? 0 : MX_SPEED_OFFSET);
		arm.moveSpeed(2, 100, load_5_dir ? 0 : MX_SPEED_OFFSET);
		arm.autoBalance = autoBalanceOn;
	}
	
    /************************************************************************/
    /* Updating Servos Section                                              */
    /************************************************************************/
	
    /* Update Position, Speed, Speed Direction, Load and Load Direction */
	if (arm.id < 4) { new_reading = getMoreData(arm.id, MX_PRESENT_POSITION_L); } /* Update current Servo (except 4 & 5) */
    new_reading = getMoreData(4, MX_PRESENT_POSITION_L); /* Update Servo 4 */
    new_reading = getMoreData(5, MX_PRESENT_POSITION_L); /* Update Servo 5 */
    
    /* Update Speed reading of Servo 1 */
    new_reading = arm.getData(1, MX_PRESENT_SPEED_L);   
    if (new_reading>=0) { arm.updateServoSpeed(1, new_reading); }
    
    /* Update Speed reading of Servo 2 */
    new_reading = arm.getData(2, MX_PRESENT_SPEED_L);
    if (new_reading>=0) { arm.updateServoSpeed(2, new_reading); }
    
    /* Update Current [mA] reading of Servo 4 */
    new_reading = arm.getData(4, MX_CURRENT_L);
    if (new_reading>=0) { arm.servos[4].current = rawToMiliAmps(new_reading); }
    
    /* Update Current [mA] reading of Servo 5 */
    new_reading = arm.getData(5, MX_CURRENT_L);
    if (new_reading>=0) { arm.servos[5].current = rawToMiliAmps(new_reading); }
		
    /************************************************************************/
    /* Printing on LCD Section                                              */
    /************************************************************************/
	
    /* Line 1 - Selected ID, Current in mA */
    printLCD(LCD_COL1, LCD_ROW1, arm.id, 1);
	lcd.setCursor(LCD_COL2, LCD_ROW1);
	lcd.print(arm.servos[4].loadDirection?"+":"-");
	printLCD(LCD_COL2+1, LCD_ROW1, arm.servos[4].load, 3);
	
	lcd.setCursor(LCD_COL3, LCD_ROW1);
	lcd.print(arm.servos[5].loadDirection?"+":"-");
	printLCD(LCD_COL3+1, LCD_ROW1, arm.servos[5].load, 3);
    //printLCD(LCD_COL2, LCD_ROW1, abs(arm.servos[4].current), 4);
    //printLCD(LCD_COL3, LCD_ROW1, abs(arm.servos[5].current), 4);
	/* Line 1 - Auto / Manual Balance Indicator */ 
    if (cycle_counter > 1) {
	    cycle_flag = !cycle_flag;
	    cycle_counter = 0;
	    printLCD(19, 0, (cycle_flag ? autoBalanceOn ? "A" : "M" : " "));
	} else { cycle_counter++; }
	    
	/* Line 2 - Position in Degrees */
	printLCD(LCD_COL1, LCD_ROW2, arm.id == 3 ? ((int) arm.servos[arm.id].position*CONVERT_POSITION_DEGREE/3) + 120 : arm.servos[arm.id].position*CONVERT_POSITION_DEGREE, 4);
	lcd.print((char)CH_DEG);
	printLCD(LCD_COL2, LCD_ROW2, arm.servos[4].position*CONVERT_POSITION_DEGREE, 4);
	//printLCD(LCD_COL2, LCD_ROW2, arm.servos[4].position, 4);
	lcd.print((char)CH_DEG);
	printLCD(LCD_COL3, LCD_ROW2, arm.servos[5].position*CONVERT_POSITION_DEGREE, 4);
	//printLCD(LCD_COL3, LCD_ROW2, arm.servos[5].position, 4);
	lcd.print((char)CH_DEG);
	    
	/* Line 3 - Speed */
	printLCD(LCD_COL1, LCD_ROW3, arm.servos[arm.id].speed, 4);
	if (arm.servos[arm.id].speed) {
		lcd.print(arm.servos[arm.id].speedDirection ? arm.id==3 ? (char)CH_AR_RIGHT : (char)CH_AR_UP : arm.id==3 ? (char)CH_AR_LEFT : (char)CH_AR_DOWN);
	} else { lcd.print(" "); }
	//printLCD(LCD_COL2, LCD_ROW3, arm.servos[4].speed, 4);
	if (arm.servos[1].speed) { lcd.print(arm.servos[1].speedDirection ? (char)CH_AR_UP : (char)CH_AR_DOWN); }
	//printLCD(LCD_COL3, LCD_ROW3, arm.servos[5].speed, 4);
	if (arm.servos[2].speed) { lcd.print(arm.servos[2].speedDirection ? (char)CH_AR_UP : (char)CH_AR_DOWN); }
		    
	/* Line 4 - Load */
	/* 
	printLCD(LCD_COL1, LCD_ROW4, arm.servos[arm.id].load, 4);
	lcd.print(arm.servos[arm.id].loadDirection ? arm.id==3 ? (char)CH_AR_RIGHT : (char)CH_AR_UP : arm.id==3 ? (char)CH_AR_LEFT : (char)CH_AR_DOWN);
	printLCD(LCD_COL2, LCD_ROW4, arm.servos[4].load, 4);
	lcd.print(arm.servos[4].loadDirection ? (char)CH_AR_UP : (char)CH_AR_DOWN);
	printLCD(LCD_COL3, LCD_ROW4, arm.servos[5].load, 4);
	lcd.print(arm.servos[5].loadDirection ? (char)CH_AR_UP : (char)CH_AR_DOWN);
	*/
		   
	printLCD(LCD_COL1-1, LCD_ROW3, AnValue[2], 4);
	printLCD(LCD_COL2-1, LCD_ROW3, AnValue[3], 4);
	arm.moveSpeedEasy(4,AnValue[2]<0?0:1,AnValue[2]/100);
	
	arm.toolPoseX = arm.fkX();
	arm.toolPoseY = arm.fkY();
	//arm.calculate(arm.toolPoseX, arm.toolPoseY);
	//Serial.println((String)"Angle Ratio: ("+arm.ratioServo4+", "+arm.ratioServo5+")");
	
	printLCD(LCD_COL1-1, LCD_ROW4, arm.toolPoseX, 4);
	printLCD(LCD_COL2-1, LCD_ROW4, arm.toolPoseY, 4);	
	/*
	printLCD(LCD_COL3-1, LCD_ROW4, ikAngleServo4()*CONVERT_RADIANS_DEGREE, 4);	
	printLCD(LCD_COL3+3, LCD_ROW4, ikAngleServo5()*CONVERT_RADIANS_DEGREE, 4);
	*/
	/************************************************************************/
	/* Printing to Serial Section                                           */
	/************************************************************************/		
		
	uint32_t t_now = millis();

	int data[number] = {
		(int)((t_now - t_start)/100),
		arm.servos[4].position,
		arm.servos[4].load,
		arm.servos[4].loadDirection,
		arm.servos[4].current,
		arm.servos[1].speed,
		arm.servos[1].speedDirection,
		arm.servos[5].position,
		arm.servos[5].load,
		arm.servos[5].loadDirection,
		arm.servos[5].current,
		arm.servos[2].speed,
		arm.servos[2].speedDirection
	};/*
	Serial.print(arm.servos[4].position);
	Serial.print(", ");
	Serial.print(arm.servos[4].position_rad);
	Serial.print(", ");
	Serial.print(arm.servos[5].position);
	Serial.print(", ");
	Serial.print(arm.servos[5].position_rad);
	Serial.print(", ");
	Serial.print(x);
	Serial.print(", ");
	Serial.print(y);
	Serial.print(", ");
	Serial.print(ikAngleServo4(x,y)*CONVERT_RADIANS_POSITION);
	Serial.print(", ");
	Serial.println(ikAngleServo5(x,y)*CONVERT_RADIANS_POSITION);
	*/
	dumpDataToSerial(data, number);
}
//
