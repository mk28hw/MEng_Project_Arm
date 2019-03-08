/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/*End of auto generated code by Atmel studio */

/* this code is a mess and not everything works, 
 * The code is compatible with Arduino Mega ATmega2560 (it may works with other boards).
 * It uses two hardware serial interfaces:
 * Serial (Port 0) for monitoring and troubleshooting
 * Serial 1 (Port 1) for communication with MX-64ARs
 * 
 * Version 0.2:
 * 	Control Buttons(3) for adjusting the servos: [motorSelect][[down][up]
 * 
 * Date: 08/03/2019 */
 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//Beginning of Auto generated function prototypes by Atmel Studio
int sumBytes(byte* bytes, byte parsNo);
void writeServo(byte pcktID, byte pcktCmnd, byte* pcktPars, byte parsNo);
void moveSpeed(unsigned char ID, int Position, int Speed);
void rotateon(int r);
void rotateoff(int r);
void Downon();
void Downoff();
void Upon();
void Upoff();
void printBuffer();
void printDataLCD();
void printBufferLCD();
void getID();
void getID_();
void setID(byte newID);
void move(unsigned char ID, int Position);
void reset(unsigned char ID);
void reset();
void setEndless(unsigned char ID, bool Status);
void setEndless(unsigned char ID);
void clearEndless(unsigned char ID);
void turn_(unsigned char ID, bool SIDE, int Speed);
void turn(byte ID, bool SIDE, int Speed);
void moveSpeed_(unsigned char ID, int Position, int Speed);
void setMaxTorque(unsigned char ID, int MaxTorque);
void torqueStatus(unsigned char ID, bool Status);
void readServo(byte pcktID, byte pcktCmnd, byte resLength);
void readServo(byte pcktID, byte pcktCmnd);
//End of Auto generated function prototypes by Atmel Studio



#define OFF 0
#define ON 1
#define NO 0
#define YES 1
#define CCW 0
#define CW 1
#define LEFT 0
#define RIGTH 1

#define MX_START 0xFF // to indicate the starting point of serial message (2x)
#define MX_ALL_SERVOS 0xFE // to broadcast to all the connected servos

/* MX-64AR Control Table (EEPROM) */
#define MX_MODEL_NUMBER_L 0
#define MX_MODEL_NUMBER_H 0x01
#define MX_VERSION_OF_FIRMWARE 0x02
#define MX_ID 0x03
#define MX_BAUD_RATE 0x04
#define MX_RETURN_RELAY_TIME 0x05
#define MX_CW_ANGLE_LIMIT_L 0x06
#define MX_CW_ANGLE_LIMIT_H 0x07
#define MX_CCW_ANGLE_LIMIT_L 0x08
#define MX_CCW_ANGLE_LIMIT_H 0x09

#define MX_ANGLE_MODE_WHEEL 0x00
#define MX_ANGLE_MODE_MULTI_TURN_L 0xFF
#define MX_ANGLE_MODE_MULTI_TURN_H 0x0F
 
#define MX_MAX_TORQUE_L 0x0E
#define MX_MAX_TORQUE_H 0x0F
#define MX_STATUS_RETURN_LEVEL 0x10
#define MX_ALARM_LED 0x11
#define MX_ALARM_SHUTDOWN 0x12
#define MX_MULTI_TURN_OFFSET_L 0x14
#define MX_MULTI_TURN_OFFSET_H 0x15
#define MX_RESOLUTION_DIVIDER 0x16
/* MX-64AR Control Table (RAM) */
#define MX_TORQUE_ENABLE 0x18
#define MX_LED 0x19
#define MX_D_GAIN 0x1A
#define MX_I_GAIN 0x1B
#define MX_P_GAIN 0x1C
#define MX_GOAL_POSITION_L 0x1E
#define MX_GOAL_POSITION_H 0x1F
#define MX_MOVING_SPEED_L 0x20
#define MX_MOVING_SPEED_H 0x21
#define MX_TORQUE_LIMIT_L 0x22
#define MX_TORQUE_LIMIT_H 0x23
#define MX_PRESENT_POSITION_L 0x24
#define MX_PRESENT_POSITION_H 0x25
#define MX_PRESENT_SPEED_L 0x26
#define MX_PRESENT_SPEED_H 0x27
#define MX_PRESENT_LOAD_L 0x28
#define MX_PRESENT_LOAD_H 0x29
#define MX_PRESENT_VOLTAGE 0x2A
#define MX_PRESENT_TEMPERATURE 0x2B
#define MX_REGISTERED 0x2C
#define MX_MOVING 0x2E
#define MX_LOCK 0x2F
#define MX_PUNCH_L 0x30
#define MX_PUNCH_H 0x31
#define MX_CURRENT_L 0x44
#define MX_CURRENT_H 0x45
#define MX_TORQUE_CONTROL_MODE_ENABLE 0x46
#define MX_GOAL_TORQUE_L 0x47
#define MX_GOAL_TORQUE_H 0x48
#define MX_GOAL_ACCELERATION 0x49
/* MX-64AR Instruction Table */
#define MX_INSTRUCTION_PING 0x01
#define MX_INSTRUCTION_READ_DATA 0x02
#define MX_INSTRUCTION_WRITE_DATA 0x03
#define MX_INSTRUCTION_REG_WRITE 0x04
#define MX_INSTRUCTION_ACTION 0x05
#define MX_INSTRUCTION_RESET 0x06
#define MX_INSTRUCTION_SYNC_WRITE 0x83
/* MX-64AR Conversion Constants */
#define MX_PRESENT_POSITION_DEGREE 0.088
#define MX_PRESENT_SPEED_RPM 0.11
#define MX_PRESENT_LOAD_PERCENT 0.0978

/* MX-64AR Commands/Message Parameters Lengths */
#define MX_GOAL_LENGTH 5
#define MX_INSTRUCTION_RESET_LENGTH 2
#define MX_SPEED_LENGTH 5
#define MX_GOAL_SP_LENGTH 7
#define MX_TORQUE_LENGTH 4
#define MX_MT_LENGTH 5
#define MX_POS_LENGTH 4
#define MX_CCW_AL_L 0xFF
#define MX_CCW_AL_H 0x03

/* Setting both of these two pins to: (a) HIGH - enable TX and disable RX, (b) LOW - enable RX and disable TX */
#define RS485_RX_EN_PIN 2 // this is actually disable pin (not enable) for RX
#define RS485_TX_EN_PIN 3 // enable pin for TX

#define ARM_ID4_ANGLE_MIN 740	// exactly 752 = 65 degrees
#define ARM_ID4_ANGLE_MAX 1700	// exactly 1762 = 157 degrees

#define ARM_ID5_ANGLE_MIN 740	// exactly 749 = 65 degrees
#define ARM_ID5_ANGLE_MAX 1700	// exactly 1744 = 155 degrees

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
	
#define BUTTON_1_PRESSED (PINB & (1<<PINB0)) // PIN 53
#define BUTTON_2_PRESSED (PINB & (1<<PINB1)) // PIN 52
#define BUTTON_3_PRESSED (PINB & (1<<PINB2)) // PIN 51
#define LED_ON PORTB |= (1<<PINB7)
#define LED_OFF PORTB &= ~(1<<PINB7)
#define LED_TOGGLE PINB |= (1<<PINB7)
#define PIN53_PRESSED (PINB & (1<<PINB0))
#define PIN52_PRESSED (PINB & (1<<PINB1))

#define TIME_OUT 10
#define LCD_ADDRESS 0x27
#define LCD_ROWS 20
#define LCD_COLS 4
#define LCD_COL1 4
#define LCD_COL2 10

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
//-----------------------------------------------------------------
unsigned char Load_High_Byte;
unsigned char Load_Low_Byte;
unsigned char Incoming_Byte;
unsigned char Position_Low_Byte;
unsigned char Position_High_Byte;
unsigned char Time_Counter;
unsigned long startTime = millis();
unsigned long interval = 2000; // 10 seconds
int Load_Long_Byte;
int x;
int w;
int Error_Byte;
int Position_Long_Byte;
int smallspring = 24;
int bigspring = 24;
float C1;
float C2;
float loadmass; // Mass for actuator
unsigned long y;// 5 seconds delay
int flag =0;
int minutes = 1000;
int position_old = 0;
int rotations = 0;
byte id = 4;
int angle = 1500;
int currPos = angle;
byte error_byte_old;
int error_counter = 0;
int cycle_counter = 0;
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_ROWS, LCD_COLS);  // set the LCD address to 0x27 for a 20 chars and 4 line display
void setupSwitch() {
	DDRB = 0xFF;
	DDRB &= ~((1<<DDB0) | (1<<DDB1) | (1<<DDB2));
	PCMSK0 |= (1<<PCINT0) | (1<<PCINT1) | (1<<PCINT2);	// Pin Change Mask Register 0. Enable Interrupt on PIN 19, 20 and 21
	//PCIFR |= (1<<PCIE0);	// Pin Change Interrupt Flag Register. Reset PCINT7:0
	PCICR |= (1<<PCIE0);	// Pin Change Interrupt Control Register. Activate PCINT7:0
}
ISR(PCINT0_vect) {
	if (BUTTON_1_PRESSED) { // Speed CHange
		LED_TOGGLE;
		//setEndless(id, OFF);
		//setMaxTorque(id,1023);
		while (BUTTON_1_PRESSED) { 
			moveSpeed(id, currPos>ARM_ID5_ANGLE_MIN ? currPos-1 : ARM_ID5_ANGLE_MIN, 20); 
			delay(200);
		}
	}
	if (BUTTON_2_PRESSED) {
		LED_OFF;
		while (BUTTON_2_PRESSED) { 
			moveSpeed(id, currPos<ARM_ID5_ANGLE_MAX ? currPos+1 : ARM_ID5_ANGLE_MAX, 20);
			delay(200);
		}
	}
	if (BUTTON_3_PRESSED) { // this switch is not working all the time
		LED_ON;
		id = id == 4 ? 5 : 4;
		clearEndless(id);
	}
}

//========================SETUP============================
void setup() {
	//setupSwitch();
	//sei(); // Enable interrupts
	
		LED_OFF;
		DDRB = 0xFF;
		DDRB &= ~((1<<DDB0) | (1<<DDB1) | (1<<DDB2)); // clear DDB0 and DDB4 bits in DDRB. PIN B0 and B4 is set to INPUT (BUTTON)
		PCMSK0 |= (1<<PCINT0) | (1<<PCINT1) | (1<<PCINT2);  // set Pin Change Mask 0 bit PCINT0.
		PCICR  |= (1<<PCIE0);
		sei(); // enable global interrupts
		//while (1) {
		//}
	
	
	
	
	lcd.init();                      // initialize the lcd
	lcd.backlight();
	lcd.clear();
	pinMode(RS485_RX_EN_PIN, OUTPUT);
	pinMode(RS485_TX_EN_PIN, OUTPUT);
	Serial.begin(57600);
	Serial1.begin(57143);
	Serial1.flush();

	lcd.setCursor(0,0);
	lcd.print("sID:");
	lcd.setCursor(0,1);
	lcd.print("deg:");
	lcd.setCursor(0,2);
	lcd.print("rpm:");
	lcd.setCursor(0,3);
	lcd.print("ld%:");
	
	clearEndless(id);

	//moveSpeed(id, angle, 10);
	
	//byte pcktPars[2] = {ARM_ID5_ANGLE_MAX, ARM_ID5_ANGLE_MAX>>8};
	//writeServo(id, MX_CCW_ANGLE_LIMIT_L, pcktPars, 2);
	//readServo(id, 0, 24);
	//printBuffer();
	//setID(id);
	readServo(id, 0x24, 34);
	//reset();
	delay(2);
	//turn(id,LEFT,50);
}
//=========================================================
//========================LOOP=============================
void loop() {
	//------------MOVING THE ARM-----------//
	/* moveSpeed(ServoID, angle: 1024 = 45 degree, speed)
				*/

	//setMaxTorque (0,1023);

	//  x =readLoad(3);
	 
	//delay(2);
	readServo(id, 0x24, 34);
	printDataLCD();
	//delay(200);
}

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
void writeServo(byte pcktID, byte pcktCmnd, byte pcktPar) {

	char Checksum = ~lowByte(pcktID + 4 + MX_INSTRUCTION_WRITE_DATA + pcktCmnd + pcktPar);

	RS485_TX_ON
	Serial1.write(pcktID);						// Servo ID
	Serial1.write(4);							// Length of message (number of Parameters + 3 (1 Command + 2))
	Serial1.write(MX_INSTRUCTION_WRITE_DATA);	// Write message type (write)
	Serial1.write(pcktCmnd);					// Write Command
	Serial1.write(pcktPar);						// Write Parameter
	Serial1.write(Checksum);					// Write Checksum
	RS485_RX_ON
}

/* Write to Servo a command with No Parameters (e.g. Reset) */ 
void writeServo(byte pcktID, byte pcktCmnd) {  

    char Checksum = ~lowByte(pcktID + 2 + MX_INSTRUCTION_WRITE_DATA + pcktCmnd);

    RS485_TX_ON
	Serial1.write(pcktID);						// Servo ID
	Serial1.write(2);							// Length of message (number of Parameters + 3 (1 Command + 2))
	Serial1.write(MX_INSTRUCTION_WRITE_DATA);	// Write message type (write)
	Serial1.write(pcktCmnd);					// Write Command
	Serial1.write(Checksum);					// Write Checksum
    RS485_RX_ON
}

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
	
void resetServo(unsigned char ID) {
	char Checksum = ~lowByte(ID + MX_INSTRUCTION_RESET_LENGTH + MX_INSTRUCTION_RESET);

	RS485_TX_ON
	Serial1.write(ID);							// ID is 0xFE, which broadcast mode (all Servos hear this message)
	Serial1.write(MX_INSTRUCTION_RESET_LENGTH);	// Length of message (number of Parameters + 3 (1 Command + 2))
	Serial1.write(MX_INSTRUCTION_RESET);			// Write message type (write)
	Serial1.write(Checksum);					// Write Checksum
	RS485_RX_ON
}

void resetServo() { resetServo(MX_ALL_SERVOS); }
	
void moveSpeed(byte ID, int Position, int Speed) {
	if (ID == 4) {
		Position = Position > ARM_ID4_ANGLE_MAX ? ARM_ID4_ANGLE_MAX : Position < ARM_ID4_ANGLE_MIN ? ARM_ID4_ANGLE_MIN : Position;
	} else if (ID == 5) {
		Position = Position > ARM_ID5_ANGLE_MAX ? ARM_ID5_ANGLE_MAX : Position < ARM_ID5_ANGLE_MIN ? ARM_ID5_ANGLE_MIN : Position;
	}
  byte parsNo = 4;
  byte pcktPars[parsNo] = {Position, Position >> 8, Speed, Speed >> 8};
  writeServo(ID, MX_GOAL_POSITION_L, pcktPars, parsNo);
  currPos = Position;
}

void rotateon(int r){
  setMaxTorque (0,1023);
  setEndless (0, ON);
  turn (0, r, 100);
}
void rotateoff(int r){
  setMaxTorque (0,1023);
  setEndless (0, OFF);
  turn (0, r, 100);
}
void Downon(){
  setMaxTorque (0,1023);
  setEndless (0, ON);
  turn (0, LEFT, 100);
}
void Downoff(){
  setMaxTorque (0,1023);
  setEndless (0, OFF);
  turn (0, LEFT, 100);
}
void Upon(){
  setMaxTorque (0,1023);
  setEndless (0, ON);
  turn (0, RIGTH, 100);
}
void Upoff(){
  setMaxTorque (0,1023);
  setEndless (0, OFF);
  turn (0, RIGTH, 100);
}
//=========================================================
String* error_decode(byte error_code) {
	String errors[8] = { 
		(error_code & (1<0) ? "Input Voltage Error" : ""),	// 1
		(error_code & (1<1) ? "Angle Limit Error" : ""),	// 2
		(error_code & (1<2) ? "Overheating Error" : ""),	// 4
		(error_code & (1<3) ? "Range Error" : ""),			// 8
		(error_code & (1<4) ? "Checksum Error" : ""),		// 16
		(error_code & (1<5) ? "Overload Error" : ""),		// 32
		(error_code & (1<6) ? "Instruction Error" : "") };	// 64
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
void printLCD1(byte col, byte row, int value) {
	char buffer[1];
	sprintf(buffer, "%1d  ", value);
	lcd.setCursor(col, row);
	lcd.print(buffer);
}
void printLCD2(byte col, byte row, int value) {
	char buffer[2];
	sprintf(buffer, "%2d", value);
	lcd.setCursor(col, row);
	lcd.print(buffer);
}
void printLCD3(byte col, byte row, int value) {
	char buffer[3];
	sprintf(buffer, "%3d", value);
	lcd.setCursor(col, row);
	lcd.print(buffer);
}
void printLCD4(byte col, byte row, int value) {
	char buffer[4];
	sprintf(buffer, "%4d", value);
	lcd.setCursor(col, row);
	lcd.print(buffer);
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
	
	if (Serial1.available() > 0){
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
			} else {
				lcd.print("          ");
			}
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
				printLCD1(LCD_COL1, 0, servoID);
				printLCD4(LCD_COL1, 1, position);
				printLCD4(LCD_COL2, 1, currPos);
				// 			lcd.setCursor(LCD_COL2 ,1);
				// 			lcd.print((int)(currPos));
				//lcd.setCursor(10,1);
				//lcd.print(rotations);
				printLCD3(LCD_COL1, 2, speed * MX_PRESENT_SPEED_RPM);
				lcd.print(speedDirection ? "CW " : "CCW");
				printLCD3(LCD_COL1, 3, load * MX_PRESENT_LOAD_PERCENT);
				lcd.print(loadDirection ? "CW " : "CCW");
				//printLCD3(LCD_COL2, 3, (45*(current-2048)));
			}
			while(Serial1.available() > 0) {
				//Serial.println(Serial1.read());
				Serial1.read();
			}
			
			delay(1);
			cycle_counter = cycle_counter > 9999 ? 0 : cycle_counter + 1;
			printLCD4(16, 0, cycle_counter);
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
void getID(){ readServo(2, 0, 20); }
	
void getID_(){
  int Inco_Byte;
  byte pcktStart = MX_START;
  byte pcktID = 0xFE;
  byte pcktLen = 0x04;
  byte pcktIns = MX_INSTRUCTION_READ_DATA;
  byte pcktPar1 = 0x03;
  byte pcktPar2 = 0x01;
  
  char Checksum = ~lowByte(pcktID + pcktLen + pcktIns + pcktPar1 + pcktPar2);
  
  RS485_TX_ON
  Serial1.write(pcktID); // 3.ID is 0xFE, which broadcast mode (all Servos hear this message)
  Serial1.write(pcktLen); // 4.Length of string (number of parameters + 2)
  Serial1.write(pcktIns); // 5.Ping read data or syncwrite 0x01,2,3,83
  Serial1.write(pcktPar1); // 6.Start address for data to be written (ADDRESS OF ID)(parameter 1)
  Serial1.write(pcktPar2);
  Serial1.write(Checksum); //8. the notchecksum
  RS485_RX_ON
}

void setID(byte newID){ writeServo(MX_ALL_SERVOS, MX_ID, newID); }

void move(unsigned char ID, int Position) {
  char Position_H, Position_L;
  Position_H = Position >> 8; // 16 bits - 2 x 8 bits variables
  Position_L = Position;
  char Checksum = (~(ID + MX_GOAL_LENGTH + MX_INSTRUCTION_WRITE_DATA +
  MX_GOAL_POSITION_L + Position_L + Position_H)) & 0xFF;
  RS485_TX_ON
  Serial1.write(ID); //servo ID
  Serial1.write(MX_GOAL_LENGTH); //length of message(2 + x amount of paramaters)
  Serial1.write(MX_INSTRUCTION_WRITE_DATA); //write data
  Serial1.write(MX_GOAL_POSITION_L); //param2ter 1
  Serial1.write(Position_L); //parameter 2
  Serial1.write(Position_H); //parameter 3
  Serial1.write(Checksum); //checksum
  RS485_RX_ON
}

	
void setEndless(unsigned char ID, bool Status) {
	byte parsNo = 4;
	if (Status == 1) { // Wheel Mode
		byte pcktPars[parsNo] = {0, 0, 0, 0};
		writeServo(ID, MX_CW_ANGLE_LIMIT_L, pcktPars, parsNo);
	} else { //Joint or Multi-turn Mode (if both 4095)
		byte pcktPars[parsNo] = {ARM_ID5_ANGLE_MIN, ARM_ID5_ANGLE_MIN >> 8, ARM_ID5_ANGLE_MAX, ARM_ID5_ANGLE_MAX >> 8};
		writeServo(ID, MX_CW_ANGLE_LIMIT_L, pcktPars, parsNo);  	    
	}
}

void setEndless(unsigned char ID) { setEndless(ID, ON); }
	
void clearEndless(unsigned char ID) { setEndless(ID, OFF); }
	
void turn_(unsigned char ID, bool SIDE, int Speed) {
  if (SIDE == 0) { // Move Left//////////////////////////
    char Speed_H, Speed_L;
    Speed_H = Speed >> 8;
    Speed_L = Speed; // 16 bits - 2 x 8 bits variables
    char Checksum = ~lowByte(ID + MX_SPEED_LENGTH + MX_INSTRUCTION_WRITE_DATA + MX_MOVING_SPEED_L + Speed_L + Speed_H);
    RS485_TX_ON
    Serial1.write(ID);
    Serial1.write(MX_SPEED_LENGTH);
    Serial1.write(MX_INSTRUCTION_WRITE_DATA);
    Serial1.write(MX_MOVING_SPEED_L);
    Serial1.write(Speed_L);
    Serial1.write(Speed_H);
    Serial1.write(Checksum);
    RS485_RX_ON
    // Return the read error
    } else { // Move RIGTH////////////////////
    char Speed_H, Speed_L;
    Speed_H = (Speed >> 8) + 4;
    Speed_L = Speed; // 16 bits - 2 x 8 bits variables
    char Checksum = ~lowByte(ID + MX_SPEED_LENGTH + MX_INSTRUCTION_WRITE_DATA + MX_MOVING_SPEED_L + Speed_L + Speed_H);
    RS485_TX_ON
    Serial1.write(ID);
    Serial1.write(MX_SPEED_LENGTH);
    Serial1.write(MX_INSTRUCTION_WRITE_DATA);
    Serial1.write(MX_MOVING_SPEED_L);
    Serial1.write(Speed_L);
    Serial1.write(Speed_H);
    Serial1.write(Checksum);
    RS485_RX_ON
    // Return the read error
  }
}
void turn(byte ID, bool SIDE, int Speed) { // only available for multiturn mode
  byte Speed_H, Speed_L;
  if (SIDE == 0) { // Move Left//////////////////////////
    Speed_H = Speed >> 8;
    Speed_L = Speed; // 16 bits - 2 x 8 bits variables 
  } else { // Move RIGTH////////////////////
    Speed_H = (Speed >> 8) + 4;
    Speed_L = Speed; // 16 bits - 2 x 8 bits variables
  }
  byte Checksum = ~lowByte(ID + MX_SPEED_LENGTH + MX_INSTRUCTION_WRITE_DATA + MX_MOVING_SPEED_L + Speed_L + Speed_H);
  RS485_TX_ON
  Serial1.write(ID);
  Serial1.write(MX_SPEED_LENGTH);
  Serial1.write(MX_INSTRUCTION_WRITE_DATA);
  Serial1.write(MX_MOVING_SPEED_L);
  Serial1.write(Speed_L);
  Serial1.write(Speed_H);
  Serial1.write(Checksum);
  RS485_RX_ON
  // Return the read error
}
/*
Goal Position: 0 ~ 4095 (0x000 ~ 0xFFF) = 0° ~ 360°, 1 ~ 0.088° 
Moving Speed: 0~2047 (0x000~0x7FF), (Joint Mode), 1 ~ 0.114rpm
              0 -> uses max rpm, no speed control
            1023 (0x3FF) -> ~ 117.07rpm
              0~1023 (0x000~0x3FF) -> CCW direction
              1024~2047 (0x400~0x7FF) -> CW direction
*/
void moveSpeed_(unsigned char ID, int Position, int Speed) {
  byte Position_H, Position_L, Speed_H, Speed_L;
  Position_H = Position >> 8;
  Position_L = Position; // 16 bits - 2 x 8 bits variables
  Speed_H = Speed >> 8;
  Speed_L = Speed; // 16 bits - 2 x 8 bits variables
  byte pcktPars[4] = {Position, Position >> 8, Speed, Speed >> 8};
  byte Checksum = ~lowByte(ID + MX_GOAL_SP_LENGTH + MX_INSTRUCTION_WRITE_DATA + MX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H);
  Serial.println(Checksum);
  Serial.println(Position_L);
  Serial.println(Position_H);
  Serial.println(Speed_L);
  Serial.println(Speed_H);
  Serial.println("*********************");
  RS485_TX_ON
  Serial1.write(ID);
  Serial1.write(4+3);
  //Serial.println(sizeof(pcktPars)+3);
  Serial1.write(MX_INSTRUCTION_WRITE_DATA);
  Serial1.write(MX_GOAL_POSITION_L);/*
  Serial1.write(Position_L);
  Serial1.write(Position_H);
  Serial1.write(Speed_L);
  Serial1.write(Speed_H);*/

  Serial1.write(pcktPars,4);
  Serial1.write(Checksum);
  RS485_RX_ON
}
void setMaxTorque(unsigned char ID, int MaxTorque) {
  char MaxTorque_H, MaxTorque_L;
  MaxTorque_L = MaxTorque;
  MaxTorque_H = MaxTorque >> 8; // 16 bits - 2 x 8 bits variables

  char Checksum = ~lowByte(ID + MX_MT_LENGTH + MX_INSTRUCTION_WRITE_DATA + MX_MAX_TORQUE_L + MaxTorque_L + MaxTorque_H);
  RS485_TX_ON
  Serial1.write(ID);
  Serial1.write(MX_MT_LENGTH);
  Serial1.write(MX_INSTRUCTION_WRITE_DATA);
  Serial1.write(MX_MAX_TORQUE_L);
  Serial1.write(MaxTorque_L);
  Serial1.write(MaxTorque_H);
  Serial1.write(Checksum);
  RS485_RX_ON
}
void torqueStatus( unsigned char ID, bool Status) {
  char Checksum = ~lowByte(ID + MX_TORQUE_LENGTH + MX_INSTRUCTION_WRITE_DATA + MX_TORQUE_ENABLE + Status);
  RS485_TX_ON
  Serial1.write(ID);
  Serial1.write(MX_TORQUE_LENGTH);
  Serial1.write(MX_INSTRUCTION_WRITE_DATA);
  Serial1.write(MX_TORQUE_ENABLE);
  Serial1.write(Status);
  Serial1.write(Checksum);
  RS485_RX_ON
}
/* Extra functions */
int sumBytes(byte* bytes, byte parsNo) {
	int sum = 0;
	for (byte i=0; i<parsNo; i++) sum +=bytes[i];
	return sum;
}