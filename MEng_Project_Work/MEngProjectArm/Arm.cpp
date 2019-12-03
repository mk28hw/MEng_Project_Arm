/*
 * Arm.cpp
 *
 * Created: 03/12/2019 12:49:50
 *  Author: Marek Kujawa
 */ 

#include <Arduino.h>
#include "MX-64AR.h"
#include "Arm.h"

void Arm::writeServo(uint8_t pcktID, uint8_t pcktCmnd, uint8_t* pcktPars, uint8_t parsNo) {
		uint8_t Checksum = ~lowByte(pcktID + parsNo + 3 + MX_INSTRUCTION_WRITE_DATA + pcktCmnd + sumBytes(pcktPars, parsNo));

		//serialWriting = YES;
		RS485_TX_ON
		Serial1.write(pcktID);						// Servo ID
		Serial1.write(parsNo + 3);					// Length of message (number of Parameters + 3 (1 Command + 2))
		Serial1.write(MX_INSTRUCTION_WRITE_DATA);	// Write message type (write)
		Serial1.write(pcktCmnd);					// Write Command
		Serial1.write(pcktPars, parsNo);			// Write Parameters
		Serial1.write(Checksum);					// Write Checksum
		RS485_RX_ON
		//serialWriting = NO;
}

/* Write to Servo a command with only one Parameter */
void Arm::writeServo(uint8_t pcktID, uint8_t pcktCmnd, uint8_t pcktPar) { this->writeServo(pcktID, pcktCmnd, &pcktPar, 1); }
/* Get Real Value from Raw Value. Works for Speed and Load */
int Arm::getRealValue(int rawValue) { return (rawValue > 0x3FF) ? rawValue - 0x400 : rawValue; }
/* Get Direction from Raw Value. Works for Speed and Load */
bool Arm::getDirection(int rawValue) { return (rawValue > 0x3FF) ? CW : CCW; }
/* Update position of servo with given angle as 0 ~ 2048 value. Where 0 = 0° and 2048 = 180° */
void Arm::updateServoPosition(uint8_t id, int position) {
	this->servos[id].position = position;
	this->servos[id].position_rad = position * CONVERT_POSITION_RADIANS;
	this->servos[id].position_deg = position * CONVERT_POSITION_DEGREE;
}
void Arm::updateServoPosition(uint8_t id, uint8_t position_L, uint8_t position_H) {
	this->updateServoPosition(id, combineBytes(position_L, position_H));
}
/* Update position of servo with given angle in radians */
void Arm::updateServoPosition_rad(uint8_t id, int position_rad) {
	this->servos[id].position = position_rad * CONVERT_RADIANS_POSITION;
	this->servos[id].position_rad = position_rad;
	this->servos[id].position_deg = position_rad * CONVERT_RADIANS_DEGREE;
}
/* Update position of servo with given angle in degrees */
void Arm::updateServoPosition_deg(uint8_t id, int position_deg) {
	this->servos[id].position = position_deg * CONVERT_DEGREE_RADIANS;
	this->servos[id].position_rad = position_deg * CONVERT_DEGREE_RADIANS;
	this->servos[id].position_deg = position_deg;
}

void Arm::updateServoSpeed(uint8_t id, uint16_t speed) {
	this->servos[id].speed = this->getRealValue(speed);
	this->servos[id].speedDirection = this->getDirection(speed);
}
//void updateServoSpeed(uint8_t id, uint8_t speed_L, uint8_t speed_H) { updateServoSpeed(id, combineBytes(speed_L, speed_H)); }

void Arm::updateServoLoad(uint8_t id, uint16_t load) {
	this->servos[id].load = this->getRealValue(load);
	this->servos[id].loadDirection = this->getDirection(load);
}
void Arm::updateServoLoad(uint8_t id, uint8_t load_L, uint8_t load_H) { this->updateServoLoad(id, combineBytes(load_L, load_H)); }
	
/* Set Torque Enable for servo with given ID */
void Arm::setTorqueEnable(uint8_t id, bool status) { writeServo(id, MX_TORQUE_ENABLE, status); }

/* Set Maximum Torque of servo with given ID */
void Arm::setMaxTorque(uint8_t id, int MaxTorque) {
	uint8_t parsNo = 2;
	uint8_t pcktPars[parsNo] = {MaxTorque, MaxTorque >> 8};
	writeServo(id, MX_MAX_TORQUE_L, pcktPars, parsNo);
}

/* Set Torque Limit of servo with given ID */
void Arm::setTorqueLimit(uint8_t id, int TorqueLimit) {
	uint8_t parsNo = 2;
	uint8_t pcktPars[parsNo] = {TorqueLimit, TorqueLimit >> 8};
	writeServo(id, MX_TORQUE_LIMIT_L, pcktPars, parsNo);
}

/* Set new ID to a single servo connected to the system */
void Arm::setID(uint8_t newID) { writeServo(MX_ALL_SERVOS, MX_ID, newID); }

/* Set new ID to a servo with given ID */
void Arm::setID(uint8_t id, uint8_t newID) { writeServo(id, MX_ID, newID); }
		
/*  Set the servo with given ID to different modes:
    0 - Joint Mode
    1 - Wheel Modes
    2 - Multi-Turn Mode */
void Arm::setMode(uint8_t id, uint8_t mode) {
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
	if (id<6 && mode<3) { this->servos[id].mode = mode; }
}
/* Set the servo with given ID to endless => wheel mode */
void Arm::setModeWheel(uint8_t id) { setMode(id, MX_MODE_WHEEL); }
/* Set the servo with given ID to joint mode (0 ~ 360°) */
void Arm::setModeJoint(uint8_t id) { setMode(id, MX_MODE_JOINT); }
/* Set the servo with given ID to multi-turn mode (-(7*360°) ~ +(7*360°)) */
void Arm::setModeMultiTurn(uint8_t id) { setMode(id, MX_MODE_MULTI); }

/* Reset the Servo with given ID */
void Arm::resetServo(uint8_t id) {
	uint8_t Checksum = ~lowByte(id + MX_INSTRUCTION_RESET_LENGTH + MX_INSTRUCTION_RESET);

	//serialWriting = YES;
	RS485_TX_ON
	Serial1.write(id);							// ID is 0xFE, which broadcast mode (all Servos hear this message)
	Serial1.write(MX_INSTRUCTION_RESET_LENGTH);	// Length of message (number of Parameters + 3 (1 Command + 2))
	Serial1.write(MX_INSTRUCTION_RESET);		// Write message type (write)
	Serial1.write(Checksum);					// Write Checksum
	RS485_RX_ON
	//serialWriting = NO;
}

/* Reset All Servos */
void Arm::resetServo() { this->resetServo(MX_ALL_SERVOS); }

/* Request message from Servo using a command with a Parameter */
void Arm::readServo(uint8_t pcktID, uint8_t pcktCmnd, uint8_t resLength) {
	uint8_t Checksum = ~lowByte(pcktID + 4 + MX_INSTRUCTION_READ_DATA + pcktCmnd + resLength);
	RS485_TX_ON

	//serialWriting = YES;
	Serial1.write(pcktID);					// Servo ID
	Serial1.write(4);						// Length of message (number of Parameters + 3 (1 Command + 2))
	Serial1.write(MX_INSTRUCTION_READ_DATA);// Write message type (read)
	Serial1.write(pcktCmnd);				// Write Command
	Serial1.write(resLength);				// Response Length (usually 1 but for some might be 2 or more (more data))
	Serial1.write(Checksum);				// Write Checksum
	//serialWriting = NO;
	RS485_RX_ON
}
void Arm::readServo(uint8_t pcktID, uint8_t pcktCmnd) { this->readServo(pcktID, pcktCmnd, 1); }

/*	Move servo with given ID to given Position at given Speed.
	Position:
		Joint Mode: 0 ~ 4095 (0x000 ~ 0xFFF) = 0° ~ 360°, 1 ? 0.088°
		Multi-Turn: -28672 ~ 28672
	Speed: 0 ~ 2047 (0x0090 ~ 0x7FF), 1 ? 0.11444rpm
		Joint Mode / Multi-Turn:
			0 => Maximum rpm is used without controlling the speed
			0 ~ 1023 (0x3FF) => 0 ~ 117.07rpm
		Wheel Mode:
			0 ~ 1023 (0x000 ~ 0x3FF) => 0 ~ 117.07rpm CCW direction
			1024 ~ 2047 (0x400 ~ 0x7FF) => 0 ~ 117.07rpm CW direction
*/
void Arm::moveSpeed(uint8_t id, uint16_t Position, uint16_t Speed) {
	Position = this->checkPosition(id, Position);
	//Speed = checkSpeed(id, Speed);
	uint8_t parsNo = 4;
	uint8_t pcktPars[parsNo] = {Position, Position >> 8, Speed, Speed >> 8};
	this->writeServo(id, MX_GOAL_POSITION_L, pcktPars, parsNo);
	this->updateServoPosition(id, Position);
	//this->servos[id].speed = getRealValue(Speed);
	//this->servos[id].speedDirection = getDirection(Speed);
}

/*	Move servo with given ID to given Position in Degrees at given Speed in Percentage.
	Position:
		Joint Mode: 0° ~ 360°
*/
void Arm::moveSpeedEasy(uint8_t id, int PositionDeg, float SpeedPer) {
	this->moveSpeed(id, PositionDeg * CONVERT_DEGREE_POSITION, (int) (SpeedPer * MX_CONVERT_PERCENT_SPEED));
}
	
/*	Move servo with given ID to given Position.
	Position:
		Joint Mode: 0 ~ 4095 (0x000 ~ 0xFFF) = 0° ~ 360°, 1 ? 0.088°
		Multi-Turn: -28672 ~ 28672 = -2520° ~ 2520°, 1 ? 0.088°
*/
void Arm::move(uint8_t id, int Position) {
	Position = checkPosition(id, Position);
	uint8_t parsNo = 2;
	uint8_t pcktPars[parsNo] = {Position, Position >> 8};
	writeServo(id, MX_GOAL_POSITION_L, pcktPars, parsNo);
	updateServoPosition(id, Position);
}

/*  Move servo with given ID at given Speed.
	Speed: 0 ~ 2047 (0x000 ~ 0x7FF), 1 ? 0.11444rpm
		Joint Mode / Multi-Turn:
			0 => Maximum rpm is used without controlling the speed
			0 ~ 1023 (0x3FF) => 0 ~ 117.07rpm
		Wheel Mode:
			0 ~ 1023 (0x000 ~ 0x3FF) => 0 ~ 117.07rpm CCW direction
			1024 ~ 2047 (0x400 ~ 0x7FF) => 0 ~ 117.07rpm CW direction
*/
void Arm::turn(uint8_t id, bool side, int Speed) {
	uint8_t parsNo = 2;
	uint8_t pcktPars[parsNo] = {Speed, (Speed >> 8) + (side ? 0 : 4)};
	writeServo(id, MX_MOVING_SPEED_L, pcktPars, parsNo);
	this->servos[id].speed = getRealValue(Speed);
	this->servos[id].speedDirection = getDirection(Speed);
	//currSpeed = Speed;
}

/*	Move servo with given ID to given Position.
	Position:
		Joint Mode: 0 ~ 4095 (0x000 ~ 0xFFF) = 0° ~ 360°, 1 ? 0.088°
		Multi-Turn: -28672 ~ 28672 = -2520° ~ 2520°, 1 ? 0.088°
*/

/* Function checking if the position is within set limits */
int Arm::checkPosition(uint8_t id, uint16_t Position) {
	if (id < 3) { // Wheel Mode Unlimited
		return Position > ARM_ID1_ANGLE_MAX
			? ARM_ID1_ANGLE_MAX
			: Position < ARM_ID1_ANGLE_MIN
			? ARM_ID1_ANGLE_MIN
			: Position;
	} else if (id == 3) { // Multi-Turn up to 7 turns each direction
		/* -28672 ~ 28672 */
		return Position > ARM_ID3_ANGLE_MAX
			? ARM_ID3_ANGLE_MAX
			: Position < ARM_ID3_ANGLE_MIN
			? ARM_ID3_ANGLE_MIN
			: Position;
	} else if (id == 4) { // Joint Mode up to one turn
		/* 0 ~ 4095 */
		if (this->fkX(Position*CONVERT_POSITION_RADIANS, this->servos[5].position*CONVERT_POSITION_RADIANS) < 100) {
			return this->servos[4].position > ARM_ID4_ANGLE_MAX
				? ARM_ID4_ANGLE_MAX
				: this->servos[4].position < ARM_ID4_ANGLE_MIN
				? ARM_ID4_ANGLE_MIN
				: this->servos[4].position;
		} else {
			return Position > ARM_ID4_ANGLE_MAX
				? ARM_ID4_ANGLE_MAX
				: Position < ARM_ID4_ANGLE_MIN
				? ARM_ID4_ANGLE_MIN
				: Position;
		}
	} else if (id == 5) { // Joint Mode up to one turn
		/* 0 ~ 4095 */
		return Position > ARM_ID5_ANGLE_MAX
			? ARM_ID5_ANGLE_MAX
			: Position < ARM_ID5_ANGLE_MIN
			? ARM_ID5_ANGLE_MIN
			: Position;
	} else {
		return -1;
	}
}
bool Arm::checkAngle(uint8_t id, uint16_t Position) {
	uint16_t  position_min, position_max;
	if (id == 1) {
		position_min = ARM_ID1_ANGLE_MIN;
		position_max = ARM_ID1_ANGLE_MAX;
	} else if (id == 2) {
		position_min = ARM_ID2_ANGLE_MIN;
		position_max = ARM_ID2_ANGLE_MAX;
	} else if (id == 3) {
		position_min = ARM_ID3_ANGLE_MIN;
		position_max = ARM_ID3_ANGLE_MAX;
	} else if (id == 4) {
		position_min = ARM_ID4_ANGLE_MIN;
		position_max = ARM_ID4_ANGLE_MAX;
	} else if (id == 5) {
		position_min = ARM_ID4_ANGLE_MIN;
		position_max = ARM_ID4_ANGLE_MAX;
	} else {
		return false;
	}
		
	return (Position > position_min) && (Position < position_max) ? true : false;

}

/* Request and Capture given Data from servo with given ID */
int Arm::getData(uint8_t id, uint8_t ctrlData) {
	uint8_t msgByte;
	bool startOne = NO, startTwo = NO, msgStarted, msgOK = NO;
	uint8_t byteCount = 0;
	uint8_t msgId, msgLength, msgError, msgChecksum, Checksum, msgData_1, msgData_2;
	int msgData;
	String line;

	readServo(id, ctrlData, 2);
	delay(5);
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

	/*  Check if the returned data is not corrupted and there are no errors
			Return -255 if the data is corrupted (Checksum error)
			Return -ErrorCode if there is error from servo */
	return msgOK ? msgError ? -msgError : msgData : -255;
}
	
/* Forward Kinematics of the Tool Position (Pose), X value */
int Arm::fkX(float S4pos_rad, float S5pos_rad) {
	float angle1, angle2;
	// Correction for custom reference frames
	angle1 = S4pos_rad - M_PI_2 + S5pos_rad - M_PI;
	angle2 = S4pos_rad - M_PI_2;
	this->toolPoseX  = (ARM_LINK_3 * cos(angle1)) + (ARM_LINK_2 * cos(angle2));
	return this->toolPoseX;
}
int Arm::fkX() { fkX(this->servos[4].position_rad, this->servos[5].position_rad); }
	
/* Forward Kinematics of the Tool Position (Pose), X value */
int Arm::fkY(float S4pos_rad, float S5pos_rad) {
	float angle1, angle2;
	// Correction for custom reference frames
	angle1 = S4pos_rad - M_PI_2 + S5pos_rad - M_PI;
	angle2 = S4pos_rad - M_PI_2;
	this->toolPoseY  = (ARM_LINK_3 * sin(angle1)) + (ARM_LINK_2 * sin(angle2));
	return this->toolPoseY;
}
int Arm::fkY() { fkY(this->servos[4].position_rad, this->servos[5].position_rad); }
		
/* Inverse Kinematics for Servo 4 */
float Arm::ikAngleServo4(int16_t x, int16_t y) {
	float fraction1, fraction2;
	float nominator;
	float denominator;
	uint32_t squarePosesSum = square(x) + square(y);
	fraction1   = (float)y / (float)x;
	nominator   = arm_link_2_square - arm_link_3_square + squarePosesSum;
	denominator = 2 * (float)ARM_LINK_2 * sqrt(squarePosesSum);
	fraction2   = nominator / denominator;
	ikAngleS4   = atan(fraction1) + acos(fraction2) + M_PI_2;
	return ikAngleS4;
}
float Arm::ikAngleServo4() { ikAngleServo4( this->toolPoseX, this->toolPoseY); }
		
float Arm::ikAngleServo5(int16_t x, int16_t y) {
	double fraction;
	int32_t nominator;
	int32_t denominator;
	nominator   = square(x) + square(y) - arm_link_2_square - arm_link_3_square;
	denominator = arm_link_2_x_arm_link_3_x_2;
	fraction     = (float) nominator / (float) denominator;
	ikAngleS5   = acos(fraction);
	return M_PI - ikAngleS5;
}
float Arm::ikAngleServo5() { ikAngleServo5(this->toolPoseX, this->toolPoseY); }
	
void Arm::calculateAngles(float angle_1_goal, float angle_2_goal) {
	float angle_1_now, angle_2_now, angle_1_dif, angle_2_dif, denominator;
	angle_1_now = (this->getData(4, MX_PRESENT_POSITION_L))*CONVERT_POSITION_RADIANS;
	angle_2_now = (this->getData(5, MX_PRESENT_POSITION_L))*CONVERT_POSITION_RADIANS;
	angle_1_dif = abs(angle_1_goal - angle_1_now);
	angle_2_dif = abs(angle_2_goal - angle_2_now);
	denominator = angle_1_dif > angle_2_dif ? angle_1_dif : angle_2_dif;
	this->ratioServo4 = angle_1_dif/denominator;
	this->ratioServo5 = angle_2_dif/denominator;
}	
void Arm::calculate(int x, int y) { this->calculateAngles(this->ikAngleServo4(x, y), this->ikAngleServo5(x, y)); }
/* Take both Servos (4 and 5) to 90° position */
void Arm::goHome(float speed_per) {
	this->calculateAngles(M_PI_2, M_PI_2);
	this->moveSpeedEasy(5, 90, speed_per*ratioServo5);
	this->moveSpeedEasy(4, 90, speed_per*ratioServo4);
}	
void Arm::goHome() { goHome(2); }
	
int8_t Arm::goToXY(int16_t x, int16_t y) {
	float position_S4, position_S5;
	if (x<ARM_TOOL_MIN_X || y<ARM_TOOL_MIN_Y) {
		return -1;
	}
	position_S4 = this->ikAngleServo4(x, y);
	position_S5 = this->ikAngleServo5(x, y);
	this->calculateAngles(position_S4, position_S5);
	position_S4 = position_S4*CONVERT_RADIANS_POSITION;
	position_S5 = position_S5*CONVERT_RADIANS_POSITION;		
	if (this->checkAngle(4, position_S4)) {
		if (this->checkAngle(5, position_S5)) {
			this->moveSpeed(5, position_S5, 0.5*MX_CONVERT_PERCENT_SPEED);
			this->moveSpeed(4, position_S4, 0.5*MX_CONVERT_PERCENT_SPEED);
			return 0;
		} else { return -5; }
	} else { return -4; }
}
//