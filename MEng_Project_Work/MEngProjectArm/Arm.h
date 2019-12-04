/*
 * Arm.h
 *
 * Created: 03/04/2019 11:11:56
 * Author: Marek Kujawa
 */ 


#ifndef ARM_H_
#define ARM_H_

#include "General.h"

/* Arm dimensions */
#define ARM_LINK_1 400	// Length of Link 1 in mm
#define ARM_LINK_2 215	// Length of Link 2 in mm
#define ARM_LINK_3 310	// Length of Link 3 in mm

/* Servo #1 - Spring #1 */
#define ARM_ID1_ANGLE_MIN 0	    // exactly 0 = 0° (deg)
#define ARM_ID1_ANGLE_MAX 1700	// exactly 1762 = 157° (deg)

#define ARM_ID1_SPEED_MIN 0	    // 0 => Max Speed Without Controlling the Speed
#define ARM_ID1_SPEED_MAX 1023	// 1023 => 117.07 rpm

/* Servo #2 - Spring #2 */
#define ARM_ID2_ANGLE_MIN 0	    // exactly 0 = 0° (deg)
#define ARM_ID2_ANGLE_MAX 1700	// exactly 1762 = 157° (deg)

#define ARM_ID2_SPEED_MIN 0	    // 0 => Max Speed Without Controlling the Speed
#define ARM_ID2_SPEED_MAX 1023	// 1023 => 117.07 rpm

/* Servo #3 - YAW #1 */
#define ARM_ID3_ANGLE_MIN -4096 // exactly -24096 = -360° (deg) [servo] = -360° / 3 = -120° [YAW]
#define ARM_ID3_ANGLE_MAX 4096	// exactly -4096 = 360° (deg) [servo] = 360° / 3 = 120° [YAW]

#define ARM_ID3_SPEED_MIN 1		// 1 => 0.11 rpm
#define ARM_ID3_SPEED_MAX 200	// 200 => 22.89 rpm

/* Servo #4 - PITCH #1 */
#define ARM_ID4_ANGLE_MIN 775	// exactly 752 = 68.2° (deg) [PITCH1]
#define ARM_ID4_ANGLE_MAX 1700	// exactly 1762 = 157° (deg) [PITCH1]

#define ARM_ID4_SPEED_MIN 1		// 1 => 0.11 rpm
#define ARM_ID4_SPEED_MAX 100	// 100 => 11.44 rpm (10%)

/* Servo #5 - PITCH #2 */
#define ARM_ID5_ANGLE_MIN 775	// exactly 749 = 68.2° (deg) [PITCH2]
#define ARM_ID5_ANGLE_MAX 1700	// exactly 1744 = 155° (deg) [PITCH2]

#define ARM_ID5_SPEED_MIN 1		// 1 => 0.11 rpm
#define ARM_ID5_SPEED_MAX 200	// 100 => 22.55 rpm (20%)

#define ARM_TOOL_MIN_X 100		// Minimum value of x coordinates of End-effector
#define ARM_TOOL_MIN_Y -450		// Minimum value of y coordinates of End-effector

/* Setting both of these two pins to: (a) HIGH - enable TX and disable RX, (b) LOW - enable RX and disable TX */
#define RS485_RX_EN_PIN 2 // this is actually disable pin (not enable) for RX
#define RS485_TX_EN_PIN 3 // enable pin for TX

#define RS485_TX_ON \
	while(Serial1.available()) { Serial1.read(); } \
	digitalWrite(RS485_RX_EN_PIN, HIGH);	/* Notify max485 transceiver to accept tx */ \
	digitalWrite(RS485_TX_EN_PIN, HIGH);	/* Notify max485 transceiver to accept tx */ \
	delay(5);								/* Allow this to take effect */ \
	Serial1.write(MX_START);				/* These 2 bytes are 'start message' */ \
	Serial1.write(MX_START);

#define RS485_RX_ON \
	Serial1.flush(); \
	digitalWrite(RS485_RX_EN_PIN, LOW);		/* Notify MAX485 transceiver to receive */ \
	digitalWrite(RS485_TX_EN_PIN, LOW);		/* Notify MAX485 transceiver to receive */ \
	delay(5);


struct Servo {
	uint8_t id;
	uint8_t mode;
	int16_t position;
	float position_rad;
	int16_t position_deg;
	int8_t turns;
	int speed;
	uint16_t speed_default;
	bool speedDirection;
	int load;
	bool loadDirection;
	int current;			/* in mili Ampers */

};
struct Arm {
	int32_t arm_link_2_square = square(ARM_LINK_2);
	int32_t arm_link_3_square = square(ARM_LINK_3);
	int32_t arm_link_2_x_arm_link_3_x_2 = ((int32_t)ARM_LINK_2 * (int32_t)ARM_LINK_3 * 2);
	uint8_t id;
	bool autoBalance = OFF;
	
	bool serialWriting = NO;
	bool serialReading = NO;
	
	struct Servo servos[6];
	int toolPoseX; // in mm 
	int toolPoseY; // in mm in the Joint A frame
	float ratioServo4;
	float ratioServo5;
	float ikAngleS5, ikAngleS4;
	uint8_t joy_x_last, joy_y_last;

	void calculateAngles(float angle_1_goal, float angle_2_goal);
	void calculate(int x, int y);
	int checkPosition(uint8_t id, int16_t Position);
	bool checkAngle(uint8_t id, uint16_t Position);
	int fkX(float S4pos_rad, float S5pos_rad);
	int fkX();
	int fkY(float S4pos_rad, float S5pos_rad);
	int fkY();
	int getDataOne(uint8_t id, uint8_t ctrlData);
	int getData(uint8_t id, uint8_t ctrlData);
	bool getDirection(int rawValue);
	int getRealValue(int rawValue);
	void goHome(float speed_per);
	void goHome();
	int8_t goToXY(int16_t x, int16_t y);
	float ikAngleServo4(int16_t x, int16_t y);
	float ikAngleServo4();
	float ikAngleServo5(int16_t x, int16_t y);
	float ikAngleServo5();
	void moveSpeed(uint8_t id, int16_t Position, uint16_t Speed);
	void moveSpeedEasy(uint8_t id, int16_t PositionDeg, float SpeedPer);
	void move(uint8_t id, int16_t Position);
	void resetServo(uint8_t id);
	void resetServo();
	void readServo(uint8_t pcktID, uint8_t pcktCmnd, uint8_t resLength);
	void readServo(uint8_t pcktID, uint8_t pcktCmnd);
	void setTorqueEnable(uint8_t id, bool status);
	void setMaxTorque(uint8_t id, int MaxTorque);
	void setTorqueLimit(uint8_t id, int TorqueLimit);
	void setID(uint8_t newID);
	void setID(uint8_t id, uint8_t newID);
	void setMode(uint8_t id, uint8_t mode);
	void setModeWheel(uint8_t id);
	void setModeJoint(uint8_t id);
	void setModeMultiTurn(uint8_t id);
	void setMultiTurnDivider(uint8_t id, uint8_t divider);
	void turn(uint8_t id, bool side, int Speed);
	void updateServoPosition(uint8_t id, int16_t position);
	void updateServoPosition(uint8_t id, uint8_t position_L, uint8_t position_H);
	void updateServoPosition_rad(uint8_t id, float position_rad);
	void updateServoPosition_deg(uint8_t id, int16_t position_deg);
	void updateServoSpeed(uint8_t id, uint16_t speed);
	void updateServoLoad(uint8_t id, uint16_t load);
	void updateServoLoad(uint8_t id, uint8_t load_L, uint8_t load_H);
	void writeServo(uint8_t pcktID, uint8_t pcktCmnd, uint8_t* pcktPars, uint8_t parsNo);
	void writeServo(uint8_t pcktID, uint8_t pcktCmnd, uint8_t pcktPar);
};

#endif /* ARM_H_ */
//