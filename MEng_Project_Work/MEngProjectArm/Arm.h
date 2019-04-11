/*
 * Arm.h
 *
 * Created: 03/04/2019 11:11:56
 * Author: Marek Kujawa
 */ 


#ifndef ARM_H_
#define ARM_H_

#define ARM_ID1_ANGLE_MIN 0	    // exactly 0 = 0° (deg)
#define ARM_ID1_ANGLE_MAX 1700	// exactly 1762 = 157° (deg)

#define ARM_ID3_ANGLE_MIN -2048 // exactly -2048 = -180° (deg) [servo] = -180° / 3 = -60° [YAW]
#define ARM_ID3_ANGLE_MAX 2048	// exactly -2048 = 180° (deg) [servo] = 180° / 3 = 60° [YAW]

#define ARM_ID4_ANGLE_MIN 740	// exactly 752 = 65° (deg) [PITCH1]
#define ARM_ID4_ANGLE_MAX 1700	// exactly 1762 = 157° (deg) [PITCH1]

#define ARM_ID5_ANGLE_MIN 740	// exactly 749 = 65° (deg) [PITCH2]
#define ARM_ID5_ANGLE_MAX 1700	// exactly 1744 = 155° (deg) [PITCH2]

struct Servo {
	uint8_t id;
	uint8_t mode;
	int position;
	bool direction;
	int8_t turns;
	int speed;
	int load;
};
struct Arm {
	uint8_t id;
	bool autoBalance = OFF;
	struct Servo servos[6];
};

#endif /* ARM_H_ */