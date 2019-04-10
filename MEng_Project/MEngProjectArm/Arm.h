/*
 * Arm.h
 *
 * Created: 03/04/2019 11:11:56
 * Author: Marek Kujawa
 */ 


#ifndef ARM_H_
#define ARM_H_

#define ARM_ID3_ANGLE_MIN 0		// exactly 752 = 65° (deg)
#define ARM_ID3_ANGLE_MAX 1700	// exactly 1762 = 157° (deg)

#define ARM_ID4_ANGLE_MIN 740	// exactly 752 = 65° (deg)
#define ARM_ID4_ANGLE_MAX 1700	// exactly 1762 = 157° (deg)

#define ARM_ID5_ANGLE_MIN 740	// exactly 749 = 65° (deg)
#define ARM_ID5_ANGLE_MAX 1700	// exactly 1744 = 155° (deg)

struct Servo {
	uint8_t id;
	uint8_t mode;
	int position;
	uint8_t turns;
	int speed;
	int load;
};
struct Arm {
	uint8_t id;
	struct Servo servos[5];
};

#endif /* ARM_H_ */