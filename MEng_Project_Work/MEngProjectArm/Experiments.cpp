/*
 * Experiments.cpp
 *
 * Created: 03/12/2019 16:50:19
 *  Author: Marek Kujawa
 */ 
#include <Arduino.h>
#include "Main.h"
#include "Experiments.h"
#include "Arm.h"
#include "MX-64AR.h"

void exampleFunc() {
	//int a, b, c;
	//a = 100;
	//b = 345;
	//c = sqrt(a * b) * a / sqrt(b)  ;
	delay(1001);
}

void timeTheRuntime()  {
	Arm arm;
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
	Arm arm;
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
	Arm arm;
	uint32_t t_start = millis();
	/* Take end-effector to the very low position slowly and output data to Serial port */
	Serial.println((String)(millis() - t_start)+". Go to E=(200,-380) Starts");
	arm.goToXY(200,-380);
	
	for (uint8_t i=0; i<50; i++) {
		getMoreData(4, MX_PRESENT_POSITION_L); /* Update Servo 4 */
		getMoreData(5, MX_PRESENT_POSITION_L); /* Update Servo 5 */
		dumpTheDataToSerial();
		delay(100);
	}
	Serial.println((String)(millis() - t_start)+". Go to E=(200,-380) Ends");
	delay(2000);
	
	/* move Servo 4 up slowly and output data to Serial port */
	Serial.println((String)(millis() - t_start)+". Move S5 to 150 Starts");
	arm.moveSpeedEasy(4, 150, 0.5);
	for (uint8_t i=0; i<140; i++) {
		getMoreData(4, MX_PRESENT_POSITION_L); /* Update Servo 4 */
		getMoreData(5, MX_PRESENT_POSITION_L); /* Update Servo 5 */
		dumpTheDataToSerial();
		delay(100);
	}
	Serial.println((String)(millis() - t_start)+". Move S5 to 150 Ends");
	delay(5000);
}
