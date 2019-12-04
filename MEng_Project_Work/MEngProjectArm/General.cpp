/*
 * General.cpp
 *
 * Created: 03/12/2019 12:44:31
 *  Author: Marek Kujawa
 */ 
#include <Arduino.h>
#include "General.h"

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

int16_t combineBytes(uint8_t byte_L, uint8_t byte_H) { return byte_L + (byte_H<<8); }

int16_t sumBytes(uint8_t* bytes, uint8_t parsNo) {
	int sum = 0;
	for (uint8_t i=0; i<parsNo; i++) sum +=bytes[i];
	return sum;
}

String combineToCSV(int* data, uint8_t number) {
	uint8_t i;
	String line = padNumber(data[0], 10);
	for (i=1; i < number; i++) { line = line + "," + padNumber(data[i], 6);}
	return line;
}

void dumpDataToSerial(int* data, uint8_t number) { Serial.println(combineToCSV(data, number)); }

uint32_t timeIt(void (*functionToBeTimed) ()) {
	uint32_t t_start = millis();
	(*functionToBeTimed) ();
	uint32_t t_now = millis();
	return t_now - t_start;
}

void printSerial(String title, int value) {
	Serial.print(title);
	Serial.print(": ");
	Serial.println(value);
}
//
