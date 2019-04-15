/*
 * LCD.h
 *
 * Created: 09/04/2019 17:36:07
 *  Author: Marek Kujawa
 */ 


#ifndef LCD_H_
#define LCD_H_

#include <LiquidCrystal_I2C.h>

/* LCD Related Defines */
#define LCD_ADDRESS 0x27
#define LCD_COLS 20
#define LCD_COL1 4
#define LCD_COL2 10
#define LCD_COL3 15
#define LCD_ROWS 4
#define LCD_ROW1 0
#define LCD_ROW2 1
#define LCD_ROW3 2
#define LCD_ROW4 3
/* LCD Characters. Use (char)CH_XXX to display */
#define CH_ARL 0b01111111
#define CH_ARR 0b01111110
#define CH_DEG 0b11011111
/* */
#define CH_CHAR_DOWN 0
#define CH_CHAR_UP 1
//#define CH_ARD {B00100, B00100, B00100, B00100, B10101, B01110, B00100, B00000}

#endif /* LCD_H_ */