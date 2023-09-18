#ifndef HD44780_LCD_H
#define HD44780_LCD_H

#ifndef F_CPU
	#define F_CPU 16000000u
#endif

#ifndef LCD_DELAY
	#define LCD_DELAY 1500
#endif

#include <avr/io.h>
#include <util/delay.h>

typedef struct PinConfig{
	volatile uint8_t* ddr;
	volatile uint8_t* port;
	uint8_t rs;
	uint8_t en;
	uint8_t d0;
	uint8_t d1;
	uint8_t d2;
	uint8_t d3;
} PinConfig;

int LCD_init(PinConfig* config);
void LCD_instruction(uint8_t instruction);
void LCD_write_char(char character);
void LCD_clear();
void LCD_set_cursor(uint8_t row, uint8_t collumn);
void LCD_write_string(char* string);
void LCD_write_buffer(char* buffer, uint16_t length);
void LCD_pulse_en();
void LCD_pulse_en_repeat(int repeat);
void LCD_show_cursor();
void LCD_hide_cursor();
void LCD_on();
void LCD_off();
void LCD_home();
void LCD_cursor_blink();
void LCD_cursor_noblink();
#endif