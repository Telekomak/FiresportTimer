#include "HD44780_LCD.h"

//datasheet: https://www.sparkfun.com/datasheets/LCD/HD44780.pdf

#define LCD_STATE_CURSOR_BLINK 1
#define LCD_STATE_CURSOR_VISIBLE 2
#define LCD_STATE_ON 4

static void LCD_clear_data_pins();
static void LCD_write_value(uint8_t value, uint8_t rs_value);
static uint8_t LCD_verify_config();
 
static uint8_t LCD_current_state;
static PinConfig* LCD_config;

int LCD_init(PinConfig* config)
{
    LCD_config = config;
    LCD_current_state = 8;
	
	if (LCD_verify_config())
	{
		uint8_t ddr_value = (LCD_config -> rs | LCD_config -> en
                             | LCD_config -> d0 | LCD_config -> d1
                             | LCD_config -> d2 | LCD_config -> d3);
		
		//Set LCD pins as output
		*(LCD_config -> ddr) |= ddr_value;
		
		//Set all pins labeled as output to LOW
		*(LCD_config -> port) &= ~ddr_value;
		
		//4-bit mode initialization sequence
		*(LCD_config -> port) |= (LCD_config -> d0 | LCD_config -> d1);
		LCD_pulse_en_repeat(3);

        LCD_clear_data_pins();
		
		*(LCD_config -> port) |= LCD_config -> d1;
		LCD_pulse_en();
	}
	else return 1;
	
	//display config
	LCD_instruction(0x2C);
	LCD_instruction(0x06);
	LCD_instruction(0x08);
	
	return 0;
}

static uint8_t LCD_verify_config()
{
	uint8_t current = 0, previous = 0;
	
	//cycle through all members the LCD_config struct
	//skip first two because they are pointers (pointer is 2 bytes long)
	for (uint8_t i = 2 * sizeof(uint8_t*); i < sizeof(PinConfig); i++)
	{
		//access the LCD_config member on address LCD_config + i
		current |= *(((uint8_t*)LCD_config) + i);
		
		//if nothing has changed, one of the previous iterations has already
		//set the bit to 1, which means that at least two values are the same,
		//or the LCD_config struct member has value of 0
		if (current == previous) return 0;
		previous = current;
	}
	
	return 1;
}

void LCD_pulse_en()
{
	*(LCD_config -> port) |= LCD_config -> en;
	_delay_us(LCD_DELAY);
	*(LCD_config -> port) &= ~LCD_config -> en;
    _delay_us(LCD_DELAY);
}

void LCD_pulse_en_repeat(int repeat)
{
	for (int i = 0; i < repeat; i++) LCD_pulse_en();
}

static void LCD_write_value(uint8_t value, uint8_t rs_value)
{
    LCD_clear_data_pins();
	
	if (rs_value) *(LCD_config -> port) |= LCD_config -> rs;
	
	*(LCD_config -> port) |= value & 0x80 ? LCD_config -> d3 : 0;
	*(LCD_config -> port) |= value & 0x40 ? LCD_config -> d2 : 0;
	*(LCD_config -> port) |= value & 0x20 ? LCD_config -> d1 : 0;
	*(LCD_config -> port) |= value & 0x10 ? LCD_config -> d0 : 0;
	
	LCD_pulse_en();

    LCD_clear_data_pins();
	
	*(LCD_config -> port) |= value & 0x08 ? LCD_config -> d3 : 0;
	*(LCD_config -> port) |= value & 0x04 ? LCD_config -> d2 : 0;
	*(LCD_config -> port) |= value & 0x02 ? LCD_config -> d1 : 0;
	*(LCD_config -> port) |= value & 0x01 ? LCD_config -> d0 : 0;
	
	LCD_pulse_en();
	
	*(LCD_config -> port) &= ~LCD_config -> rs;

    LCD_clear_data_pins();
}

void LCD_instruction(uint8_t instruction)
{
    LCD_write_value(instruction, 0);
}

void LCD_write_char(char character)
{
    LCD_write_value(character, 1);
}

void LCD_write_string(char* string)
{
	for (uint16_t i = 0; string[i] != 0; i++) LCD_write_char(string[i]);
}

void LCD_write_buffer(char* buffer, uint16_t length)
{
	for (uint16_t i = 0; i < length; i++) LCD_write_char(buffer[i]);
}

static void LCD_clear_data_pins()
{
	*(LCD_config -> port) &= ~(LCD_config -> d0 | LCD_config -> d1 | LCD_config -> d2 | LCD_config -> d3);
}

void LCD_clear()
{
	LCD_instruction(1);
}

void LCD_set_cursor(uint8_t row, uint8_t collumn)
{
	LCD_instruction(0x80 + (row? 64 : 0) + (collumn % 40));
}

void LCD_cursor_blink()
{
    LCD_current_state |= LCD_STATE_CURSOR_BLINK;
	LCD_instruction(LCD_current_state);
}

void LCD_cursor_noblink()
{
    LCD_current_state &= ~LCD_STATE_CURSOR_BLINK;
	LCD_instruction(LCD_current_state);
}

void LCD_show_cursor()
{
    LCD_current_state |= LCD_STATE_CURSOR_VISIBLE;
	LCD_instruction(LCD_current_state);
}

void LCD_hide_cursor()
{
    LCD_current_state &= ~LCD_STATE_CURSOR_VISIBLE;
	LCD_instruction(LCD_current_state);
}

void LCD_on()
{
    LCD_current_state |= LCD_STATE_ON;
	LCD_instruction(LCD_current_state);
}

void LCD_off()
{
    LCD_current_state &= ~LCD_STATE_ON;
	LCD_instruction(LCD_current_state);
}

void LCD_home()
{
	LCD_instruction(2);
	//this operation requires 1.52ms delay
	_delay_us(1600 - LCD_DELAY);
}
