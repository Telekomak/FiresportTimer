#ifndef FIRMWARE_MAIN_H
#define FIRMWARE_MAIN_H

#define F_CPU 16000000U
#define LCD_DELAY 100

#define TIMER_PIN_START 32
#define TIMER_PIN_STOP 64
#define TIMER_PIN_RESET 128

#define TIMER_PIN_LEFT 1
#define TIMER_PIN_RIGHT 2

#define PCMSK PCMSK2 //Enables interrupt on pins
#define CONTROL_PORT PORTD
#define CONTROL_DDR DDRD
#define CONTROL_PIN PIND

#define LCD_PORT PORTB
#define LCD_DDR DDRB
#define LEFT_INT INT0_vect
#define RIGHT_INT INT1_vect

//Status = valid pins for any given situation
#define TIMER_TARGETS (TIMER_PIN_LEFT | TIMER_PIN_RIGHT)
#define TIMER_STATUS_RUNNING TIMER_PIN_STOP
#define TIMER_STATUS_STOPPED (TIMER_PIN_RESET | TIMER_PIN_START)

#include "HD44780_LCD.h"
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>

//static void debug(uint8_t value);

//TIMER FUNCTIONS
void timer_setup();
void timer_start();
void timer_stop();
void timer_reset();
void timer_event();

//UI FUNCTIONS
void display_init();
void update_timer_status();
void update_target_time();

//UART FUNCTIONS
void uart_setup();

//Timer status register
typedef union{
    struct{
    uint8_t start:1;
    uint8_t stop:1;
    uint8_t reset:1;
    uint8_t left_down:1;
    uint8_t right_down:1;
    uint8_t reserved:3;
    }bit;
    uint8_t reg;
}TSREG;


//Flags for main loop
#define TIMER_CONTROL_START 1
#define TIMER_CONTROL_STOP 2
#define TIMER_CONTROL_RESET 4
#define TIMER_CONTROL_LEFT_DOWN 8
#define TIMER_CONTROL_RIGHT_DOWN 16
#define TIMER_CONTROL_USART_WRITE_STATUS 32

#endif //FIRMWARE_MAIN_H
