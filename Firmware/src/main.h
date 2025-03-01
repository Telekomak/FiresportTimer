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

#define TBMAP_SIZE 13

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>
#include "../lib/TimerBus/timer_bus.h"
#include "HD44780_LCD.h"

//static void debug(uint8_t value);

//TIMER FUNCTIONS
void timer_init();
void timer_start();
void timer_stop();
void timer_reset();
void timer_event();

//UI FUNCTIONS
void display_init();
void update_timer_status();
void update_target_time();

//Timer status register
typedef union{
    struct{
        uint8_t start:1;
        uint8_t stop:1;
        uint8_t reset:1;
        uint8_t left_down:1;
        uint8_t right_down:1;
        uint8_t countdown:1;
        uint8_t reserved:2;
    }bit;
    uint8_t reg;
}TSREG;

typedef union {
    struct{
        TSREG status;
        uint32_t time;
        uint32_t left_time;
        uint32_t right_time;
    }vars;
    uint8_t array[sizeof(TBMAP_SIZE)];
}TBMAP;

//VARIABLES
volatile uint8_t target_latch = 0;
volatile uint8_t last_input_state = 0xFF;
TBMAP tb_map;

#endif //FIRMWARE_MAIN_H
