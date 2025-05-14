#ifndef FIRMWARE_MAIN_H
#define FIRMWARE_MAIN_H

#define F_CPU 16000000U
#define LCD_DELAY 100

#define TIMER_PIN_START 32
#define TIMER_PIN_STOP 64
#define TIMER_PIN_RESET 128

#define TIMER_PIN_EXINT0 1
#define TIMER_PIN_EXINT1 2

#define PCMSK PCMSK2 //Enables interrupt on pins
#define CONTROL_PORT PORTD
#define CONTROL_DDR DDRD
#define CONTROL_PIN PIND

#define LCD_PORT PORTB
#define LCD_DDR DDRB

//Status = valid pins for any given situation
#define TIMER_EXINTS (TIMER_PIN_EXINT0 | TIMER_PIN_EXINT1)
#define TIMER_STATUS_RUNNING TIMER_PIN_STOP
#define TIMER_STATUS_STOPPED (TIMER_PIN_RESET | TIMER_PIN_START)

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
void timer_internal_ISR();
void timer_external_ISR();
void timer_exint0();
void timer_exint1();

//UI FUNCTIONS
void display_init();
void update_timer_status();
void update_target_time();

#endif //FIRMWARE_MAIN_H
