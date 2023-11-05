//prolific driver: https://www.miklor.com/COM/UV_Drivers.php

#define F_CPU 16000000UL

#define USART_PACKET_TYPE 0xC0
#define USART_PACKET_TYPE_COMMAND 0x80
#define USART_PACKET_TYPE_DATA 0x40
#define USART_PACKET_TYPE_STATUS 0xC0
#define USART_PACKET_TYPE_HELLO 0x00

#define TIMER_PIN_START 16
#define TIMER_PIN_STOP 32
#define TIMER_PIN_RESET 64

#define TIMER_PIN_LEFT 4
#define TIMER_PIN_RIGHT 8

#define PCMSK PCMSK2 //Enables interrupt on pins
#define CONTROL_PORT PORTD
#define CONTROL_DDR DDRD
#define CONTROL_PIN PIND

#define LCD_PORT PORTB
#define LCD_DDR DDRB

//Status = valid pins for any given situation
#define TIMER_TARGETS (TIMER_PIN_LEFT | TIMER_PIN_RIGHT)
#define TIMER_STATUS_RUNNING TIMER_PIN_STOP
#define TIMER_STATUS_STOPPED (TIMER_PIN_RESET | TIMER_PIN_START)

//Flags for main loop
#define TIMER_CONTROL_START 1
#define TIMER_CONTROL_STOP 2
#define TIMER_CONTROL_RESET 4
#define TIMER_CONTROL_LEFT_DOWN 8
#define TIMER_CONTROL_RIGHT_DOWN 16
#define TIMER_CONTROL_USART_WRITE_STATUS 32

#include "HD44780_LCD.h"
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>

//static void debug(uint8_t value);

//TIMER FUNCTIONS
static void timer_setup();
static void timer_start();
static void timer_stop();
static void timer_reset();
static void timer_event();

//UI FUNCTIONS
static PinConfig* lcd_setup();
static void display_init();
static void update_timer_status();
static void update_target_time();

//USART FUNCTIONS
static void usart_setup();
static void usart_command(uint8_t command);
static void usart_write(uint8_t data);
static void usart_write_buffer(uint8_t length);

static volatile uint32_t time = 0;
static volatile uint32_t left_time = 0;
static volatile uint32_t right_time = 0;
static volatile uint8_t timer_control = 0; //Flags for main loop events
static volatile uint8_t target_status = 0;
static volatile uint8_t last_input_state = 0xFF;

static PinConfig *lcd_config;
static uint8_t *usart_tx_buffer;
static char *left_time_str;
static char *right_time_str;

int main(void)
{
    left_time_str = calloc(9, 1);
    right_time_str = calloc(9, 1);

	cli();
	timer_setup();

    lcd_config = lcd_setup();
    LCD_init(lcd_config);
    LCD_on();
	LCD_clear();
	LCD_home();

    usart_setup();
    sei();

    display_init();

	while(1)
	{
        if(timer_control) timer_event();
        if((time & 0x8) && (PCMSK == TIMER_STATUS_RUNNING)) update_target_time();
	}
	
	free(lcd_config);
	return 0;
}

static void usart_setup()
{
    //9600
    UBRR0L = 103;

    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);

    usart_tx_buffer = calloc(5, 1);
}

//START OF UI CODE
static void display_init()
{
    LCD_clear();

    LCD_set_cursor(0, 0);
    LCD_write_string("L: 00:00:00");

    LCD_set_cursor(1, 0);
    LCD_write_string("P: 00:00:00");

    update_timer_status();
}

static void update_target_time()
{
    if(target_status & TIMER_PIN_RIGHT) sprintf(right_time_str, "%02d:%02d:%02d", (uint16_t)(time / 6000), (uint16_t)((time / 100) % 60), (uint16_t)(time % 100));
    if(target_status & TIMER_PIN_LEFT) sprintf(left_time_str, "%02d:%02d:%02d", (uint16_t)(time / 6000), (uint16_t)((time / 100) % 60), (uint16_t)(time % 100));
    //Alternative super fast solution:
    //sprintf(str, "%02d:%02d:%02d", (uint16_t)(time >> 13), (uint16_t)((time >> 7) % 0x3F), (uint16_t)(time & 0x7F));

    LCD_set_cursor(0, 3);
    LCD_write_string(left_time_str);

    LCD_set_cursor(1, 3);
    LCD_write_string(right_time_str);
}

static void update_timer_status()
{
    if (PCMSK == TIMER_STATUS_STOPPED)
    {
        LCD_set_cursor(0, 14);
        LCD_write_string("||");
    }
    else
    {
        LCD_set_cursor(0, 14);
        LCD_write_string("> ");
    }
}

static PinConfig* lcd_setup()
{
    PinConfig* result = calloc(sizeof(PinConfig), 1);
    result -> port = &LCD_PORT;
    result -> ddr = &LCD_DDR;

    result -> rs = 1;
    result -> en = 2;
    result -> d0 = 4;
    result -> d1 = 8;
    result -> d2 = 16;
    result -> d3 = 32;

    return result;
}
//END OF UI CODE

static void timer_event()
{
    if(timer_control & TIMER_CONTROL_START)
    {
        usart_write(USART_PACKET_TYPE_COMMAND | TIMER_CONTROL_START);
        update_timer_status();
        timer_control &= ~TIMER_CONTROL_START;

        if(!(timer_control)) return;
    }
    if(timer_control & TIMER_CONTROL_STOP)
    {
        usart_write(USART_PACKET_TYPE_COMMAND | TIMER_CONTROL_STOP);
        update_timer_status();
        timer_control &= ~TIMER_CONTROL_STOP;

        update_target_time();

        if(!(timer_control)) return;
    }
    if(timer_control & TIMER_CONTROL_RESET)
    {
        usart_write(USART_PACKET_TYPE_COMMAND | TIMER_CONTROL_RESET);
        display_init();
        timer_control &= ~TIMER_CONTROL_RESET;

        if(!(timer_control)) return;
    }
    if(timer_control & TIMER_CONTROL_LEFT_DOWN)
    {
        usart_tx_buffer[0] = USART_PACKET_TYPE_DATA | TIMER_CONTROL_LEFT_DOWN;
        memcpy(usart_tx_buffer + 1, &left_time, 4);
        usart_write_buffer(5);

        timer_control &= ~TIMER_CONTROL_LEFT_DOWN;

        sprintf(left_time_str, "%02d:%02d:%02d", (uint16_t)(left_time / 6000), (uint16_t)((left_time / 100) % 60), (uint16_t)(left_time % 100));

        if (PCMSK == TIMER_STATUS_STOPPED) update_target_time();
        if(!(timer_control)) return;
    }
    if(timer_control & TIMER_CONTROL_RIGHT_DOWN)
    {
        usart_tx_buffer[0] = USART_PACKET_TYPE_DATA | TIMER_CONTROL_RIGHT_DOWN;
        memcpy(usart_tx_buffer + 1, &right_time, 4);
        usart_write_buffer(5);

        timer_control &= ~TIMER_CONTROL_RIGHT_DOWN;

        sprintf(right_time_str, "%02d:%02d:%02d", (uint16_t)(right_time / 6000), (uint16_t)((right_time / 100) % 60), (uint16_t)(right_time % 100));

        if (PCMSK == TIMER_STATUS_STOPPED) update_target_time();
        if(!(timer_control)) return;
    }
    if(timer_control & TIMER_CONTROL_USART_WRITE_STATUS)
    {
        //PORTB |= 0x20;
        usart_write(USART_PACKET_TYPE_STATUS | target_status | PCMSK);
        timer_control &= ~TIMER_CONTROL_USART_WRITE_STATUS;
    }
}

static void timer_setup()
{
    //TARGET_PINS
    EICRA = ISC01 | ISC11;
    EIMSK = 0;

	//CONTROL PINS:
	CONTROL_DDR = ~(TIMER_PIN_START | TIMER_PIN_STOP | TIMER_PIN_RESET | TIMER_TARGETS);
	CONTROL_PORT = TIMER_PIN_START | TIMER_PIN_STOP | TIMER_PIN_RESET | TIMER_TARGETS;

	//TIMER:
    //10ms: Prescaler = 1024 | OCR0A = 155
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS00) | (1 << CS02);
	OCR0A = 155;
	
	//INTERRUPTS:
	PCICR = (1 << PCIE2);//enable pin change interrupt 2

	PCMSK = TIMER_STATUS_STOPPED;
    target_status = TIMER_TARGETS;
}

static inline void timer_start()
{
    PCMSK = TIMER_STATUS_RUNNING;
    EIMSK = target_status;
    timer_control |= TIMER_CONTROL_START;

	//enable compare match interrupt
	TIMSK0 |= 0x02;

}

static inline void timer_stop()
{
    PCMSK = TIMER_STATUS_STOPPED;
    EIMSK = 0;
    timer_control |= TIMER_CONTROL_STOP;

	//disable compare match interrupt
	TIMSK0 &= ~0x02;
}

static inline void timer_reset()
{
    if(CONTROL_PIN & TIMER_PIN_LEFT && CONTROL_PIN & TIMER_PIN_RIGHT)
    {
        target_status = TIMER_TARGETS;
        timer_control |= TIMER_CONTROL_RESET;
        time = 0;
	    right_time = 0;
	    left_time = 0;
        PCMSK = TIMER_STATUS_STOPPED;
        EIMSK = 0;
    }
}

ISR(PCINT0_vect)
{
    //PCMSK holds valid pins for current status
    //Invert PIN because pullup
	switch (((~CONTROL_PIN) & PCMSK) & last_input_state)
	{
        case TIMER_PIN_START:
            timer_start();
            break;

        case TIMER_PIN_STOP:
            timer_stop();
            break;

        case TIMER_PIN_RESET:
			timer_reset();
			break;

		default: break;
	}

    //last_input_state = CONTROL_PIN;
}

static void usart_command(uint8_t command)
{
    switch (command)
    {
        case TIMER_CONTROL_START:
            timer_start();
            timer_control |= TIMER_CONTROL_START;
            return;

        case TIMER_CONTROL_STOP:
            timer_stop();
            return;

        case TIMER_CONTROL_RESET:
            timer_reset();
            timer_control |= TIMER_CONTROL_RESET;
            return;

        default:
            return;
    }
}

static void usart_write(uint8_t data)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

static void usart_write_buffer(uint8_t length)
{
    for (int i = 0; i < length; i++)
    {
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = usart_tx_buffer[i];
    }
}

ISR(TIMER0_COMPA_vect)
{
	time++;
}

ISR(USART_RX_vect)
{
    uint8_t udr = UDR0;

    switch (udr & USART_PACKET_TYPE)
    {
        case USART_PACKET_TYPE_COMMAND:
            usart_command(udr & ~USART_PACKET_TYPE);
            break;

        case USART_PACKET_TYPE_HELLO:
            timer_control |= TIMER_CONTROL_USART_WRITE_STATUS;
            break;

        default:
            break;
    }
}

ISR(INT0_vect)
{
    EIMSK &= ~TIMER_PIN_LEFT;
    target_status = EIMSK;
    left_time = time;
    timer_control |= TIMER_CONTROL_LEFT_DOWN;

    if(!EIMSK) timer_stop();
}

ISR(INT1_vect)
{
    EIMSK &= ~TIMER_PIN_RIGHT;
    target_status = EIMSK;
    right_time = time;
    timer_control |= TIMER_CONTROL_RIGHT_DOWN;

    if(!EIMSK) timer_stop();
}
/*
static void debug(uint8_t value)
{
    char str[3];
    sprintf(str, "%x|", value);
    LCD_write_string(str);
}*/