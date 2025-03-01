#include "main.h"

//#define UART_ECHO

PinConfig lcd_config = {
        .port = &LCD_PORT,
        .ddr = &LCD_DDR,
        .rs = 32,
        .en = 16,
        .d0 = 8,
        .d1 = 4,
        .d2 = 2,
        .d3 = 1
};

char left_time_str[9];
char right_time_str[9];

int main(void)
{
    memset(tb_map.array, 0, sizeof(tb_map.array));
    memset(left_time_str, 0, 9);
    memset(right_time_str, 0, 9);

	cli();
    LCD_init(&lcd_config);
    LCD_on();
	LCD_clear();
	LCD_home();

    timer_init();

    sei();

    display_init();

	while(1)
	{
        if(tb_map.vars.status.reg) timer_event();
        if((tb_map.vars.time & 0x8) && (PCMSK == TIMER_STATUS_RUNNING)) update_target_time();
	}

    return 0;
}

//START OF UI CODE
void display_init()
{
    LCD_clear();

    LCD_set_cursor(0, 0);
    LCD_write_string("L: 00:00:00");

    LCD_set_cursor(1, 0);
    LCD_write_string("P: 00:00:00");

    update_timer_status();
}

void update_target_time()
{
    if(target_latch & TIMER_PIN_RIGHT) sprintf(right_time_str, "%02d:%02d:%02d", (uint16_t)(tb_map.vars.time / 6000), (uint16_t)((tb_map.vars.time / 100) % 60), (uint16_t)(tb_map.vars.time % 100));
    if(target_latch & TIMER_PIN_LEFT) sprintf(left_time_str, "%02d:%02d:%02d", (uint16_t)(tb_map.vars.time / 6000), (uint16_t)((tb_map.vars.time / 100) % 60), (uint16_t)(tb_map.vars.time % 100));

    LCD_set_cursor(0, 3);
    LCD_write_string(left_time_str);

    LCD_set_cursor(1, 3);
    LCD_write_string(right_time_str);
}

void update_timer_status()
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
//END OF UI CODE

void timer_event()
{
    if(tb_map.vars.status.bit.start)
    {
        update_timer_status();
        tb_map.vars.status.bit.start = 0;

        if(!tb_map.vars.status.reg) return;
    }
    if(tb_map.vars.status.bit.stop)
    {
        update_timer_status();
        tb_map.vars.status.bit.stop = 0;

        update_target_time();

        if(!tb_map.vars.status.reg) return;
    }
    if(tb_map.vars.status.bit.reset)
    {
        display_init();
        tb_map.vars.status.bit.reset = 0;

        if(!tb_map.vars.status.reg) return;
    }
    if(tb_map.vars.status.bit.left_down)
    {
        tb_map.vars.status.bit.left_down = 0;

        sprintf(left_time_str, "%02d:%02d:%02d", (uint16_t)(tb_map.vars.left_time / 6000), (uint16_t)((tb_map.vars.left_time / 100) % 60), (uint16_t)(tb_map.vars.left_time % 100));

        if (PCMSK == TIMER_STATUS_STOPPED) update_target_time();
        if(!tb_map.vars.status.reg) return;
    }
    if(tb_map.vars.status.bit.right_down)
    {
        tb_map.vars.status.bit.right_down = 0;

        sprintf(right_time_str, "%02d:%02d:%02d", (uint16_t)(tb_map.vars.right_time / 6000), (uint16_t)((tb_map.vars.right_time / 100) % 60), (uint16_t)(tb_map.vars.right_time % 100));

        if (PCMSK == TIMER_STATUS_STOPPED) update_target_time();
        //if(!tb_map.vars.status.reg) return;
    }
}

void timer_init()
{
    //TARGET_PINS
    EICRA = 1 << ISC01 | 1 << ISC00 | 1 << ISC11 | 1 << ISC10; //Rising edge
    EIMSK = 0;

	//CONTROL PINS:
	CONTROL_DDR = ~(TIMER_PIN_START | TIMER_PIN_STOP | TIMER_PIN_RESET | (TIMER_TARGETS << 2));
	CONTROL_PORT = TIMER_PIN_START | TIMER_PIN_STOP | TIMER_PIN_RESET | (TIMER_TARGETS << 2);

	//TIMER:
    //10ms: Prescaler = 1024 | OCR0A = 155
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS00) | (1 << CS02);
	OCR0A = 155;
    TCNT0 = 0;
	
	//INTERRUPTS:
	PCICR = (1 << PCIE2);//enable pin change interrupt 2

	PCMSK = TIMER_STATUS_STOPPED;
    target_latch = TIMER_TARGETS;
}

inline void timer_start()
{
    PCMSK = TIMER_STATUS_RUNNING;
    EIMSK = target_latch;
    tb_map.vars.status.bit.start = 1;

	//enable compare match interrupt
	TIMSK0 |= (1<<OCIE0A);
}

inline void timer_stop()
{
    PCMSK = TIMER_STATUS_STOPPED;
    EIMSK = 0;
    tb_map.vars.status.bit.stop = 1;

	//disable compare match interrupt
	TIMSK0 &= ~(1<<OCIE0A);
}

inline void timer_reset()
{
    if(!(CONTROL_PIN & (TIMER_PIN_LEFT << 2)) && !(CONTROL_PIN & (TIMER_PIN_RIGHT << 2)))
    {
        target_latch = TIMER_TARGETS;
        tb_map.vars.status.bit.reset = 1;
        tb_map.vars.time = 0;
	    tb_map.vars.right_time = 0;
	    tb_map.vars.left_time = 0;
        PCMSK = TIMER_STATUS_STOPPED;
        EIMSK = 0;
    }
}

ISR(PCINT2_vect)
{
    //PCMSK holds valid pins for current tb_map.vars.status
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

    last_input_state = CONTROL_PIN;
}

ISR(TIMER0_COMPA_vect)
{
	tb_map.vars.time++;
}

#ifdef UART_ECHO
ISR(USART_RX_vect)
{
    char data = UDR0;
    while ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = data;
}
#else
ISR(USART_RX_vect)
{
    char data = UDR0;
    while ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = data;
}
#endif

ISR(LEFT_INT)
{
    EIMSK &= ~TIMER_PIN_LEFT;
    //target_latch &= ~TIMER_PIN_LEFT;
    target_latch = EIMSK;
    tb_map.vars.left_time = tb_map.vars.time;
    tb_map.vars.status.bit.left_down = 1;

    if(!EIMSK) timer_stop();
}

ISR(RIGHT_INT)
{
    EIMSK &= ~TIMER_PIN_RIGHT;
    //target_latch &= ~TIMER_PIN_RIGHT;
    target_latch = EIMSK;
    tb_map.vars.right_time = tb_map.vars.time;
    tb_map.vars.status.bit.right_down = 1;

    if(!EIMSK) timer_stop();
}

/*
static void debug(uint8_t value)
{
    char str[3];
    sprintf(str, "%x|", value);
    LCD_write_string(str);
}*/