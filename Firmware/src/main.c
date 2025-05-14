#include "main.h"

//static void debug(uint8_t value);

const PinConfig lcd_config = {
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

TINTREG internal_interrupts;
volatile uint8_t target_latch;
volatile uint8_t last_input_state = 0;

const uint8_t target_config = 1;

int main(void)
{
    memset(tb_map.array, 0, TBMAP_SIZE);
    memset(left_time_str, 0, 9);
    memset(right_time_str, 0, 9);
    internal_interrupts.reg = 0;

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
        if(internal_interrupts.reg) timer_internal_ISR();
        //if(tb_map.vars.external_interrupts.reg) timer_external_ISR();
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
    if(!tb_map.vars.status.bit.left_down) sprintf(left_time_str, "%02d:%02d:%02d", (uint16_t)(tb_map.vars.time / 6000), (uint16_t)((tb_map.vars.time / 100) % 60), (uint16_t)(tb_map.vars.time % 100));
    if(!tb_map.vars.status.bit.right_down) sprintf(right_time_str, "%02d:%02d:%02d", (uint16_t)(tb_map.vars.time / 6000), (uint16_t)((tb_map.vars.time / 100) % 60), (uint16_t)(tb_map.vars.time % 100));

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

void timer_internal_ISR()
{
    if(internal_interrupts.bit.start)
    {
        update_timer_status();
        internal_interrupts.bit.start = 0;

        if(!internal_interrupts.reg) return;
    }
    if(internal_interrupts.bit.stop)
    {
        update_timer_status();
        internal_interrupts.bit.stop = 0;

        update_target_time();

        if(!internal_interrupts.reg) return;
    }
    if(internal_interrupts.bit.reset)
    {
        display_init();
        internal_interrupts.bit.reset = 0;

        if(!internal_interrupts.reg) return;
    }
    if(internal_interrupts.bit.left_down)
    {
        internal_interrupts.bit.left_down = 0;

        sprintf(left_time_str, "%02d:%02d:%02d", (uint16_t)(tb_map.vars.left_time / 6000), (uint16_t)((tb_map.vars.left_time / 100) % 60), (uint16_t)(tb_map.vars.left_time % 100));

        if(PCMSK == TIMER_STATUS_STOPPED) update_target_time();
        if(!internal_interrupts.reg) return;
    }
    if(internal_interrupts.bit.right_down)
    {
        internal_interrupts.bit.right_down = 0;

        sprintf(right_time_str, "%02d:%02d:%02d", (uint16_t)(tb_map.vars.right_time / 6000), (uint16_t)((tb_map.vars.right_time / 100) % 60), (uint16_t)(tb_map.vars.right_time % 100));

        if(PCMSK == TIMER_STATUS_STOPPED) update_target_time();
        //if(!internal_interrupts.reg) return;
    }
}

void timer_external_ISR()
{
    if(tb_map.vars.external_interrupts.bit.start && (PCMSK == TIMER_STATUS_STOPPED))
    {
        tb_map.vars.external_interrupts.bit.start = 0;
        timer_start();

        if(!tb_map.vars.external_interrupts.reg) return;
    }
    if(tb_map.vars.external_interrupts.bit.stop && (PCMSK == TIMER_STATUS_RUNNING))
    {
        tb_map.vars.external_interrupts.bit.stop = 0;
        timer_stop();

        if(!tb_map.vars.external_interrupts.reg) return;
    }
    if(tb_map.vars.external_interrupts.bit.reset && (PCMSK == TIMER_STATUS_STOPPED))
    {
        tb_map.vars.external_interrupts.bit.reset = 0;
        timer_reset();

        if(!tb_map.vars.external_interrupts.reg) return;
    }
    if(tb_map.vars.external_interrupts.bit.left_down && (EIMSK & TIMER_PIN_EXINT0))
    {
        tb_map.vars.external_interrupts.bit.left_down = 0;
        timer_exint0();

        if(!tb_map.vars.external_interrupts.reg) return;
    }
    if(tb_map.vars.external_interrupts.bit.right_down && (EIMSK & TIMER_PIN_EXINT0))
    {
        tb_map.vars.external_interrupts.bit.right_down = 0;
        timer_exint1();

        //if(!tb_map.vars.external_interrupts.reg) return;
    }
}

void timer_init()
{
    //TARGET_PINS
    //EICRA = 1 << ISC01 | 1 << ISC00 | 1 << ISC11 | 1 << ISC10; //Rising edge
    EICRA = 1 << ISC01 | 1 << ISC11; //Falling edge
    EIMSK = 0;
    target_latch = TIMER_EXINTS;

	//CONTROL PINS:
	CONTROL_DDR = (uint8_t)~(TIMER_PIN_START | TIMER_PIN_STOP | TIMER_PIN_RESET | (TIMER_EXINTS << 2));
	CONTROL_PORT = TIMER_EXINTS << 2;

	//TIMER:
    //10ms: Prescaler = 1024 | OCR0A = 155
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS00) | (1 << CS02);
	OCR0A = 155;
    TCNT0 = 0;
	
	//INTERRUPTS:
	PCICR = (1 << PCIE2);//enable pin change interrupt 2
	PCMSK = TIMER_STATUS_STOPPED;
}

void timer_start()
{
    EIFR = TIMER_EXINTS;//clear external interrupt flags
    EIMSK = target_latch;
    PCMSK = TIMER_STATUS_RUNNING;
    internal_interrupts.bit.start = 1;
    tb_map.vars.status.bit.running = 1;

    //enable compare match interrupt
	TIMSK0 |= (1<<OCIE0A);
}

void timer_stop()
{
    PCMSK = TIMER_STATUS_STOPPED;
    EIMSK = 0;
    internal_interrupts.bit.stop = 1;
    tb_map.vars.status.bit.running = 0;

    //disable compare match interrupt
	TIMSK0 &= ~(1<<OCIE0A);
}

void timer_reset()
{
    if((CONTROL_PIN & (TIMER_PIN_EXINT0 << 2)) && (CONTROL_PIN & (TIMER_PIN_EXINT1 << 2)))
    {
        target_latch = TIMER_EXINTS;
        internal_interrupts.bit.reset = 1;
        tb_map.vars.time = 0;
	    tb_map.vars.right_time = 0;
	    tb_map.vars.left_time = 0;
        tb_map.vars.status.bit.left_down = 0;
        tb_map.vars.status.bit.right_down = 0;
        tb_map.vars.status.bit.running = 0;
        PCMSK = TIMER_STATUS_STOPPED;
        EIMSK = 0;
    }
}

void timer_exint0()
{
    EIMSK &= ~TIMER_PIN_EXINT0;
    target_latch = EIMSK;

    if(target_config)
    {
        tb_map.vars.right_time = tb_map.vars.time;
        internal_interrupts.bit.right_down = 1;
        tb_map.vars.status.bit.right_down = 1;
    }
    else
    {
        tb_map.vars.left_time = tb_map.vars.time;
        internal_interrupts.bit.left_down = 1;
        tb_map.vars.status.bit.left_down = 1;
    }

   if(target_latch == 0) timer_stop();
}

void timer_exint1()
{
    EIMSK &= ~TIMER_PIN_EXINT1;
    target_latch = EIMSK;

    if (target_config)
    {
        tb_map.vars.left_time = tb_map.vars.time;
        internal_interrupts.bit.left_down = 1;
        tb_map.vars.status.bit.left_down = 1;
    }
    else
    {
        tb_map.vars.right_time = tb_map.vars.time;
        internal_interrupts.bit.right_down = 1;
        tb_map.vars.status.bit.right_down = 1;
    }

   if(target_latch == 0) timer_stop();
}

ISR(PCINT2_vect)
{
    //PCMSK holds valid pins for current internal_interrupts
    //Invert PIN because pullup
	switch ((CONTROL_PIN & PCMSK) & ~last_input_state)
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

		default:
            break;
	}

    last_input_state = (CONTROL_PIN & PCMSK);
}

ISR(INT0_vect)
{
    timer_exint0();
}

ISR(INT1_vect)
{
    timer_exint1();
}

ISR(TIMER0_COMPA_vect)
{
    tb_map.vars.time++;
}

/*
static void debug(uint8_t value)
{
    char str[2];
    sprintf(str, "%x", value);
    LCD_write_string(str);
}
 */