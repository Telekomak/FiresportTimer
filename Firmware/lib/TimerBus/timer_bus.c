#define RX_ENABLE UCSR0B |= (1 << RXEN0)
#define RX_DISABLE UCSR0B &= (1 << RXEN0)

#include "timer_bus.h"
#include "crc_table.h"

static void uart_init();
static void timeout_timer_init();
static void timeout_timer_start();
static void timeout_timer_stop();
static void uart_process_request();
static uint8_t calculate_crc(uint8_t *data, uint8_t length);
static void write_rx_err();
static void write_crc_error();
static TB_RESPONSE write_registers(uint8_t index, uint8_t length, uint8_t *data);
static TB_RESPONSE read_registers(uint8_t index, uint8_t length, uint8_t *out_buffer);

uint8_t input_buffer[IO_BUFFER_LENGTH];
uint8_t output_buffer[IO_BUFFER_LENGTH];
uint8_t input_buffer_length;
uint8_t output_buffer_length;
uint8_t output_buffer_index;
UartStatus status;

void tb_init()
{
    memset(input_buffer, 0, IO_BUFFER_LENGTH);
    memset(output_buffer, 0, IO_BUFFER_LENGTH);
    input_buffer_length = 0;
    output_buffer_index = 0;
    output_buffer_length = 0;
    memset(&status, 0, 1);
    status.rx_complete = 1;

    uart_init();
    timeout_timer_init();
}

void tb_service()
{
    if(status.tx_complete)
    {
        RX_ENABLE;
        status.tx_complete = 0;
    }

    if(status.rx_complete)
    {
        uart_process_request();
        status.rx_complete = 0;
        status.tx_new_data = 1;
    }

    if(UCSR0A & (1<<UDRE0) && status.tx_new_data)
    {
        UDR0 = output_buffer[output_buffer_index++];
        status.tx_new_data = 0;
    }
}

static void uart_process_request()
{
    uint8_t input_crc;
    TimerBusHeader input_header, output_header;
    TB_RESPONSE output_response;

    if(status.rx_err)//send error
    {
        status.rx_err = 0;
        write_rx_err();
        return;
    }

    input_header.value = input_buffer[0];
    if((input_buffer_length - 1) != input_header.bits.remaining_bytes)
    {
        write_rx_err();
        return;
    }

    input_crc = calculate_crc(input_buffer, input_buffer_length - 1);
    if(input_crc != input_buffer[input_buffer_length - 1])
    {
        write_crc_error();
        return;
    }

    output_header.bits.function = input_header.bits.function;

    if (input_header.bits.function)//write
    {
        output_response = write_registers(input_buffer[1], input_buffer[2], &input_buffer[3]);
        if (output_response)
            output_header.bits.error = 1;

        output_header.bits.remaining_bytes = 2;
    }
    else
    {
        output_response = read_registers(input_buffer[1], input_buffer[2], &output_buffer[1]);
        if (output_response)
            output_header.bits.error = 1;

        output_header.bits.remaining_bytes = input_buffer[2] + 3;
    }

    output_buffer[0] = output_header.value;

    output_buffer_length = output_header.bits.remaining_bytes + 1;
    output_buffer[output_buffer_length - 1] = calculate_crc(output_buffer, output_buffer_length - 1);
    output_buffer_index = 0;
}

static void write_rx_err()
{
    output_buffer[0] = 0b10000010;
    output_buffer[1] = TB_RX_ERR;
    output_buffer[2] = calculate_crc(output_buffer, 2);
    output_buffer_length = 3;
    output_buffer_index = 0;
}

static void write_crc_error()
{
    output_buffer[0] = 0b10000010;
    output_buffer[1] = TB_CRC_ERR;
    output_buffer[2] = calculate_crc(output_buffer, 2);
    output_buffer_length = 3;
    output_buffer_index = 0;
}

static TB_RESPONSE write_registers(uint8_t index, uint8_t length, uint8_t *data)
{
    if(index >= TBMAP_SIZE) return TB_UNKNOWN_ADDRESS;
    if(index + length > TBMAP_SIZE) return TB_OVERFLOW;

    memcpy(&index, data, 1);
    memcpy(&tb_map.array[index], data + 1, length);
    return TB_OK;
}

static TB_RESPONSE read_registers(uint8_t index, uint8_t length, uint8_t *out_buffer)
{
    if(index >= TBMAP_SIZE) return TB_UNKNOWN_ADDRESS;
    if(index + length > TBMAP_SIZE) return TB_OVERFLOW;

    memcpy(out_buffer, &tb_map.array[index], length);
    return TB_OK;
}

static uint8_t calculate_crc(uint8_t *data, uint8_t length)
{
    uint8_t crc = 0xFF;

    for (int i = 0; i < length; ++i)
        crc = crc_table[crc^data[i]];

    return crc;
}

static void uart_init()
{
    //9600
    UBRR0L = 103;

    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
}

static void timeout_timer_init()
{
    //2ms: Prescaler = 1024 | OCR2A = 32
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS00) | (1 << CS02);
    OCR2A = 32;
}

static void timeout_timer_start()
{
    status.rx_complete = 0;

    TCNT2 = 0;
    TIMSK2 |= (1<<OCIE2A);
}

static void timeout_timer_stop()
{
    status.rx_complete = 1;

    RX_DISABLE;
    TIMSK2 &= ~(1<<OCIE2A);
}

ISR(USART_RX_vect)
{
    timeout_timer_start();

    if(input_buffer_length >= IO_BUFFER_LENGTH) input_buffer[input_buffer_length++] = UDR0;
    else
    {
        uint8_t tmp = UDR0;
        status.rx_err = 1;
    }
}

ISR(USART_TX_vect)
{
    if(output_buffer_index >= output_buffer_length)
    {
        status.tx_complete = 1;
        return;
    }

    UDR0 = output_buffer[output_buffer_index++];
}

ISR(TIMER2_COMPA_vect)
{
    timeout_timer_stop();
}