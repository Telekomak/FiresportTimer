#ifndef FIRMWARE_TIMER_BUS_H
#define FIRMWARE_TIMER_BUS_H

#define IO_BUFFER_LENGTH 128

#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

typedef struct{
    uint8_t rx_complete :1;
    uint8_t tx_complete :1;
    uint8_t tx_new_data :1;
    uint8_t rx_err      :1;
    uint8_t reserved    :4;
}UartStatus;

typedef union{
    struct {
        uint8_t error: 1; //0-ok 1-error
        uint8_t function: 1; //0-read 1-write
        uint8_t remaining_bytes: 6;
    }bits;
    uint8_t value;
}TimerBusHeader;

typedef enum{
    TB_OK = 0,
    TB_RX_ERR = 1,
    TB_UNKNOWN_ADDRESS = 2,
    TB_OVERFLOW = 3,
    TB_CRC_ERR = 4
}TB_RESPONSE;

extern TBMAP tb_map;

void tb_init();
void tb_service();

#endif //FIRMWARE_TIMER_BUS_H