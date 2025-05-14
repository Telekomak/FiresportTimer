#ifndef FIRMWARE_TIMER_BUS_H
#define FIRMWARE_TIMER_BUS_H

#define IO_BUFFER_LENGTH 128

#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define TBMAP_SIZE 14

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

//Timer status register
//Is read-only from uart
typedef union{
    struct{
        uint8_t running             :1;
        uint8_t left_down           :1;
        uint8_t right_down          :1;
        uint8_t count_mode          :1;//0-countdown, 1-countup
        uint8_t start_pin_edge      :1;
        uint8_t target_pin_config   :1;
        uint8_t reserved    :2;
    }bit;
    uint8_t reg;
}TSREG;

//Timer interrupt register
//Is write-only* from uart
//*Reading this register returns irrelevant information
typedef union{
    struct{
        uint8_t start       :1;
        uint8_t stop        :1;
        uint8_t reset       :1;
        uint8_t left_down   :1;
        uint8_t right_down  :1;
        uint8_t count_up    :1;
        uint8_t count_down  :1;
        uint8_t reserved    :1;
    }bit;
    uint8_t reg;
}TINTREG;

typedef union {
    struct{
        TSREG status;
        TINTREG external_interrupts;
        uint32_t time;
        uint32_t left_time;
        uint32_t right_time;
    }vars;
    uint8_t array[TBMAP_SIZE];
}TBMAP;

//VARIABLES
volatile TBMAP tb_map;

void tb_init();
void tb_service();

#endif //FIRMWARE_TIMER_BUS_H