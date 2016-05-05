#ifndef DEFINES_H
#define DEFINES_H

#include <avr/io.h>

//TODO: bunch of defines
#define F_CPU 14745600UL

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

//Ring buffer size
#define UART_RX_BUFFER_SIZE 32
#define UART_TX_BUFFER_SIZE 32

#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED1 PB0
#define LED2 PB1

#define RELAY_DDR DDRC
#define RELAY_PORT PORTC
#define RELAY1 PC7
#define RELAY2 PC6

//RS485 drivers control
#define RS485_DDR DDRD
#define RS485_PORT PORTD
#define RS485_CTRL1 PD5 //uart0(PC)
#define RS485_CTRL2 PD4 //uart1(slaves)

//Protocol Defines
#define SLAVE_ADDRESS 0xFF //Master is itself slave to PC

typedef enum {
    //add more
    ASCII
}DATA_TYPE;

//TODO: add more
//STATE MACHINE
typedef enum {
    STATE_IDLE,
    STATE_FETCH_ADDRESS,
    STATE_MASTER_FRAME,
    STATE_SLAVE_FRAME,
    STATE_SLAVE_SEND,
    STATE_SLAVE_RECEIVE,
    STATE_MASTER_COMMAND,//parse command
    STATE_TOGGLE_RELAY,
    STATE_WRITE_EEPROM,
    STATE_READ_EEPROM,
    STATE_SEND_RESPONSE
}FRAME_STATE;

/*
 * For putting actions
 * Alarm/Timer events:
 * 1. 
 * Alarm events go to priority queue
 * Timer events go to 
 * Triggering events:
 *
 **/
typedef enum {
    CMD_RELAY = 0x00,
    CMD_GET_TIME,
    CMD_RD_CFG,
    CMD_WR_CFG,
    CMD_PUT_ACTION
    //CMD_
} COMMAND_TYPE;

//typedef enum {

//}

typedef struct {
    uint8_t id;
    uint8_t type;
}sAction ;


static volatile FRAME_STATE state;

//RS485 PROTOCOL BYTES
#define START_BYTE 0x96
#define STOP_BYTE 0xA9
#endif

