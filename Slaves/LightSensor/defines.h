#ifndef DEFINES_H
#define DEFINES_H

//This device
#define ATMEGA8

#define F_CPU 14745600UL

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

//Ring buffer size
#define UART_RX_BUFFER_SIZE 16
#define UART_TX_BUFFER_SIZE 16



//SPECIFIC FOR EACH AVR
/******************************/
//RS485 drivers control
#define RS485_DDR DDRD
#define RS485_PORT PORTD
#define RS485_CTRL PD4

//Status LED's
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED0 PB0

#define LDR_PIN 0//PC0

#define SLAVE_ADDRESS 0x15
/******************************/

//STATE MACHINE

typedef enum {
    STATE_IDLE,
    STATE_FETCH_ADDRESS,
    STATE_FETCH_DATA,
    STATE_PROCESSING,
    STATE_SEND_RESPONSE
}FRAME_STATE;

static volatile FRAME_STATE state;

//RS485 PROTOCOL BYTES
#define START_BYTE 0x96
#define STOP_BYTE 0xA9
#endif
