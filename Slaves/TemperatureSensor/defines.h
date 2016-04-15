#ifndef DEFINES_H
#define DEFINES_H

//Status LED's
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED0 PB0

#define LDR_PIN PC0

#define SLAVE_ADDRESS 0x15

//RS485 drivers control
#define RS485_DDR DDRD
#define RS485_PORT PORTD
#define RS485_CTRL PD4
//Protocol bytes
#define START_BYTE 0x96
#define STOP_BYTE 0xA9

//TODO: create constants.h
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

//Ring buffer size
#define UART_RX_BUFFER_SIZE 16
#define UART_TX_BUFFER_SIZE 16

#endif
