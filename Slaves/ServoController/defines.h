#ifndef DEFINES_H
#define DEFINES_H

/* experiment with these values */
#define SERVO_0DEG 9732
#define SERVO_30DEG 13356
#define SERVO_45DEG 15746
#define SERVO_60DEG 17645
#define SERVO_90DEG 22110
#define SERVO_120DEG 26542
/*#define SERVO_135DEG 28232 //Riba ant 120laipsniu?
#define SERVO_150DEG 26542
#define SERVO_180DEG 32735*/

//Status LED's
#define LED_DDR DDRD
#define LED_PORT PORTD
#define LED0 PD6

#define SLAVE_ADDRESS 0x16

//RS485 drivers control
#define RS485_DDR DDRD
#define RS485_PORT PORTD
#define RS485_CTRL PD2
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
