//TODO: bunch of defines
#define F_CPU 14745600UL

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

#define START_BYTE 0x96
#define STOP_BYTE 0xA9
typedef enum {
    //add more
    ASCII
}DATA_TYPE;

//TODO: add more
typedef enum {
    STATE_START,
    STATE_FRAME_START,
    STATE_MASTER_FRAME,
    STATE_SLAVE_FRAME,
    STATE_SLAVE_SEND,
    STATE_SLAVE_RECEIVE,
    STATE_MASTER_COMMAND
}FRAME_STATE;

FRAME_STATE state = STATE_START;

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "uart.h"

//TODO:
//Relay control
//Status Leds
//Both uarts
//more...
int main (void){

    //Set LED pins to Output
    LED_DDR |= (1 << LED1) | (1 << LED2);
    LED_PORT &= ~(1 << LED1) | (1 << LED2);

    //Set RELAY pins to output
    RELAY_DDR |= (1 << RELAY1) | (1 << RELAY2);
    RELAY_PORT &= ~(1 << RELAY1) | (1 << RELAY2);

    //Set RS485 drivers Mode
    //RS485 Drivers Tx/Rx control pin to output
    RS485_DDR |= (1 << RS485_CTRL1) | (1 << RS485_CTRL2);
    //uart0 to receive, uart1 to transmit
    RS485_PORT &= ~(1 << RS485_CTRL1);
    RS485_PORT |= (1 << RS485_CTRL2);

    //enable internal UAR0 and UART1 RX pull-up on Atmega324A
    PORTD |= (1 << PD0);
    PORTD |= (1 << PD2);

    //initialize both uarts
    uart0_init(9600);
    uart1_init(9600);

    //enable global interrupts
    sei();

    //ony byte from RX buffer
    unsigned int cmd; //was unsigned int
    //Frame variables
    uint8_t *response;
    uint8_t dataAddress;
    uint8_t dataLen;
    uint8_t data[32];//Maximum defined data lenght
    uint8_t i;
    unsigned int tempDataByte;

    //Main Loop with Finite State Machine for handling protocol frames
    while(1){
        switch(state){
            case STATE_START:
                //Wait for start byte of protocol frame
                do{cmd = uart0_getc();}while(cmd != START_BYTE);
                state = STATE_FRAME_START;
                break;
            case STATE_FRAME_START:
                //Get adress byte
                do{cmd = uart0_getc();}while(cmd & UART_NO_DATA);
                //Message for master from PC
                if(cmd == SLAVE_ADDRESS){
                    state = STATE_MASTER_FRAME;
                //Or message for slaves
                }else{
                    LED_PORT |= (1 << LED1);
                    state = STATE_SLAVE_FRAME;
                }
                dataAddress = cmd;
                break;
            case STATE_MASTER_FRAME:
                //Get bytes from UART0 until STOP_BYTE
                //get datalen byte
                do{cmd = uart0_getc();}while(cmd & UART_NO_DATA);
                dataLen = cmd;

                i = 0;
                do{
                    //Get each data byte
                    tempDataByte = uart0_getc();
                    if(!(tempDataByte & UART_NO_DATA) && i != dataLen){
                        data[i] = tempDataByte;
                        i++;
                    }

                }while(i < dataLen);

                //get stop byte
                do{cmd = uart0_getc();}while(cmd != STOP_BYTE);

                state = STATE_MASTER_COMMAND;
                break;
            //Get all slave message from UART0
            case STATE_SLAVE_FRAME:
                //get datalen byte
                do{cmd = uart0_getc();}while(cmd & UART_NO_DATA);
                dataLen = cmd;

                i = 0;
                do{
                    //Get each data byte
                    tempDataByte = uart0_getc();
                    if(!(tempDataByte & UART_NO_DATA) && i != dataLen){
                        data[i] = tempDataByte;
                        i++;
                    }

                }while(i < dataLen);

                //get stop byte
                do{cmd = uart0_getc();}while(cmd != STOP_BYTE);
                //TODO: later add XOR checksum message validation

                state = STATE_SLAVE_SEND;
                break;
            //Send message to slave
            case STATE_SLAVE_SEND:
                RS485_PORT |= (1 << RS485_CTRL2);
                //Second time does not sende this data to slave
                uart1_putc(START_BYTE);//temp slave must light led after receiving this
                uart1_putc(dataAddress);
                uart1_putc(dataLen);
                for(i = 0; i < dataLen; i++){
                    uart1_putc(data[i]);
                }
                uart1_putc(STOP_BYTE);
                //TODO: improve
                while(!(UCSR1A & (1 << TXC1)));
                /*_delay_ms(20);//This is shit*/
                //Enable USART1 RX
                RS485_PORT &= ~(1 << RS485_CTRL2);

                state = STATE_SLAVE_RECEIVE;
                break;
            //Receive response from slave
            case STATE_SLAVE_RECEIVE:
                //BUG: After second frame send, stuck here
                //Start byte
                LED_PORT |= (1 << LED2);
                do{cmd = uart1_getc();}while(cmd != START_BYTE);
                //Address
                do{cmd = uart1_getc();}while(cmd & UART_NO_DATA);
                //Datalen
                do{cmd = uart1_getc();}while(cmd & UART_NO_DATA);
                dataLen = cmd;

                LED_PORT &= ~(1 << LED2);
                i = 0;
                do{
                    //Get each data byte
                    tempDataByte = uart1_getc();
                    if(!(tempDataByte & UART_NO_DATA) && i != dataLen){
                        data[i] = tempDataByte;
                        i++;
                    }

                }while(i < dataLen);
                //stop byte
                do{cmd = uart1_getc();}while(cmd != STOP_BYTE);


                //Enable uart0 tx
                RS485_PORT |= (1 << RS485_CTRL1);
                //Send Response
                uart0_putc(START_BYTE);
                uart0_putc(dataAddress);
                uart0_putc(dataLen);
                for(i = 0; i < dataLen; i++){
                    uart0_putc(data[i]);
                }
                uart0_putc(STOP_BYTE);
                while(!(UCSR0A & (1 << TXC0)));
                /*_delay_ms(10);//This is shit*/
                //Reenable RX
                RS485_PORT &= ~(1 << RS485_CTRL1);
                state = STATE_START;

                LED_PORT &= ~(1 << LED1);
                break;
            case STATE_MASTER_COMMAND:

                //TODO: implement master protocol logic
                //TEST: switch relay on
                RELAY_PORT ^= (1 << RELAY1) | (1 << RELAY2);

                state = STATE_START;
                break;
            default:
                /**/
                break;
        }

    }
}
