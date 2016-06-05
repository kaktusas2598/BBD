#define F_CPU 14745600UL

#define RS485_RXTX_EN PB1
#define STATUS_LED PB2

#define SLAVE_ADDRESS 0x14

#define START_BYTE 0x96
#define STOP_BYTE 0xA9

typedef enum {
    STATE_IDLE,
    STATE_FETCH_ADDRESS,
    STATE_FETCH_DATA,
    STATE_SEND_RESPONSE
}FRAME_STATE;

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "ds18b20.h"
#include "uart.h"

volatile uint8_t lastWholeTemp;
volatile uint16_t lastDecimalTemp;

void sendResponse(char * buffer, int len){
    PORTB |= (1 << RS485_RXTX_EN);

    for(uint8_t i = 0; i < len; i++){
        uart_putc(buffer[i]);
    }
    UCSRB |= (1 << TXCIE);

}
ISR(USART_TXC_vect){
    UCSRB &= ~(1 << TXCIE);
    PORTB &= ~(1 << RS485_RXTX_EN);
}

int main (void){

    //Status Led set and turn off
    DDRB |= (1 << DDB2);
    PORTB &= ~(1 << STATUS_LED);

    //RS485 Driver Tx/Rx control pin to output and enable RX
    DDRB |= (1 << DDB1);
    PORTB &= ~(1 << RS485_RXTX_EN);

    //enable internal UART RX pull-up
    PORTD |= (1 << PD0);

    //initialize uart
    uart_init();

    //enable global interrupts
    sei();

    //ony byte from RX buffer
    unsigned int cmd; //was unsigned int
    uint8_t response[16];
    uint8_t dataAddress;
    uint8_t dataLen;
    uint8_t data[16];

    FRAME_STATE state = STATE_IDLE;

    while(1){
        switch(state){
            //Start of frame, wait for start byte
            case STATE_IDLE:
                /*do{cmd = uart_getc();}while(cmd != START_BYTE);*/
                /*state = STATE_FETCH_ADDRESS;*/
                cmd = uart_getc();
                if(cmd == START_BYTE)
                    state = STATE_FETCH_ADDRESS;

                break;
                //Get slave address
            case STATE_FETCH_ADDRESS:
                do{cmd = uart_getc();}while(cmd & UART_NO_DATA);
                if(cmd == SLAVE_ADDRESS){
                    dataAddress = cmd;
                    state = STATE_FETCH_DATA;
                }else{
                    //Message not for this slave
                    state = STATE_IDLE;
                }
                break;
                //Get rest of frame
            case STATE_FETCH_DATA:
                //Led on mark start of message
                PORTB |= (1 << STATUS_LED);

                //Get bytes from UART until STOP_BYTE
                //get datalen byte
                do{cmd = uart_getc();}while(cmd & UART_NO_DATA);
                dataLen = cmd;

                uint8_t i = 0;
                do{
                    //Get each data byte
                    cmd = uart_getc();
                    if((!(cmd & UART_NO_DATA)) && i != dataLen){
                        data[i] = cmd;
                        i++;
                    }
                }while(i < dataLen);

                //get stop byte
                if(cmd != STOP_BYTE){
                    do{cmd = uart_getc();}while(cmd != STOP_BYTE);
                }
                //TODO: later add XOR checksum message validation

                state = STATE_SEND_RESPONSE;

                break;
            case STATE_SEND_RESPONSE:

                ds18b20_gettemp(&lastWholeTemp, &lastDecimalTemp);
                //Send Response
                response[0] = START_BYTE;
                response[1] = SLAVE_ADDRESS;
                response[2] = 0x05;
                /*response[2] = 0x08;*/
                response[3] = (lastWholeTemp/100+'0');
                response[4] = (lastWholeTemp/10+'0');
                response[5] = (lastWholeTemp%10+'0');
                response[6] = ('.');
                response[7] = (lastDecimalTemp/1000 + '0');
                /*response[8] = ((lastDecimalTemp/100)%10 + '0');
                response[9] = ((lastDecimalTemp/10)%10+ '0');
                response[10] = (lastDecimalTemp%10 + '0');*/
                response[8] = (STOP_BYTE);

                /*sendResponse(response, 12);*/
                sendResponse(response, 9);

                PORTB &= ~(1 << STATUS_LED);
                state = STATE_IDLE;
                break;
            default:
                break;
        }
    }
}
