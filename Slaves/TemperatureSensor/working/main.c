#define F_CPU 14745600UL

#define RS485_RXTX_EN PB1
#define STATUS_LED PB2

#define SLAVE_ADDRESS 0x14

#define START_BYTE 0x96
#define STOP_BYTE 0xA9

typedef enum {
    STATE_START,
    STATE_FRAME_START,
    STATE_FRAME_DATA,
    STATE_FRAME_REPLY
}FRAME_STATE;

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "ds18b20.h"
#include "uart.h"

volatile uint8_t lastWholeTemp;
volatile uint16_t lastDecimalTemp;

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
    /*unsigned char *response;*/
    uint8_t dataAddress;
    uint8_t dataLen;
    uint8_t * data;

    FRAME_STATE state = STATE_START;

    while(1){
        switch(state){
            //Start of frame, wait for start byte
            case STATE_START:
                /*do{cmd = uart_getc();}while(cmd != START_BYTE);*/
                /*state = STATE_FRAME_START;*/
				cmd = uart_getc();
				if(cmd == START_BYTE)
					state = STATE_FRAME_START;

                break;
            //Get slave address
            case STATE_FRAME_START:
                do{cmd = uart_getc();}while(cmd & UART_NO_DATA);
                if(cmd == SLAVE_ADDRESS){
					//Led on mark start of message
					PORTB |= (1 << STATUS_LED);
                    dataAddress = cmd;
                    state = STATE_FRAME_DATA;
                }else{
                    //Message not for this slave
                    state = STATE_START;
                }
                break;
            //Get rest of frame
            case STATE_FRAME_DATA:
                //Get bytes from UART until STOP_BYTE
                //get datalen byte
                do{cmd = uart_getc();}while(cmd & UART_NO_DATA);
                dataLen = cmd;

                //allocate data buffer
                //malloc over 500bytes...
                data = (uint8_t *)malloc(dataLen*sizeof(uint8_t));//Works without this..
                //get data bytes
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

                state = STATE_FRAME_REPLY;

                break;
            case STATE_FRAME_REPLY:

				ds18b20_gettemp(&lastWholeTemp, &lastDecimalTemp);
                //Send Response
                PORTB |= (1 << RS485_RXTX_EN);
                uart_putc(START_BYTE);
                uart_putc(SLAVE_ADDRESS);
                uart_putc(0x08);//Send temp in 8 bytes
                uart_putc(lastWholeTemp/100+'0');
                uart_putc(lastWholeTemp/10+'0');
                uart_putc(lastWholeTemp%10+'0');
                uart_putc('.');
                uart_putc(lastDecimalTemp/1000 + '0');
                uart_putc((lastDecimalTemp/100)%10 + '0');
                uart_putc((lastDecimalTemp/10)%10+ '0');
                uart_putc(lastDecimalTemp%10 + '0');
                uart_putc(STOP_BYTE);

                /*while(!(UCSRA & (1 << TXC))); why u no work????*/
                _delay_ms(20);//This is shit
                //Reenable RX
                PORTB &= ~(1 << RS485_RXTX_EN);

                PORTB &= ~(1 << STATUS_LED);
                state = STATE_START;
                break;
            default:
                break;
        }
    }
}
