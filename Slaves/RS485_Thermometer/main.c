#define F_CPU 14745600UL

#define RS485_RXTX_EN PB1
#define STATUS_LED PB2

#define SLAVE_ADDRESS 0x14

#define START_BYTE 0x96
#define STOP_BYTE 0xA9

typedef enum {
    STATE_START,
    STATE_FRAME_START
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
    unsigned char *response;
    FRAME_STATE state = STATE_START;

    while(1){
        ds18b20_gettemp(&lastWholeTemp, &lastDecimalTemp);

        //get byte from receive buffer
        cmd = uart_getc();

        if (!( cmd & UART_NO_DATA )){
            //toggle led on data receive
            /*PORTB ^= (1 << STATUS_LED);
            //enable TX, disable RX
            PORTB |= (1 << RS485_RXTX_EN);
            //echo back test
            uart_putc((unsigned char)cmd);
            _delay_ms(1);//Neveikia be sito delay

            //Reenable RX
            PORTB &= ~(1 << RS485_RXTX_EN);*/
            //Start of frame
            switch(state){
                case STATE_START:
                    if(cmd == START_BYTE){
                        PORTB |= (1 << STATUS_LED);
                        state = STATE_FRAME_START;
                    }
                    break;
                case STATE_FRAME_START:
                    if(cmd == SLAVE_ADDRESS){

                        //Send Response
                        PORTB |= (1 << STATUS_LED);
                        PORTB |= (1 << RS485_RXTX_EN);
                        uart_putc(START_BYTE);
                        uart_putc(SLAVE_ADDRESS);
                        //Send temp in 8 bytes
                        //TODO: add data len byte
                        uart_putc(lastWholeTemp/100+'0');
                        uart_putc(lastWholeTemp/10+'0');
                        uart_putc(lastWholeTemp%10+'0');
                        uart_putc('.');
                        uart_putc(lastDecimalTemp/1000 + '0');
                        uart_putc((lastDecimalTemp/100)%10 + '0');
                        uart_putc((lastDecimalTemp/10)%10+ '0');
                        uart_putc(lastDecimalTemp%10 + '0');
                        _delay_ms(10);//This is shit
                        //Reenable RX
                        PORTB &= ~(1 << RS485_RXTX_EN);

                    }
                    state = STATE_START;
                    break;

            }

        }
    }
}
