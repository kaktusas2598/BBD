//TODO: bunch of defines
#define F_CPU 14745600UL

#define LED_DDR DDRD
#define LED_PORT PORTD
#define LED1 PD7
#define LED2 PD6

#define RELAY_DDR DDRC
#define RELAY_PORT PORTC
#define RELAY1 PC7
#define RELAY2 PC6

//RS485 drivers control
#define RS485_DDR DDRB
#define RS485_PORT PORTB
#define RS485_CTRL1 PB0 //uart0(PC)
#define RS485_CTRL2 PB1 //uart1(slaves)

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
    STATE_FRAME_STOP,
    STATE_MASTER_FRAME,
    STATE_SLAVE_FRAME
}FRAME_STATE;

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
    LED_DDR = (1 << LED1) | (1 << LED2);
    LED_PORT &= ~(1 << LED1) | (1 << LED2);

    //Set RS485 drivers Mode
    //RS485 Drivers Tx/Rx control piasn to output
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
    uint8_t * data;
    unsigned int tempDataByte;
    FRAME_STATE state = STATE_START;

    while(1){
        //Main Loop:
        //Get data from USART0 (PC)
        //Make Finite State Machine(FSM)
        //Parse messages
        //Do actions:
        //control power
        //control slaves
        //
        //get byte from receive buffer
        cmd = uart0_getc();

        /*//Echo back test
        RS485_PORT |= (1 << RS485_CTRL1);
        uart0_putc((unsigned char)cmd);
        _delay_ms(1);//Neveikia be sito delay
        RS485_PORT &= ~(1 << RS485_CTRL1);*/

        
        //Do actions if there is data in uart0 rx buffer
        if (!( cmd & UART_NO_DATA )){
            switch(state){
                case STATE_START:
                    //Start of protocol frame
                    if(cmd == START_BYTE){
                        state = STATE_FRAME_START;
                    }
                    break;
                case STATE_FRAME_START:
                    //Message for master from PC
                    if(cmd == SLAVE_ADDRESS){
                        state = STATE_MASTER_FRAME;
                    }else{
                        state = STATE_SLAVE_FRAME;
                    }
                    dataAddress = cmd;
                    break;
                case STATE_MASTER_FRAME:
                    //Get bytes from UART0 until STOP_BYTE
                    while(cmd != STOP_BYTE){
                    }
                    break;
                case STATE_SLAVE_FRAME:
                    //Get bytes from UART0 until STOP_BYTE
                    /*while(cmd != STOP_BYTE){*/
                        //get datalen byte
                        dataLen = cmd;

                        //allocate data buffer
                        //malloc over 500bytes...
                        data = (uint8_t *)malloc(dataLen*sizeof(uint8_t));//Works without this..
                        //get data bytes
                        uint8_t i = 0;
                        do{
                            //Get each data byte
                            tempDataByte = uart0_getc();
                            if(!(tempDataByte & UART_NO_DATA) && i != dataLen){
                                data[i] = tempDataByte;
                                i++;
                            }

                        }while(i < dataLen);

                        //Enable uart0 tx
                        /*RS485_PORT |= (1 << RS485_CTRL1);
                        //Send Response
                        uart0_putc(START_BYTE);
                        uart0_putc(dataAddress);
                        uart0_putc(dataLen);//temp
                        for(i = 0; i < dataLen; i++){
                            uart0_putc(data[i]);
                        }
                        uart0_putc(STOP_BYTE);*/
                        /*UCSR0B |= (1 << TXCIE0);*/


                        //We have all data, now send this message to slaves
                        //Send Response
						/*UCSR1B |= (1 << TXCIE1);*/
                        RS485_PORT |= (1 << RS485_CTRL2);
                        uart1_putc(START_BYTE);
                        uart1_putc(0x14);
						while(!(UCSR1A & (1 << TXC1)));
                        /*uart1_putc(dataLen);//temp
                        for(i = 0; i < dataLen; i++){
                            uart1_putc(data[i]);
                        }
                        uart1_putc(STOP_BYTE);*/
                        /*_delay_ms(10);//This is shit*/
                        //Enable USART1 RX
                        RS485_PORT &= ~(1 << RS485_CTRL2);

                        RS485_PORT |= (1 << RS485_CTRL1);
                        //Get slave message
                        //BUG: stuck on this loop
                        i = 0;
                        do{
                            //Get each data byte
                            tempDataByte = uart1_getc();
                            if(!(tempDataByte & UART_NO_DATA) && i != 10){
                                /*data[i] = tempDataByte;*/
                                uart0_putc(tempDataByte);
                                /*LED_PORT |= (1 << LED1);*/
                                i++;
                            }

                        }while(i < 10);

                        LED_PORT |= (1 << LED2);
                        RS485_PORT |= (1 << RS485_CTRL1);
                        for(i = 0; i < 8; i++){
                            uart0_putc(data[i]);
                        }


                        /*//Enable uart0 tx
                        RS485_PORT |= (1 << RS485_CTRL1);
                        //Send Response
                        uart0_putc(START_BYTE);
                        uart0_putc(dataAddress);
                        uart0_putc(dataLen);//temp
                        for(i = 0; i < dataLen; i++){
                            uart0_putc(data[i]);
                        }
                        uart0_putc(STOP_BYTE);*/
                        _delay_ms(10);//This is shit
                        //Reenable RX
                        RS485_PORT &= ~(1 << RS485_CTRL1);
                        state = STATE_START;

                    /*}*/
                    break;
                //Unneccessary???
                /*case STATE_FRAME_STOP:
                    state = STATE_START;
                    break;*/
                default:
                    /**/
                    break;
            }

        }
    }
}

//Slave transmit complete
ISR(USART1_TX_vect){
    LED_PORT ^= (1 << LED1);
}
//Master transmit complete
ISR(USART0_TX_vect){
    LED_PORT |= (1 << LED2);
}
