#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "uart.h"
#include "defines.h"

volatile unsigned char adcRes[4];

void initADC(){
    ADMUX |= (1 < REFS0);//AVCC acts as ARef
    /*ADMUX |= (1 << ADLAR);//8bit mode*/
    ADCSRA = (1<<ADPS1) | (1<<ADPS2) | (1<<ADPS0);//prescale 128
    ADCSRA |= (1 << ADIE);//enable interrupt
    /*ADCSRA |= (1 << ADFR);  // Set ADC to Free-Running Mode*/
    /*ADCSRA |= (1 << ADEN);  // Enable ADC*/
    /*ADCSRA |= (1 << ADSC);  // Start A2D Conversions*/
    ADCSRA |=  (1 << ADEN); //enable adc
}
ISR(ADC_vect){
    itoa(ADC,adcRes,10);
    //uint8_t low = ADCL;
    //or (ADCH <<8) | low

    ADCSRA |= (1 << ADSC);//start next conversion

}
uint16_t readADC(uint8_t ch){
    DDRC = 0x00;
    PORTC &= ~(1 << ch);//disable internal pullup
    ADMUX = (0xf0 & ADMUX) | 0;//good?
    /*ch=ch&0b00000111;*/
    /*ADMUX|=ch;*/
    ADCSRA |= (1 << ADSC);//start adc conversion
    while(ADCSRA & (1 << ADSC));
    
    uint16_t tmpAdc = ADC;

    return tmpAdc;
}

void sendResponse(char * buffer, int len){
    RS485_PORT |= (1 << RS485_CTRL);

    for(uint8_t i = 0; i < len; i++){
        uart_putc(buffer[i]);
    }
    UCSRB |= (1 << TXCIE);

}
ISR(USART_TXC_vect){
    UCSRB &= ~(1 << TXCIE);
    RS485_PORT &= ~(1 << RS485_CTRL);
}

int main (void){
    initADC();

    /*FOR DEBUG, set pin to high*/
    LED_DDR |= (1 << LED0);

    //initialize uart
    uart_init();

    //enable global interrupts
    sei();

    uint16_t adcValue;
    /*uint8_t adcValue;*/

    //ony byte from RX buffer
    uint16_t cmd; //was unsigned int
    uint8_t response[16];
    uint8_t dataAddress;
    uint8_t dataLen;
    uint8_t data[16];

    state = STATE_IDLE;


    while(1){
        switch(state){
            //Start of frame, wait for start byte
            case STATE_IDLE:
                cmd = uart_getc();
                if(cmd == START_BYTE){
                    state = STATE_FETCH_ADDRESS;
                }
                ADCSRA |= (1 << ADSC);  // Start A2D Conversions
                break;
                //Get slave address
            case STATE_FETCH_ADDRESS:
                do{cmd = uart_getc();}while(cmd & UART_NO_DATA);
                if(cmd == SLAVE_ADDRESS){
                    /*LED_PORT |= (1 << LED0);*/
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
                LED_PORT |= (1 << LED0);

                //Get bytes from UART until STOP_BYTE
                //get datalen byte
                do{cmd = uart_getc();}while(cmd & UART_NO_DATA);
                dataLen = cmd;

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

                state = STATE_PROCESSING;
                break;
            case STATE_PROCESSING:
                adcValue = readADC(LDR_PIN);

                /*while(ADCSRA & (1 << ADSC));*/
                i = adcValue / 204,8;
                adcVoltage = (i*10/(5-i));
                dtostrf(LDR, 4, 1, LDRSHOW);
                state = STATE_SEND_RESPONSE;

                break;
            case STATE_SEND_RESPONSE:

                //Send Response
                response[0] = START_BYTE;
                response[1] = SLAVE_ADDRESS;
                response[2] = 0x02;
                response[3] = adcRes[0];
                response[4] = adcRes[1];
                response[5] = adcRes[2];
                response[6] = adcRes[3];
                /*response[3] = (uint8_t)(adcValue >> 8);*/
                /*response[4] = (uint8_t)(adcValue);*/
                response[7] = STOP_BYTE;

                sendResponse(response, 8);

                LED_PORT &= ~(1 << LED0);
                state = STATE_IDLE;
                break;
            default:
                break;
        }
    }
}
