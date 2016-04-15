#include "defines.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "avr-pcf8563/PCF8563.h"    
#include "avr-pcf8563/PCF8563.h"    
#include "uart.h"

//TODO:
//Both uarts
//i2c ds1307


void sendResponse(char * buffer, int len){
    RS485_PORT |= (1 << RS485_CTRL1);
    /*_delay_ms(1);*/

    for(uint8_t i = 0; i < len; i++){
        uart0_putc(buffer[i]);
        /*_delay_ms(1);*/
    }
    /*while(!(UCSR0A & (1 << TXC0)));*/
    UCSR0B |= (1 << TXCIE0);
    /*RS485_PORT &= ~(1 << RS485_CTRL1);*/

}
void sendRequest(char * buffer, int len){
    RS485_PORT |= (1 << RS485_CTRL2);
    /*_delay_ms(1);*/

    for(uint8_t i = 0; i < len; i++){
        uart1_putc(buffer[i]);
        _delay_ms(1);
    }
    UCSR1B |= (1 << TXCIE1);
    /*while(!(UCSR1A & (1 << TXC1)));*/
    /*RS485_PORT &= ~(1 << RS485_CTRL2);*/

}

ISR(USART0_TX_vect){
    UCSR0B &= ~(1 << TXCIE0);
    RS485_PORT &= ~(1 << RS485_CTRL1);
}
ISR(USART1_TX_vect){
    UCSR1B &= ~(1 << TXCIE1);
    RS485_PORT &= ~(1 << RS485_CTRL2);
}
//PCD8563 alarm
////TODO: implement queue and sorting by remaining time
ISR(INT2_vect){
}
//Use 1HZ output from rtc chip? 
ISR(TIMER1_COMPA_vect){
}
//Eeprom ready
//TODO: write and read configs
ISR(EE_READY_vect){
}
//is needed I2C int??
/*ISR(TWI_vect){
}*/


int main (void){

    //Set LED pins to Output
    LED_DDR |= (1 << LED1) | (1 << LED2);
    LED_PORT &= ~(1 << LED1) | (1 << LED2);

    PCF_Init(PCF_ALARM_INTERRUPT_ENABLE);// | PCF_TIMER_INTERRUPT_ENABLE);
    
    //Wont yet work, not soldered
    PCF_SetClockOut(PCF_CLKOUT_1HZ);//RTC clkout to avr one second clock

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
    uint16_t cmd; //was unsigned int
    //Frame variables
    uint8_t response[32];
    uint8_t dataAddress;
    uint8_t dataLen;
    uint8_t data[32];//Maximum defined data lenght
    uint8_t i;
    unsigned int tempDataByte;

    //TEMP for testing auto lights
    /*state = STATE_SLAVE_SEND;*/

    //Main Loop with Finite State Machine for handling protocol frames
    while(1){
        switch(state){
            case STATE_IDLE:
                //Wait for start byte of protocol frame
                cmd = uart0_getc();
                if(cmd == START_BYTE){
                    state = STATE_FETCH_ADDRESS;
                }
                break;
            case STATE_FETCH_ADDRESS:
                //Get adress byte
                do{cmd = uart0_getc();}while(cmd & UART_NO_DATA);
                //Message for master from PC
                if(cmd == SLAVE_ADDRESS){
                    state = STATE_MASTER_FRAME;
                    LED_PORT |= (1 << LED1);
                //Or message for slaves
                }else{
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
                /*do{cmd = uart0_getc();}while(cmd != STOP_BYTE);*/
                if(cmd != STOP_BYTE){
                    do{cmd = uart0_getc();}while(cmd != STOP_BYTE);
                }


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
            //Send Request to slave
            case STATE_SLAVE_SEND:

                //TEST: send message to LDR;
                /*response[0] = START_BYTE;
                response[1] = 0x15;
                response[2] = 0x01;
                response[3] = 0x00;
                response[4] = STOP_BYTE;

                sendRequest(response, 5);*/
                response[0] = START_BYTE;
                response[1] = dataAddress;
                response[2] = dataLen;
                for(i = 0;i < dataLen; i++)
                    response[3 + i] = data[i];
                response[3+i] = STOP_BYTE;

                sendRequest(response, (4+dataLen));

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

                state = STATE_SEND_RESPONSE;
                break;
            //Send Response back to PC
            case STATE_SEND_RESPONSE:

                response[0] = START_BYTE;
                response[1] = dataAddress;
                response[2] = dataLen;
                for(i = 0;i < dataLen; i++)
                    response[3 + i] = data[i];
                response[3+i] = STOP_BYTE;

                sendResponse(response, (4+dataLen));

                state = STATE_IDLE;

                //TODO: get LDR value, compare switch lights relay
                //TEMP
                /*state = STATE_SLAVE_SEND;*/

                break;
            case STATE_MASTER_COMMAND:

                /*
                 *PCF_DateTime dateTime;
                 dateTime.second = 43;
                 dateTime.minute = 59;
                 dateTime.day = 15;
                 dateTime.weekday = 6;
                 dateTime.month = 8;
                 dateTime.year = 2015;

                 PCF_SetDateTime(&dateTime);
                 * */


                //TODO: implement master protocol logic
                //TEST: switch relay on
                RELAY_PORT ^= (1 << RELAY1) | (1 << RELAY2);
                LED_PORT &= ~(1 << LED1);

                state = STATE_IDLE;
                break;
            default:
                /**/
                break;
        }

    }
}
