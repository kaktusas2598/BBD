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

//TODO: simplify for usb later for june
//Send RS485 response to pc station
void masterInit();
void sendResponse(char * buffer, int len);
void sendRequest(char * buffer, int len);

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
    LED_PORT ^= (1 << LED1);
    /*LED_PORT |= (1 << LED1);*/

}
//1 HZ clkout from RTC to AVR 16bit timer
//Compare Metch on Minute on more
//
ISR(TIMER1_COMPA_vect){
    //TODO: Execute repeating actions here
    //Temp disable led
    /*PORTD ^= (1 << PD6);*/
}
//Eeprom ready
//TODO: write and read configs
/*ISR(EE_READY_vect){*/
/*}*/
//is needed I2C int??
/*ISR(TWI_vect){
}*/


int main (void){

    masterInit();

    //External interrupt from RTC (Alarm/Timer)
    //Look on p68 table for ISCn bits
    EICRA = (1 << ISC20) | (1 < ISC21); // configure for rising edge
    EIMSK |= (1<<INT2);

    //TODO: disable later TI_TP flag as it is for timer conttinous interrupt, will fuck up alarm!
    PCF_Init(PCF_ALARM_INTERRUPT_ENABLE | PCF_TIMER_INTERRUPT_ENABLE | PCF8563_TI_TP);

    PCF_DateTime dateTime;
    dateTime.second = 0;
    dateTime.minute = 0;
    dateTime.hour = 7;
    dateTime.day = 5;
    dateTime.weekday = 4;
    dateTime.month = 5;
    dateTime.year = 2016;
    /*PCF_SetDateTime(&dateTime);*/

    PCF_Alarm pcfAlarm;
    pcfAlarm.minute = 0;
    pcfAlarm.hour = 7;
    pcfAlarm.day = 6;
    pcfAlarm.weekday = PCF_DISABLE_ALARM;
    /*PCF_SetAlarm(&pcfAlarm);*/

    //TODO: set alarms based on actions queue
    PCF_SetTimer(PCF_TIMER_1HZ, 1);//10 ticks and fire int

    PCF_SetClockOut(PCF_CLKOUT_1HZ);//RTC clkout to avr one second clock

    //Init timer 1
    //Desired Seconds = OCR1A - 1
    OCR1A = 299;//Interrupt every 5 minutes

    TCCR1B |= (1 << WGM12);//p130. Timer1 mode CTC, TOP on OCR1A
    TIMSK1 |= (1 << OCIE1A); //Timer1 output compare A interrupt enable

    TCCR1B |= (1 << CS12) | (1 << CS11); //Timer source T1, falling edge

    /*rs485Init();*/
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

    //TEMP for testing auto lights
    /*state = STATE_SLAVE_SEND;*/
    state = STATE_IDLE;

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
                    cmd = uart0_getc();
                    if(!(cmd & UART_NO_DATA) && i != dataLen){
                        data[i] = cmd;
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
                    cmd = uart0_getc();
                    if(!(cmd & UART_NO_DATA) && i != dataLen){
                        data[i] = cmd;
                        i++;
                    }

                }while(i < dataLen);

                //get stop byte
                do{cmd = uart0_getc();}while(cmd != STOP_BYTE);
                //TODO: later add XOR checksum message validation

                state = STATE_SLAVE_SEND;
                break;
            //Forward Request to slave
            case STATE_SLAVE_SEND:
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
                //Start byte
                do{cmd = uart1_getc();}while(cmd != START_BYTE);
                //Address
                do{cmd = uart1_getc();}while(cmd & UART_NO_DATA);
                //Datalen
                do{cmd = uart1_getc();}while(cmd & UART_NO_DATA);
                dataLen = cmd;

                i = 0;
                do{
                    //Get each data byte
                    cmd = uart1_getc();
                    if(!(cmd & UART_NO_DATA) && i != dataLen){
                        data[i] = cmd;
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
                 PCF_DateTime dateTime;
                 */
                //switch by command type byte
                /*switch(data[0]){
                    case CMD_RELAY:
                        Switch relay based on second byte
                        break;
                    case CMD_GET_TIME;
                        Get time from PCF8563 
                        PCF_GetDateTime();
                    default:
                        break;
                }*/


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
void masterInit(){
    //Set LED pins to Output
    ////LED1 PB0
    LED_DDR |= (1 << LED1) ;//| (1 << LED2);
    LED_PORT &= ~(1 << LED1);// | (1 << LED2);
    //LED2
    DDRD |= (1 << PD6);

    //Set RELAY pins to output and turn relays off
    RELAY_DDR |= (1 << RELAY1) | (1 << RELAY2);
    RELAY_PORT &= ~(1 << RELAY1) | (1 << RELAY2);

}
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


