 #include <avr/io.h>
 #include <util/delay.h>
 #include <avr/interrupt.h>
 #include "defines.h"
 #include "uart.h"

typedef enum {
    STATE_IDLE,
    STATE_FETCH_ADDRESS,
    STATE_FETCH_DATA,
    STATE_SEND_RESPONSE
}FRAME_STATE;


//For Servo SRM-102, 0.2sec/60deg
static inline void initTimer1Servo(void) {
    // Direct output on PB1 / OC1A
    DDRB |= (1 << PB3);

    // Set up Timer1 (16bit) to give a pulse every 20ms
    // Use Fast PWM mode, counter max in ICR1
    TCCR1A |= (1 << WGM11); //Fast PWM mode, page 113
    TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS10);//No prescaler

    //Non inverted mode
    TCCR1A |= (1 << COM1A1);// | (1 << COM1A0); //set COM1A0 for inverted mode

    // TOP value = 20ms, 50Hz
    ICR1 = 294911; // (14745600 / 50) - 1
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

static inline void showOff(void) {

    OCR1A = SERVO_0DEG;
    _delay_ms(1000);
    OCR1A = SERVO_30DEG;
    _delay_ms(1000);
    OCR1A = SERVO_45DEG;
    _delay_ms(1000);
    OCR1A = SERVO_60DEG;
    _delay_ms(1000);
    OCR1A = SERVO_90DEG;
    _delay_ms(1000);
    OCR1A = SERVO_120DEG;
    _delay_ms(1000);
    OCR1A = SERVO_90DEG;
    _delay_ms(1000);
    OCR1A = SERVO_60DEG;
    _delay_ms(1000);
    OCR1A = SERVO_45DEG;
    _delay_ms(1000);
    OCR1A = SERVO_30DEG;
    _delay_ms(1000);
    OCR1A = SERVO_0DEG;
    _delay_ms(1000);

 }

 int main(void) {
    // -------- Inits --------- //
    LED_DDR |= (1 << LED0);

    initTimer1Servo();

    //RS485 Driver Tx/Rx control pin to output and enable RX
    RS485_DDR |= (1 << RS485_CTRL);
    RS485_PORT &= ~(1 << RS485_CTRL);

    //enable internal UART RX pull-up
    PORTD |= (1 << PD0);
    uart_init();

    /*showOff();*/

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
                LED_PORT |= (1 << LED0);

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

                //Parse message and turn servo accordingly
                if(data[0] == 0x01){
                    switch(data[1]){
                        case 0:
                            OCR1A = SERVO_0DEG;
                            break;
                        case 30:
                            OCR1A = SERVO_30DEG;
                            break;
                        case 45:
                            OCR1A = SERVO_45DEG;
                            break;
                        case 60:
                            OCR1A = SERVO_60DEG;
                            break;
                        case 90:
                            OCR1A = SERVO_90DEG;
                            break;
                        case 120:
                            OCR1A = SERVO_120DEG;
                            break;
                    }
                }

                state = STATE_SEND_RESPONSE;

                break;
            case STATE_SEND_RESPONSE:

                //Send Response ACK
                response[0] = START_BYTE;
                response[1] = SLAVE_ADDRESS;
                response[2] = 0x01;
                response[3] = 0x01;
                response[4] = (STOP_BYTE);

                sendResponse(response, 5);

                LED_PORT &= ~(1 << LED0);
                state = STATE_IDLE;
                break;
            default:
                break;
        }
    }
 }
