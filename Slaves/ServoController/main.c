 /*------- Preamble -------- //*/
 #include <avr/io.h>
 #include <util/delay.h>
 #include <avr/interrupt.h>
 /*#include "pinDefines.h"*/
 /*#include "uart.h"*/

 /* experiment with these values */
 /* to match your own servo */
 #define PULSE_MIN 1000
 /*#define PULSE_MIN 97*/
 #define PULSE_MAX 2000
 /*#define PULSE_MAX 535*/
 #define PULSE_MID 1500
 /*#define PULSE_MID 315*/


 /*static inline uint16_t getNumber16(void);*/
 static inline void initTimer1Servo(void) {
	// Set up Timer1 (16bit) to give a pulse every 20ms
	// Use Fast PWM mode, counter max in ICR1
	TCCR1A |= (1 << WGM11) | (1<<COM1B1); //Fast PWM modea all WGm bits set
	TCCR1B |= (1 << WGM12) | (1 << WGM13);
	TCCR1A |= (1 << CS10) |  (1<<CS11); //1 prescaling -- counting in microseconds
	/*ICR1 = 4607;*/
	ICR1 = 20000;
	// TOP value = 20ms, 50Hz
	TCCR1A |= (1 << COM1A1);
	// Direct output on PB1 / OC1A
	DDRB |= (1 << PB3);
	// set pin for output
 }
 static inline void showOff(void) {
	/*printString("Center\r\n");*/
	OCR1A = PULSE_MIN;
	_delay_ms(1500);

	OCR1A = PULSE_MIN;
	_delay_ms(1500);

	OCR1A = PULSE_MAX;
	_delay_ms(1500);

	OCR1A = PULSE_MID;
	_delay_ms(1500);
 }
 int main(void) {
	// -------- Inits --------- //
	uint16_t servoPulseLength;
	DDRD |= (1 << PD6);
	OCR1A = PULSE_MID;
	/* set it to middle position initially */
	initTimer1Servo();
	/*initUSART();*/
	/*printString("\r\nWelcome to the Servo Demo\r\n");*/
	/*220*/
	/*Make: AVR ProgrammingServos*/
	showOff();
	PORTD |= (1 << PD6);
	// ------ Event loop ------ //
	servoPulseLength = 1000;
	while (1) {
		/*printString("\r\nEnter a four-digit pulse length:\r\n");*/
		/*servoPulseLength = getNumber16();*/
		/*printString("On my way....\r\n");*/
		OCR1A = servoPulseLength;
		/* re-enable output pin */
		_delay_ms(1000);
		/*printString("Releasing...\r\n");*/
		while (TCNT1 < 3000) {;
		}
		/* delay until pulse part of cycle done */
		DDRB &= ~(1 << PB3);
		/* disable output pin */
		}
	return (0);
	/* End event loop */
	/* This line is never reached */
 }
 /*static inline uint16_t getNumber16(void) {
 // Gets a PWM value from the serial port.
 // Reads in characters, turns them into a number
 char thousands = '0';
 char hundreds = '0';
 char tens = '0';
 char ones = '0';
 char thisChar = '0';
 do {
 thousands = hundreds;
 hundreds = tens;
 tens = ones;
 ones = thisChar;
 thisChar = receiveByte();
 transmitByte(thisChar);
 } while (thisChar != '\r');
 /* shift numbers over */
 /* get a new character */
 /* echo /
 transmitByte('\n');
 /* newline /
 return (1000 * (thousands - '0') + 100 * (hundreds - '0') +
 10 * (tens - '0') + ones - '0');
 }*/
