/*
 * ATMega8 Light Sensor with RS485 interface
 * Created By: Nerijus Vilčinskas
 */
/*#define F_CPU 14745600UL*/

#include <avr/io.h>
#include <util/delay.h>

int main (void){

    //Status Led set and turn off
    DDRD |= (1 << 7);
    PORTD |= (1 << 7) ;

	while(1){
    }
}
