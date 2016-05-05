!#/bin/bash

#8.0- MGHz ext crystal fuses settings for ATMega324A
avrdude -c usbasp -p atmega324a -U lfuse:w:0b1111111:m -U hfuse:w:0b10011001:m
