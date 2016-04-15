!#/bin/bash

#PROBLEM IS WITH USBASP PROGRAMMER
#8.0- MGHZ EXT CRYSTAL FUSES SETTINGS FOR ATMEGA8
#lfuse was FF, higher startup time..
sudo avrdude -p atmega8 -c usbasp -b 115200 -U lfuse:w:0xFF:m -U hfuse:w:0xC9:m
#Default
#sudo avrdude -p atmega8 -c usbasp -b 115200 -U lfuse:w:0xe1:m -U hfuse:w:0xD9:m

