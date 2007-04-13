#
# By Lars Englund 2007
# lars . englund at gmail . com
#
# Cypress Wireless USB module (CYWM6934 and CYWM6935 (CYWUSB6934, CYWUSB6935)) demo, see project page for more information:
# http://code.google.com/p/cywusb
#
# $Id$
#

all:			clean wireless show_size wireless_hex
rx:				clean wireless_rx show_size_rx wireless_rx_hex
tx:				clean wireless_tx show_size_tx wireless_tx_hex


CC = avr-gcc
CCFLAGS = -Os -mmcu=atmega8 -DF_CPU=8000000UL -DF_OSC=8000000 -Wall
OBJCOPY = avr-objcopy
PROGRAMMER = avrdude
PROGRAMMERFLAGS = -p m8 -P com9 -c avrispv2 -V -U


wireless:		usart.o
				$(CC) $(CCFLAGS) -o wireless.o wireless.c usart.o

wireless_hex:
				$(OBJCOPY) -O ihex -j.text -j.data wireless.o wireless.hex

usart.o:		
				$(CC) $(CCFLAGS) -c usart.c

onewire.o:		
				$(CC) $(CCFLAGS) -c onewire.c
				
ds18x20.o:		
				$(CC) $(CCFLAGS) -c ds18x20.c

crc8.o:		
				$(CC) $(CCFLAGS) -c crc8.c

delay.o:		
				$(CC) $(CCFLAGS) -c delay.c

prog:			
				$(PROGRAMMER) $(PROGRAMMERFLAGS) flash:w:wireless.hex

clean:
				rm -f *.o *.hex

show_size:		
				avr-size wireless.o


# RX
wireless_rx:	usart.o
				$(CC) $(CCFLAGS) -o wireless_rx.o wireless_rx.c usart.o

wireless_rx_hex:
				$(OBJCOPY) -O ihex -j.text -j.data wireless_rx.o wireless_rx.hex

show_size_rx:		
				avr-size wireless_rx.o

prog_rx:			
				$(PROGRAMMER) $(PROGRAMMERFLAGS) flash:w:wireless_rx.hex


#TX
wireless_tx:	usart.o onewire.o ds18x20.o crc8.o delay.o 
				$(CC) $(CCFLAGS) -o wireless_tx.o wireless_tx.c usart.o onewire.o ds18x20.o crc8.o delay.o

wireless_tx_hex:
				$(OBJCOPY) -O ihex -j.text -j.data wireless_tx.o wireless_tx.hex

show_size_tx:		
				avr-size wireless_tx.o

prog_tx:			
				$(PROGRAMMER) $(PROGRAMMERFLAGS) flash:w:wireless_tx.hex
