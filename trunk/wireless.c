/*
 * By Lars Englund 2007
 * lars . englund at gmail . com
 *
 * Cypress Wireless USB module (CYWM6934 and CYWM6935 (CYWUSB6934, CYWUSB6935)) demo
 * This is only a simple (and incomplete) test for reading and writing registers in the CYWUSB693x, see wireless_tx.c & wireless_rx.c for a complete TX/RX demonstration.
 * Also see the project page for more information:
 * http://code.google.com/p/cywusb
 *
 * $Id$
 */

#define F_CPU 8000000UL

#include <ctype.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "usart.h"


//#define CYWM_CTRL_DIR_PORTB	DDRB // Set bits here to 1 to make them outputs, 0 for inputs
//#define CYWM_CTRL_IN_PORTC	PINC
//#define CYWM_CTRL_OUT_PORT	PORTC

#define CYWM_SCK		PB5 // Output
#define CYWM_MISO		PB4	// Input
#define CYWM_MOSI		PB3	// Output
#define CYWM_nSS		PB2	// Output	

#define CYWM_nPD		PC1	// Output	
#define CYWM_nRESET		PC0	// Output
#define CYWM_IRQ		PD2 // Input


#define low(port, pin) (port &= ~_BV(pin))
#define high(port, pin) (port |= _BV(pin))


void SPI_Write(uint8_t byte)
{
	SPDR = byte;	// Send SPI byte
	while(!(SPSR & (1<<SPIF)));	// Wait for SPI transmission complete
}


int main() {
	uint8_t data;
	//uint16_t i;

	cli();	// Disable interrupts

	// Set port I/O directions
	DDRB = _BV(CYWM_SCK) | _BV(CYWM_MOSI) | _BV(CYWM_nSS);
	DDRC = _BV(CYWM_nPD) | _BV(CYWM_nRESET);
	DDRD = 0;
	
	// Setup SPI
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0); // _BV(SPIE)
	//SPSR = _BV(SPI2X);	// Fast SPI

	// Set initial pin states
	high(PORTB, CYWM_nSS);

	// Initialise USART
	usart_init(USART_BAUDRATE(9600,8));
	usart_puts("\n\rUSART enabled\n\r");

	// Enable radio
	high(PORTC, CYWM_nPD);
	high(PORTC, CYWM_nRESET);
	usart_puts("Radio enabled\n\r");

	// Setup radio
	low(PORTB, CYWM_nSS);
	SPI_Write(0x03);
	SPI_Write(0x03);
	high(PORTB, CYWM_nSS);
	data = SPDR;
	usart_putc(data);

	low(PORTB, CYWM_nSS);
	SPI_Write(0x83);
	SPI_Write(0x80);
	high(PORTB, CYWM_nSS);

	low(PORTB, CYWM_nSS);
	SPI_Write(0x03);
	SPI_Write(0x03);
	high(PORTB, CYWM_nSS);
	data = SPDR;
	usart_putc(data);

	// Repeat indefinitely
	for (;;)
	{
		// Echo received characters
		data = usart_getc();
		usart_putc(data);
	}

	return 0;
}
