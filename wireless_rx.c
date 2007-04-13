/*
 * By Lars Englund 2007
 * lars . englund at gmail . com
 *
 * Cypress Wireless USB module (CYWM6934 and CYWM6935 (CYWUSB6934, CYWUSB6935)) demo, see project page for more information:
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
#include "CYWUSB693x.h"


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

#define LED_PIN			PC5 // Output

#define low(port, pin) (port &= ~_BV(pin))
#define high(port, pin) (port |= _BV(pin))


/*
ISR(INT0_vect) {
	// Handle CYWUSB6934x interrupts
}
*/


void SPI_Write(uint8_t byte)
{
	SPDR = byte;				// Send SPI byte
	while(!(SPSR & (1<<SPIF)));	// Wait for SPI transmission complete
}

void CYWM_WriteReg(uint8_t which, uint8_t data)
{
	low(PORTB, CYWM_nSS);
	SPI_Write(REG_WRITE | which);
	SPI_Write(data);
	high(PORTB, CYWM_nSS);
}

uint8_t CYWM_ReadReg(uint8_t which) 
{
	low(PORTB, CYWM_nSS);
	SPI_Write(which);
	SPI_Write(which);
	high(PORTB, CYWM_nSS);
	return SPDR;
}


// Delay hekto-seconds :P
void delay_hs(uint16_t hs) 
{
	uint16_t n;
	for (n=0; n<hs; n++) {
		_delay_ms(10);
	}
}


int main() {
	uint8_t data, n;

	cli();	// Disable interrupts

	// Set port I/O directions
	DDRB = _BV(CYWM_SCK) | _BV(CYWM_MOSI) | _BV(CYWM_nSS);
	DDRC = _BV(CYWM_nPD) | _BV(CYWM_nRESET);
	DDRD = 0;
	
	// Setup SPI
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0); // _BV(SPIE)
	//SPSR = _BV(SPI2X);	// Fast SPI

	// Setup interrupts
	MCUCR = _BV(ISC01) | _BV(ISC00); // Trigger INT0 on rising edge
	//GICR = _BV(INT0); // Enable INT0

	// Set initial pin states
	high(PORTB, CYWM_nSS);

	for (n=0; n<2; n++) {
		high(PORTC, LED_PIN);
		delay_hs(50);
		low(PORTC, LED_PIN);
	}

	//sei(); // Enable interrupts

	// Initialise USART
	usart_init(USART_BAUDRATE(9600,8));
	usart_puts("\n\rUSART enabled\n\r");

	// Enable radio
	high(PORTC, CYWM_nPD);
	high(PORTC, CYWM_nRESET);
	usart_puts("Radio enabled\n\r");

	usart_puts("RX: This will be a RX radio\n\r");

	// Setup radio
	// Test
	data = CYWM_ReadReg( REG_ID );
	if (data == 0x07) {
		usart_puts("REG_ID == 0x07: OK!\n\r");
	}
	else {
		usart_puts("REG_ID == 0x07: Failed!\n\r");
	}

	CYWM_WriteReg( REG_CLOCK_MANUAL, 0x41 );		// Must be written with 0x41 after reset for correct operation
	CYWM_WriteReg( REG_CLOCK_ENABLE, 0x41 );		// Must be written with 0x41 after reset for correct operation
	CYWM_WriteReg( REG_SERDES_CTL, 0x03 | 0x08 );	// Enable SERDES
	CYWM_WriteReg( REG_TX_VALID, 0xFF );			// Set all SERDES bits valid for TX
	CYWM_WriteReg( REG_VCO_CAL, 0xC0 );				// Set VCO adjust to -5/+5
	CYWM_WriteReg( REG_ANALOG_CTL, 0x04 );			// Enable PA Control Output
	//CYWM_WriteReg( REG_PWR_CTL, 0x80 );				// Conserve power (must set REG_ANALOG_CTL, bit 6=1 to enable writes)
	CYWM_WriteReg( REG_CHANNEL, 42 );				// Use channel 42
	CYWM_WriteReg( REG_PA, 0x07 );					// Set maximum transmit power
	CYWM_WriteReg( REG_RX_INT_EN, 0x03 );			// Enable EOF and FULL interrupts for channel A
	CYWM_WriteReg( REG_CONTROL, 0x80 );				// Enable RX
	//CYWM_WriteReg( REG_CONTROL, 0x40 );				// Enable TX

	usart_puts("Receiving data:\n\r");

	// Repeat indefinitely
	for (;;)
	{
		data = CYWM_ReadReg( REG_RX_INT_STAT );
		if (data & 0x01 || data & 0x02) { // FULL A || EOF A
			high(PORTC, LED_PIN);
			CYWM_WriteReg( REG_RX_INT_EN, 0x03 );			// Enable EOF and FULL interrupts for channel A
			if (data & 0x08) { // Valid A
				data = CYWM_ReadReg( REG_RX_DATA_A );
				usart_putc(data);
			}
			//data = CYWM_ReadReg( REG_RX_VALID_A );
			low(PORTC, LED_PIN);
		}

		// Echo received characters
		//data = usart_getc();
		//usart_putc(data);
	}

	return 0;
}
