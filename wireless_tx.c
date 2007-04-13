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
#define F_OSC 8000000	// for ds18x20 & onewire

#include <ctype.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "usart.h"
#include "CYWUSB693x.h"

#include "onewire.h"
#include "ds18x20.h"
#include "delay.h"


#define MAXSENSORS 5

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


uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];

uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
	usart_puts( "\rScanning Bus for DS18X20\r" );
	
	nSensors = 0;
	
	for( diff = OW_SEARCH_FIRST; diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ; )
	{
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == OW_PRESENCE_ERR ) {
			usart_puts( "No Sensor found\r" );
			break;
		}
		
		if( diff == OW_DATA_ERR ) {
			usart_puts( "Bus Error\r" );
			break;
		}
		
		for (i=0;i<OW_ROMCODE_SIZE;i++) {
			gSensorIDs[nSensors][i]=id[i];
		}
		
		nSensors++;
	}
	
	return nSensors;
}

void usart_put_temp(const uint8_t subzero, uint8_t cel, uint8_t cel_frac_bits)
{
	uint16_t decicelsius;
	
	usart_putc((subzero)?'-':'+');
	decicelsius = DS18X20_temp_to_decicel(subzero, cel, cel_frac_bits);
	usart_puti( (int)(decicelsius/10) );
	usart_puts(".");
	usart_putc( (decicelsius%10) + '0');
}


void wireless_putc(char data)
{
    // Transmit data
	_delay_us(10);
	while (!(CYWM_ReadReg( REG_TX_INT_STAT ) & 0x01));
	CYWM_WriteReg( REG_TX_INT_EN, 0x01 );			// Enable EMPTY interrupt
	CYWM_WriteReg( REG_TX_DATA, data);
}

void wireless_puts(char *data) 
{
	int len, count;

	len = strlen(data);
	for (count = 0; count < len; count++)
		wireless_putc(*(data+count));
}

void wireless_puti( const int val )
{
    char buffer[sizeof(int)*8+1];
    
    wireless_puts( (char*)(itoa(val, buffer, 10)) );
}

void wireless_put_temp(const uint8_t subzero, uint8_t cel, uint8_t cel_frac_bits)
{
	uint16_t decicelsius;
	
	wireless_putc((subzero)?'-':'+');
	decicelsius = DS18X20_temp_to_decicel(subzero, cel, cel_frac_bits);
	wireless_puti( (int)(decicelsius/10) );
	wireless_puts(".");
	wireless_putc( (decicelsius%10) + '0');
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

	uint8_t nSensors, i;
	uint8_t subzero, cel, cel_frac_bits;

	cli();	// Disable interrupts

	// Set port I/O directions
	DDRB = _BV(CYWM_SCK) | _BV(CYWM_MOSI) | _BV(CYWM_nSS);
	DDRC = _BV(CYWM_nPD) | _BV(CYWM_nRESET) | _BV(LED_PIN);
	DDRD = 0;
	
	// Setup SPI
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0); // _BV(SPIE)
	//SPSR = _BV(SPI2X);	// Fast SPI

	// Setup interrupts
	MCUCR = _BV(ISC01) | _BV(ISC00); // Trigger INT0 on rising edge
	//GICR = _BV(INT0); // Enable INT0

	// Set initial pin states
	high(PORTB, CYWM_nSS);

	for (n=0; n<3; n++) {
		high(PORTC, LED_PIN);
		delay_hs(50);
		low(PORTC, LED_PIN);
	}

	//sei(); // Enable interrupts

	// Initialise USART
	usart_init(USART_BAUDRATE(9600,8));
	usart_puts("\n\rUSART enabled\n\r");

	// Init I2C-bus
	#ifndef OW_ONE_BUS
	ow_set_bus(&PIND,&PORTD,&DDRD,PD6);
	#endif

	nSensors = search_sensors();
	usart_puti((int) nSensors);
	usart_puts( " DS18X20 Sensor(s) available:\r" );

	#ifdef DS18X20_VERBOSE
	for (i=0; i<nSensors; i++) {
		usart_puts("# in Bus :");
		usart_puti((int) i+1);
		usart_puts(" : ");
		DS18X20_show_id_uart( &gSensorIDs[i][0], OW_ROMCODE_SIZE );
		usart_puts( "\r" );
	}
	#endif

	for (i=0; i<nSensors; i++) {
		usart_puts("Sensor# ");
		usart_puti((int) i+1);
		usart_puts(" is a ");
		if ( gSensorIDs[i][0] == DS18S20_ID)
			usart_puts("DS18S20/DS1820");
		else
			usart_puts("DS18B20");
	    usart_puts(" which is ");
		if ( DS18X20_get_power_status( &gSensorIDs[i][0] ) == DS18X20_POWER_PARASITE ) 
			usart_puts( "parasite" );
		else
			usart_puts( "externally" ); 
		usart_puts( " powered\r" );
	}
	
	if ( nSensors == 1 ) {
		usart_puts( "\rThere is only one sensor -> Demo of \"read_meas_single\":\r" ); 
		i = gSensorIDs[0][0]; // family-code for conversion-routine
		DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL );
		delay_ms(DS18B20_TCONV_12BIT);
		DS18X20_read_meas_single(i, &subzero, &cel, &cel_frac_bits);
		usart_put_temp(subzero, cel, cel_frac_bits);
		usart_puts("\r");
	}

	// Enable radio
	high(PORTC, CYWM_nPD);
	high(PORTC, CYWM_nRESET);
	usart_puts("Radio enabled\n\r");

	usart_puts("TX: This will be a TX radio\n\r");

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
	//CYWM_WriteReg( REG_RX_INT_EN, 0x03 );			// Enable EOF and FULL interrupts for channel A
	CYWM_WriteReg( REG_TX_INT_EN, 0x01 );			// Enable EMPTY interrupt
	//CYWM_WriteReg( REG_CONTROL, 0x80 );				// Enable RX
	CYWM_WriteReg( REG_CONTROL, 0x40 );				// Enable TX


	// Repeat indefinitely
	for (;;)
	{
		delay_hs(50);

		i = gSensorIDs[0][0]; // family-code for conversion-routine
		DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL );
		delay_ms(DS18B20_TCONV_12BIT);
		DS18X20_read_meas_single(i, &subzero, &cel, &cel_frac_bits);
		usart_put_temp(subzero, cel, cel_frac_bits);
		usart_puts("\r");
		wireless_put_temp(subzero, cel, cel_frac_bits);
		wireless_puts("\r");

		// Echo received characters
		//data = usart_getc();
		//usart_putc(data);
	}

	return 0;
}
