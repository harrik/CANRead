/*
 * CANRead.c
 *
 * Created: 15.1.2015 0:19:57
 *  Author: harri
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define ACK 0x7E
#define LONG_TIME 10000


//Initialize SPI Master Device
void spi_init_master (void)
{
	DDRC = (1<<PORTC2);              //For testing
	DDRB = (1<<PORTB5)|(1<<PORTB3)|(1<<PORTB2);              //Set MOSI, SCK as Output
	//SPCR = (1<<SPE)|(1<<MSTR); //Enable SPI, Set as Master
	PORTB = PORTB|(1<<PORTB2);
	SPCR=(1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPR1);
	_delay_ms(10);

}

//Function to send and receive data
unsigned char spi_tranceiver (unsigned char data)
{
	unsigned char dummy;
	unsigned char mcp_read_reg;
	unsigned char mcp_reg_canctrl;
	unsigned char mcp_reset;
	
	mcp_read_reg = 0b00000011;
	mcp_reg_canctrl = 0x0f;
	mcp_reset = 0b11000000;

	PORTB &= ~(1<<2);
	SPDR = mcp_reset;
	while (!(SPSR & (1<<SPIF)));
	PORTB = PORTB | (1<<PORTB2);
	_delay_ms(200);
	
	
	
	while (1==1)
	{
		
		dummy = 0b00000000;
		PORTB &= ~(1<<2);
		
		SPDR = mcp_read_reg;
		while (!(SPSR & (1<<SPIF)));
		
		SPDR = mcp_reg_canctrl;
	    while (!(SPSR & (1<<SPIF)));
		
		SPDR = dummy;
		while (!(SPSR & (1<<SPIF)));
		
		
		_delay_ms(5);
	
		PORTB = PORTB | (1<<PORTB2); 
		_delay_ms(10);
		
			}
	SPDR = data;                       //Load data into the buffer
	//while(!(SPSR)&(1<<SPIF));
	
	while (!(SPSR & (1<<SPIF)))
	;
	return(SPDR);                      //Return received data
}


//Main
int main(void)
{
	DDRC= 0XFF;
	spi_init_master();
	spi_tranceiver(0b10100000);
	//spi_tranceiver(6);
	//spi_tranceiver(6);


}


