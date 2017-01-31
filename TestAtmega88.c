/*******************************************************
This program was created by the
CodeWizardAVR V3.12 Advanced
Automatic Program Generator
© Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 11/28/2016
Author  : 
Company : 
Comments: 


Chip type               : ATmega88PA
Program type            : Application
AVR Core Clock frequency: 8.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 256
*******************************************************/

// #define SENDER 1
#define RECEIVER 1

#include <mega88a.h>
#include <stdio.h>
#include <spi.h>
#include <delay.h>
#include "nRF24L01p.h"
#include <string.h>

char data[32] = {0};
char receiveData[32] = {0};
int i=0;
char t1;
unsigned char receiveCounter = 0;

void main(void)
{
// Declare your local variables here
unsigned int count = 0;
unsigned int lastCount = 0;
// Crystal Oscillator division factor: 1
#pragma optsize-
CLKPR=(1<<CLKPCE);
CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif

// Input/Output Ports initialization
// Port B initialization
// Function: Bit7=In Bit6=In Bit5=Out Bit4=In Bit3=Out Bit2=Out Bit1=Out Bit0=In 
DDRB=(0<<DDB7) | (0<<DDB6) | (1<<DDB5) | (0<<DDB4) | (1<<DDB3) | (1<<DDB2) | (1<<DDB1) | (0<<DDB0);
// State: Bit7=T Bit6=T Bit5=0 Bit4=T Bit3=0 Bit2=0 Bit1=0 Bit0=T 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRC=(0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
// State: Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTC=(0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=Out Bit3=Out Bit2=Out Bit1=In Bit0=In 
DDRD=(0<<DDD7) | (0<<DDD6) | (0<<DDD5) | (1<<DDD4) | (1<<DDD3) | (1<<DDD2) | (0<<DDD1) | (0<<DDD0);
// State: Bit7=T Bit6=T Bit5=T Bit4=0 Bit3=0 Bit2=0 Bit1=T Bit0=T 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// Interrupt on any change on pins PCINT0-7: On
// Interrupt on any change on pins PCINT8-14: Off
// Interrupt on any change on pins PCINT16-23: Off
EICRA=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
EIMSK=(0<<INT1) | (0<<INT0);
PCICR=(0<<PCIE2) | (0<<PCIE1) | (1<<PCIE0);
PCMSK0=(0<<PCINT7) | (0<<PCINT6) | (0<<PCINT5) | (0<<PCINT4) | (0<<PCINT3) | (0<<PCINT2) | (0<<PCINT1) | (1<<PCINT0);
PCIFR=(0<<PCIF2) | (0<<PCIF1) | (1<<PCIF0);

// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: Off
// USART Transmitter: On
// USART0 Mode: Asynchronous
// USART Baud Rate: 250000
UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
UCSR0B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
UBRR0H=0x00;
UBRR0L=0x01;

// SPI initialization
// SPI Type: Master
// SPI Clock Rate: 2*2000.000 kHz
// SPI Clock Phase: Cycle Start
// SPI Clock Polarity: Low
// SPI Data Order: MSB First
SPCR=(0<<SPIE) | (1<<SPE) | (0<<DORD) | (1<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);
SPSR=(1<<SPI2X);

// Global enable interrupts
#asm("sei")

#ifdef SENDER
	nRF_Config(NRF24_TRANSMITTER); //set module as transmitter
	#asm("sei")
	for(i=0;i<32;i++)
		data[i] = 's';

	while (1)
	{
		//two first byte of sent data are count
		data[0] = count;
		data[1] = count>>8;
		
		//send the packet over the air
		sendData(data, 32);
		
		PORTD.2=~PORTD.2;
		
		//wait till every thing is stable
		delay_us(600);
		
		//increment counter
		count++;
	}
#elif RECEIVER
	nRF_Config(NRF24_RECEIVER); //set module as receiver
	#asm("sei")
	while (1)
	{
		//how many bytes are available?
		t1 = bytesAvailable(); 
		
		if(t1 > 0) //is there any data?
		{
			//read data from buffer and copy into receiveData
			readRxFIFO(receiveData, t1);
			
			//what is the count value that is sent
			count = ((unsigned int)receiveData[0])+( ((unsigned int)receiveData[1])<<8);
			
			//if any packet is not lost, lastCount = count-1
			if(count-lastCount != 1){
				PORTD.2 = ~PORTD.2;
			}
			
			//store current count value
			lastCount=count; 
			
			//send count value to the uart
			sprintf(data, "%d\r",count);
			puts(data);
			
			//number of bytes received over the air
			receiveCounter++;
			if(receiveCounter==0) //overflow of receive packet counter
				PORTD.4=~PORTD.4;
			
		}
	}
#endif

}
