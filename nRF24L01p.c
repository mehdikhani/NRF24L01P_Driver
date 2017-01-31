/**
  ******************************************************************************
  * @file    nrf24L01p.c
  * @author  Saleh Mehdikhani <saleh.mehdikhani@gmail.com>, www.nooby.ir
  * @version V1.0.0
  * @date    28-Nov-2016
  * @brief   Driver for nrf24L01p configuration and operation.
  *    
  *         This file provides firmware functions to manage the following 
  *         functionalities of the nrf24l01+ configuration and operations
  *           + Initialization and configuration functions
  *           + IO operation functions
  *           + Interrupt related functions
  @verbatim     
  ==============================================================================      
                        ##### How to use this driver #####
  ============================================================================== 
  [..]
   (#) Set the CE and CSN port at nrf24L01p.h if it is diffrent from default ones.

   (#) Initialize and config the module by calling nRF_Config() function.
	   the parameter sets the module as Transmitter or Receiver, In the same
	   time, just one mode of operation is allowed.

   (#) In case of Transmitter:
	   Use sendData() function in order to send a data array of maximum 32 byte
                    
   (#) In case of Receiver:
	   Check if any new data has received using bytesAvailable().
	   If any byte is available, read received bytes by calling readRxFIFO().
	   Until readRxFIFO() is not called, all new received data will be droped.

     *** Defaul configuration ***    
     =================================== 
    [..]
      (+) Auto Acknowledgment: Disabled.
      (+) Active Data Pipe: 0 
      (+) RF Channel: 1
      (+) TX Power: 0dBm
      (+) Baud Rate: 1 Mbps
	  (+) Device Address: 0x00,0x01,0x03,0x07,0x00
	  (+) Dynamic Payload Length: Enabled
	  (+) CRC Width: 2 Byte

  ==============================================================================      
                        ##### Circuit Schematic #####
  ==============================================================================
  [..]
   (#) This project is based on Atmega88 ICs and codevision compiler, but it 
	   can be used in other platform by changing some lines of code.
	   The circuit used to test this project is as below (both transmitter &
	   receiver)
	   
	              ^
				  |
		+---------+---------+         +---+
		|        VCC      D2+---------+LED+----+GND
		|                   |         +---+
		|                   |
		|                   |         +---+
		|                 D4+---------+LED+----+GND
		|                   |         +---+                     ^
		|                   |                                   |
		|      AtMega88     |                       +-----------+------------+
		|                   |                       |          VCC           |
		|                   |                       |                        |
		|                 B1+-----------------------+CE                      |
		|                 B0|-----------------------|IRQ           NRF24L01P |
		|                 B2|-----------------------|CSN                     |
		|                 B5|-----------------------|SCK                     |
		|                 B4|-----------------------|MISO                    |
		|GND              B3+-----------------------+MOSI      GND           |
		+-+--+-----+--------+                       +-----------+------------+
		  |  |     |                                            |
		  |  |  _  |                                            |
		  v  |_| |_|                                            v
			   |_|

			 Crystal 8MHz

  
  @endverbatim
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2016 Saleh Mehdikhani <saleh.mehdikhani@gmail.com>
  *
  * This program is free software; you can redistribute it and/or
  * modify it under the terms of the GNU General Public License
  * version 3 as published by the Free Software Foundation.
  * https://www.gnu.org/licenses/gpl-3.0.en.html
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include <mega88a.h>
#include <nRF24L01p.h>
#include <stdio.h>
#include <delay.h>
#include <spi.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
/* Instruction Memories */
#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define ACTIVATE 0x50
#define R_RX_PL_WID 0x60
#define W_ACK_PAYLOAD 0xA8
#define W_TX_PAYLOAD_NOACK 0xB0
#define NOP 0xFF

/* Memory Map */
#define CONFIG 0x00
#define EN_AA 0x01
#define EN_RXADDR 0x02
#define SETUP_AW 0x03
#define SETUP_RETR 0x04
#define RF_CH 0x05
#define RF_SETUP 0x06
#define STATUS 0x07
#define OBSERVE_TX 0x08
#define RPD 0x09
#define RX_ADDR_P0 0x0A
#define RX_ADDR_P1 0x0B
#define RX_ADDR_P2 0x0C
#define RX_ADDR_P3 0x0D
#define RX_ADDR_P4 0x0E
#define RX_ADDR_P5 0x0F
#define TX_ADDR 0x10
#define RX_PW_P0 0x11
#define RX_PW_P1 0x12
#define RX_PW_P2 0x13
#define RX_PW_P3 0x14
#define RX_PW_P4 0x15
#define RX_PW_P5 0x16
#define FIFO_STATUS 0x17
#define DYNPD 0x1C
#define FEATURE 0x1D

/* Private variables ---------------------------------------------------------*/
unsigned char Base_Addrs[5]={0x00,0x01,0x03,0x07,0x00}; //address of this device
unsigned char Temp_Addrs[5]={0x00,0x01,0x03,0x07,0x00};
unsigned char payload[33]; //stores last received bytes
unsigned char receiveBytesAvailable = 0; //store numbers of bytes available in RX FIFO, reset when RX FIFO is read
Mode operationMode; //which mode the device is, transmitter or receiver

#pragma used+
/* library function prototypes */

/** @defgroup nrf24L01p Initialization and configuration functions
 *  @brief   Initialization and configuration functions 
 *
@verbatim   
 ===============================================================================
             ##### Initialization and configuration functions  #####
 ===============================================================================  
    [..]
    This section provides functions allowing to initialize and configures 
    the nrf24L01p module.
    [..] 

@endverbatim
  * @{
  */
 
/**
  * @brief  Initialize and configures modules by default parameters.
  *         
  * @param	mode: Mode of operation, Transmitter or Receiver.
  * @retval NONE
  */
void nRF_Config(Mode mode)
{
    char data[10];
	
	CSN = 1; 
    CE = 0;
           
    delay_ms(110);
    
    operationMode = mode;
	
	setPowerDown();
	delay_ms(5);
	data[0] = 0x7E;
	writeCommand(W_REGISTER+STATUS, data, 1); //write 1 to clear interrupt flags
	writeCommand(FLUSH_TX, NULL, 0); //flush TX FIFO
	writeCommand(FLUSH_RX, NULL, 0); //flush TX FIFO
	
	// enable auto acknowledge
	setAutoAck(0); //disable auto ACK
    
	//enable data pipe0 reciever
	enableRxDataPipe(1); //Enable data pipe 0
      
	//set address width to 5 byte
	setAddressWidth(NRF24_3Byte);

	//set rf channel
	serRFChannel(1);
    
	//set baud rate and power
	setTXPower(NRF24_0dBm);
	setBaudRate(NRF24_1Mbps);
    
	//write base address of RX in pipe 0
	writeCommand(W_REGISTER+RX_ADDR_P0, Base_Addrs, 5); //Command:W_REGISTER on address 0A (RX_ADDR_P0, Receive address data pipe 0. 5 Bytes maximum)
    
	//write base address of TX in pipe 0
	writeCommand(W_REGISTER+TX_ADDR, Base_Addrs, 5); //Command:W_REGISTER on address 10 (Transmit address. Used for a PTX device only)
    
	//enable dynamic payload lenght
	setDynamicPayloadLength(1);
    
	//set CRC length
	setCRCScheme(2); //CRC is 2 byte
	
	if(mode==NRF24_TRANSMITTER){
		setInterruptMask(0, 1, 1); //enable TX_DS and MAX_RT
		setMode(NRF24_TRANSMITTER); //set device mode to be PTX
		setPowerUp();
		delay_ms(100);
	}else if(mode==NRF24_RECEIVER){
		setInterruptMask(1, 0, 0); //enable RX_DR
		setMode(NRF24_RECEIVER); //set device mode to be PRX
		setPowerUp();
		delay_ms(5);
        CE = 1;
	}
}
 
/**
  * @brief  Sets mode of operation, Change other configuration if this function has used.
  *         
  * @param	m: Mode of operation.
  * @retval NONE.
  */
void setMode(Mode m)
{
	char data[1];
	
	writeCommand(R_REGISTER+CONFIG, data, 1); //read current config register
	
	if(m==NRF24_TRANSMITTER) //if transmitter
    {
		data[0] &= 0xFE; //clear bit 0 (set device to PTX)
    }
    else if(m==NRF24_RECEIVER) //it is receiver
    {
		data[0] |= 0x01; //set bit 0 (set device to PRX)
    }
	
	writeCommand(W_REGISTER+CONFIG, data, 1); //write data to update CONFIG
}

/**
  * @brief  Sets the number of CRC bytes.
  *         
  * @param	num: Number of CRC bytes.
  * @retval NONE.
  */
void setCRCScheme(unsigned char num)
{
	char data[1];
	
	writeCommand(R_REGISTER+CONFIG, data, 1); //read current config register
	
	switch(num){
		case 0:
			data[0] &= 0xF7; //clear bit 3, disable CRC, NOTICE: if auto ACK is enabled, it is forced high
		break;
		
		case 1:
			data[0] |= 0x08; //set bit 3, enable CRC
			data[0] &= 0xFB; //clear bit 2, 1 byte CRC
		break;
		
		case 2:
			data[0] |= 0x08; //set bit 3, enable CRC
			data[0] |= 0x04; //set bit 2, 2 byte CRC
		break;
	}
	
	writeCommand(W_REGISTER+CONFIG, data, 1); //write data
}

/**
  * @brief  Powers On the module if it is not.
  *         
  * @param	NONE.
  * @retval NONE.
  */
void setPowerUp()
{
	char data[1];
	
	writeCommand(R_REGISTER+CONFIG, data, 1); //read current config register
	data[0] |= 0x02; //set bit 2 (power up)
	writeCommand(W_REGISTER+CONFIG, data, 1); //write data
}

/**
  * @brief  Powers Down the module if it is not.
  *         
  * @param	NONE.
  * @retval NONE.
  */
void setPowerDown()
{
	char data[1];
	
	writeCommand(R_REGISTER+CONFIG, data, 1); //read current config register
	data[0] &= 0xFD; //clear bit 2 (power down)
	writeCommand(W_REGISTER+CONFIG, data, 1); //write data
}

/**
  * @brief  Sets baud rate of nrf24 over  the air.
  *         
  * @param	br: Baud rate of data transmission.
  * @retval NONE.
  */
void setBaudRate(NRF24_BaudRate br)
{
	char data[1];
	
	writeCommand(R_REGISTER+RF_SETUP, data, 1); //read current RF_SETUP register
	
	switch(br){
		case NRF24_250Kbps:
			data[0] |= 0x20; //set bit 5 (set RF_DR_LOW)
			data[0] &= 0xF7; //clear bit 3 (clear RF_DR_HIGH)
		break;
		
		case NRF24_1Mbps:
			data[0] &= 0xDF; //clear bit 5 (clear RF_DR_LOW)
			data[0] &= 0xF7; //clear bit 3 (clear RF_DR_HIGH)
		break;
		
		case NRF24_2Mbps:
			data[0] &= 0xDF; //clear bit 5 (clear RF_DR_LOW)
			data[0] |= 0x08; //clear bit 3 (clear RF_DR_HIGH)
		break;
	}
	
	writeCommand(W_REGISTER+RF_SETUP, data, 1); //write data
	
}

/**
  * @brief  Enables or Disables auto acknowledge.
  *         
  * @param	param: 1: Enable, 0:Disable.
  * @retval NONE.
  */
void setAutoAck(bool param)
{
	char data[1];
	
	writeCommand(R_REGISTER+EN_AA, data, 1); //read current EN_AA register
	
	if(param==1) //enable auto ack
		data[0] |= 0x01; //set bit 0, enable auto ACK
	else //disable
		data[0] &= 0xFE; //clear bit 0, disable auto ACK
		
	writeCommand(W_REGISTER+EN_AA, data, 1); //Command:W_REGISTER on address 01 (EN_AA, Enable ‘Auto Acknowledgment’ Function)
}

/**
  * @brief  Enables or Disables data pipe 0 to receive data.
  *         
  * @param	param: 1: Enable, 0:Disable.
  * @retval NONE.
  */
void enableRxDataPipe(bool param)
{
	char data[1];
	
	writeCommand(R_REGISTER+EN_RXADDR, data, 1); //read current EN_AA register
	
	if(param==1)
		data[0] |= 0x01; //set bit 0, enable data pipe 0
	else //disable
		data[0] &= 0xFE; //clear bit 0, disable data pipe 0
		
	writeCommand(W_REGISTER+EN_RXADDR, data, 1); //Command:W_REGISTER on address 02 (EN_RXADDR, Enabled RX Addresses)
}

/**
  * @brief  Sets the address width of nrf24 module.
  *         
  * @param	aw: Number of address bytes.
  * @retval NONE.
  */
void setAddressWidth(NRF24_AddressWidth aw)
{
	char data[1];
	
	writeCommand(R_REGISTER+SETUP_AW, data, 1); //read current SETUP_AW register
	
	switch(aw){
		case NRF24_3Byte:
			data[0] |= 0x01; //set bit 0
			data[0] &= 0xFD; //clear bit 1
		break;
		
		case NRF24_4Byte:
			data[0] &= 0xFE; //clear bit 0
			data[0] |= 0x02; //set bit 1
		break;
		
		case NRF24_5Byte:
			data[0] |= 0x03; //set bit 0 and bit 1
		break;
	}
		
	writeCommand(W_REGISTER+SETUP_AW, data, 1); //Command:W_REGISTER on address 03 (SETUP_AW, Setup of Address Widths)
}

/**
  * @brief  Sets the RF channel (1 Mhz is space between two channel).
  *         
  * @param	ch: Number of channel.
  * @retval NONE.
  */
void serRFChannel(unsigned char ch)
{
	char data[1];
	
	if(ch<=125){
		data[0] = ch;
		writeCommand(W_REGISTER+RF_CH, data, 1); //Command:W_REGISTER on address 05 (RF_CH, RF Channel)
	}
}

/**
  * @brief  Sets the TX power level.
  *         
  * @param	power: TX power level.
  * @retval NONE.
  */
void setTXPower(NRF24_TXPower power)
{
	char data[1];
	
	writeCommand(R_REGISTER+RF_SETUP, data, 1); //read current RF_SETUP register
	
	switch(power){
		case NRF24_m18dBm:
			data[0] &= 0xF9; //clear bit 1 and bit 2
		break;
		
		case NRF24_m12dBm:
			data[0] |= 0x02; //set bit 1
			data[0] &= 0xFB; //clear bit 2
		break;
		
		case NRF24_m6dBm:
			data[0] &= 0xFD; //clear bit 1
			data[0] |= 0x04; //set bit 2
		break;
		
		case NRF24_0dBm:
			data[0] |= 0x06; //set bit 1 and bit 2
		break;
	}
		
	writeCommand(W_REGISTER+RF_SETUP, data, 1); //Command:W_REGISTER on address 06 (RF_SETUP, RF Setup Register)   
}

/**
  * @brief  Enable or Disable dynamic payload lenghth, If disabled, Transmitter have to send 32 bytes of data at any transmission.
  *         
  * @param	param: 1:Enabled, 0:Disabled.
  * @retval NONE.
  */
void setDynamicPayloadLength(bool param)
{
	char data[1];
	
	data[0] = 0;
	//enable/disable DPL_P0 bit of DYNPD register
	// writeCommand(R_REGISTER+DYNPD, data, 1); //read current DYNPD register
	if(param==1) //enable auto ack
		data[0] |= 0x01; //set bit 0, enable dynamic payload length
	else //disable
		data[0] &= 0xFE; //clear bit 0, disable dynamic payload length
	data[0] &= 0x3F;
	writeCommand(W_REGISTER+DYNPD, data, 1); //Command:W_REGISTER on address 1C (DYNPD, Enable dynamic payload length) 
	
	data[0] = 0;
	//enable/disable EN_DPL of FEATURE register
	// writeCommand(R_REGISTER+FEATURE, data, 1); //read current FEATURE register
	if(param==1) //enable auto ack
		data[0] |= 0x04; //set bit 2, enable dynamic payload length
	else //disable
		data[0] &= 0xFB; //clear bit 2, disable dynamic payload length
	data[0] &= 0x07;
	writeCommand(W_REGISTER+FEATURE, data, 1); //Command:W_REGISTER on address 1D (FEATURE, Feature Register)
}

/** @defgroup nrf24L01p Initialization and configuration functions
 *  @brief   Initialization and configuration functions 
 *
@verbatim   
 ===============================================================================
				##### Input and Output operation functions  #####
 ===============================================================================  
    [..]
    This section provides functions needed to send and receive data using nrf24l01p
    [..] 

@endverbatim
  * @{
  */

/**
  * @brief  Writes into the nrf24 memory via SPI
  *         
  * @param  ins: instruction, as defined "Instruction Memories"
  * @param	data: data to be written in nrf24 memory
  * @param	size: size of data, 0 if no data is used.
  * @retval status register of nrf24 and error code if any else OK
  */
WriteAnswer writeCommand(unsigned char ins, char* data, int size)
{
	WriteAnswer returnValue;
	unsigned char answer = 0; //stores last status that is read from NRF24
	ErrorCode error = OK; //indicates any error in system
	int i=0;
	
	if( (ins&0xE0) == R_REGISTER ){ //if it is read command
		switch( ins&0x1F ){ //based on mamory map address (write register is : 001x xxxx that xxxxx is address of memory map
			case CONFIG: //Configuration Register
			case EN_AA: //Enable ‘Auto Acknowledgment’ Function
			case EN_RXADDR: //Enabled RX Addresses
			case SETUP_AW: //Setup of Address Widths
			case SETUP_RETR: //Setup of Automatic Retransmission
			case RF_CH: //RF Channel
			case RF_SETUP: //RF Setup Register
			case STATUS: //Status Register
			case OBSERVE_TX: //Transmit observe register
			case RPD: //Carrier Detect (Read only)
			case RX_ADDR_P2: //Receive address data pipe 2. Only LSB, MSB is equal to RX_ADDR_P1[39:8]
			case RX_ADDR_P3: //Receive address data pipe 3. Only LSB, MSB is equal to RX_ADDR_P1[39:8]
			case RX_ADDR_P4: //Receive address data pipe 4. Only LSB, MSB is equal to RX_ADDR_P1[39:8]
			case RX_ADDR_P5: //Receive address data pipe 5. Only LSB, MSB is equal to RX_ADDR_P1[39:8]
			case RX_PW_P0: //Number of bytes in RX payload in data pipe 0
			case RX_PW_P1: //Number of bytes in RX payload in data pipe 1
			case RX_PW_P2: //Number of bytes in RX payload in data pipe 2
			case RX_PW_P3: //Number of bytes in RX payload in data pipe 3
			case RX_PW_P4: //Number of bytes in RX payload in data pipe 4
			case RX_PW_P5: //Number of bytes in RX payload in data pipe 5
			case FIFO_STATUS: //FIFO Status Register
			case DYNPD: //Enable dynamic payload length
			case FEATURE: //Feature Register
				if(size==1){
					CSN=0; //select the chip to send spi command
					answer=spi(ins); //write command
					data[0] = spi(NOP); //read data
					CSN=1;  //deselect the chip
				}else{
					error = PARAMETER_ERROR; //parameter is not correct
				}
			break;
			
			case RX_ADDR_P0: //Receive address data pipe 0. 5 Bytes maximum
			case RX_ADDR_P1: //Receive address data pipe 1. 5 Bytes maximum
			case TX_ADDR: //Transmit address. Used for a PTX device only, Set RX_ADDR_P0 equal to this address to handle automatic acknowledge if this is a PTX device with Enhanced ShockBurst™ enabled
				if(size<=5){
					CSN=0; //select the chip to send spi command
					answer=spi(ins); //write command
					for(i=size-1 ; i>=0 ; i--)
						data[i] = spi(NOP); //read data
					CSN=1;  //deselect the chip
				}else{
					error = PARAMETER_ERROR; //parameter is not correct
				}
			break;
			
			default:
				error = UNKNOWN_COMMAND; //command is not supported on this version
			
		}//end of switch
	}else if( (ins&0xE0) == W_REGISTER ){ //if it is write command
		switch( ins&0x1F ){ //based on mamory map address (write register is : 001x xxxx that xxxxx is address of memory map
			case CONFIG: //Configuration Register
			case EN_AA: //Enable ‘Auto Acknowledgment’ Function
			case EN_RXADDR: //Enabled RX Addresses
			case SETUP_AW: //Setup of Address Widths
			case SETUP_RETR: //Setup of Automatic Retransmission
			case RF_CH: //RF Channel
			case RF_SETUP: //RF Setup Register
			case STATUS: //Status Register
			case RX_ADDR_P2: //Receive address data pipe 2. Only LSB, MSB is equal to RX_ADDR_P1[39:8]
			case RX_ADDR_P3: //Receive address data pipe 3. Only LSB, MSB is equal to RX_ADDR_P1[39:8]
			case RX_ADDR_P4: //Receive address data pipe 4. Only LSB, MSB is equal to RX_ADDR_P1[39:8]
			case RX_ADDR_P5: //Receive address data pipe 5. Only LSB, MSB is equal to RX_ADDR_P1[39:8]
			case RX_PW_P0: //Number of bytes in RX payload in data pipe 0
			case RX_PW_P1: //Number of bytes in RX payload in data pipe 1
			case RX_PW_P2: //Number of bytes in RX payload in data pipe 2
			case RX_PW_P3: //Number of bytes in RX payload in data pipe 3
			case RX_PW_P4: //Number of bytes in RX payload in data pipe 4
			case RX_PW_P5: //Number of bytes in RX payload in data pipe 5
			case FIFO_STATUS: //FIFO Status Register
			case DYNPD: //Enable dynamic payload length
			case FEATURE: //Feature Register
				if(size==1){
					CSN=0; //select the chip to send spi command
					answer=spi(ins); //write command
					spi(data[0]); //write data
					CSN=1;  //deselect the chip
				}else{
					error = PARAMETER_ERROR; //parameter is not correct
				}
			break;
			
			case OBSERVE_TX: //Transmit observe register
			case RPD: //Carrier Detect (Read only)
				error = BAD_COMMAND; //it is not a writable command
			break;

			
			case RX_ADDR_P0: //Receive address data pipe 0. 5 Bytes maximum
			case RX_ADDR_P1: //Receive address data pipe 1. 5 Bytes maximum
			case TX_ADDR: //Transmit address. Used for a PTX device only, Set RX_ADDR_P0 equal to this address to handle automatic acknowledge if this is a PTX device with Enhanced ShockBurst™ enabled
				if(size<=5){
					CSN=0; //select the chip to send spi command
					answer=spi(ins); //write command
					for(i=size-1 ; i>=0 ; i--)
						spi(data[i]); //write data
					CSN=1;  //deselect the chip
				}else{
					error = PARAMETER_ERROR; //parameter is not correct
				}
			break;
			
			default:
				error = UNKNOWN_COMMAND; //command is not supported on this version
		} //end of switch
	}else if( (ins&0xE0) == W_ACK_PAYLOAD ){ //Used in RX mode, Write Payload to be transmitted together with ACK
		if(size<=32){ //the maximim size is 32
			CSN=0; //select the chip to send spi command
			answer = spi(ins); //command to read RX Payload
			for(i=size-1;i>=0;i--) //LSByte first
				spi(data[i]); //write data byte by byte
			CSN=1;  //deselect the chip
		}else{
			error = PARAMETER_ERROR; //parameter is not correct
		}
	}else{ //it is not read or write to mamory map
		switch(ins){
			case R_RX_PAYLOAD:
				if(size<=32){ //the maximim size is 32
					CSN=0; //select the chip to send spi command
					answer = spi(ins); //command to read RX Payload
					for(i=size-1;i>=0;i--) //LSByte first
						data[i] = spi(NOP); //read data byte by byte
					CSN=1;  //deselect the chip
				}else{
					error = PARAMETER_ERROR; //parameter is not correct
				}
			break;
			
			case W_TX_PAYLOAD:
			case W_TX_PAYLOAD_NOACK:
				if(size<=32){ //the maximim size is 32
					CSN=0; //select the chip to send spi command
					answer = spi(ins); //command to read RX Payload
					for(i=size-1;i>=0;i--) //LSByte first
						spi(data[i]); //write data byte by byte
					CSN=1;  //deselect the chip
				}else{
					error = PARAMETER_ERROR; //parameter is not correct
				}
			break;
			
			case FLUSH_TX:
			case FLUSH_RX:
			case REUSE_TX_PL:
			case NOP:
				if(size==0){
					CSN=0; //select the chip to send spi command
					answer=spi(ins); //write command
					CSN=1;  //deselect the chip
				}else{
					error = PARAMETER_ERROR; //parameter is not correct
				}
			break;
			
			case ACTIVATE:
				if(size==1){
					CSN=0; //select the chip to send spi command
					answer=spi(ins); //write command
					spi(data[0]); //write data
					CSN=1;  //deselect the chip
				}else{
					error = PARAMETER_ERROR; //parameter is not correct
				}
			break;
			
			case R_RX_PL_WID:
				if(size==1){
					CSN=0; //select the chip to send spi command
					answer=spi(ins); //write command
					data[0]=spi(NOP); //read payload width
					CSN=1;  //deselect the chip
				}else{
					error = PARAMETER_ERROR; //parameter is not correct
				}
			break;
			
			default:
				error = UNKNOWN_COMMAND; //command is not supported on this version
		}
	}
	
	returnValue.status = answer;
	return returnValue;
}

/**
  * @brief  Sends data over air when configured as trasmitter.
  *         
  * @param	data: data to be sent.
  * @param	size: size of data.
  * @retval NONE
  */
void sendData(char *data, int size)
{
	int counter = 0; 
    
	if((Temp_Addrs[4]==Base_Addrs[4]) && (Temp_Addrs[3]==Base_Addrs[3]) && (Temp_Addrs[2]==Base_Addrs[2]) && (Temp_Addrs[1]==Base_Addrs[1]) && (Temp_Addrs[0]==Base_Addrs[0]))
	{
		writeCommand(FLUSH_TX, NULL, 0);
		writeCommand(W_TX_PAYLOAD, data, size);        
		CE = 1;
		delay_us(15+130); //SE is 1 for more than 10us and 130us for TX settling time
		CE = 0;
	}
}

/**
  * @brief  Indicate number of bytes available to be read.
  *         
  * @param	NONE.
  * @retval Number of available received bytes.
  */
unsigned char bytesAvailable()
{
	return receiveBytesAvailable;
}

/**
  * @brief  Reads the packet available in buffer, till it is not read, next packet will be droped.
  *         
  * @param	data: Array to store received packet, Size of packet should be at leat size of received packet.
  * @param	size: size of data to read from packet and copy into data array.
  * @retval NONE.
  */
void readRxFIFO(char* data, unsigned char size)
{
	memcpy(data, payload, receiveBytesAvailable); //copy last read data
	receiveBytesAvailable = 0; //RX FIFO is read, so there is no data in buffer, NOTICE: while the last RX FIFO is not read, all incoming data are discarded, so there is no need to update this value before reading last RX FIFO

}

/**
  * @brief  Reads status register of nrf24l01p.
  *         
  * @param	NONE
  * @retval Status register value.
  */
unsigned char getStatus()
{
	return writeCommand(NOP, NULL, 0).status;
}

/** @defgroup nrf24L01p Initialization and configuration functions
 *  @brief   Initialization and configuration functions 
 *
@verbatim   
 ===============================================================================
							##### Interrupt functions  #####
 ===============================================================================  
    [..]
    This section provides functions needed to send and receive data using nrf24l01p
    [..] 

@endverbatim
  * @{
  */

// Pin change 0-7 interrupt service routine
/**
  * @brief  IRQ signal interrupt service routine
  *         
  * @param  NONE
  * @retval NONE
  */
interrupt [PC_INT0] void pin_change_isr0(void)
{
	char dataTemp[32] = {0};
	unsigned char status = 0; 
	
	if(IRQ==0){ //if IRQ is falling edge
		if(operationMode==NRF24_TRANSMITTER) //if it is transmitter
		{
			status = getStatus();
			if(status & W_REGISTER) //Data Sent TX FIFO interrupt
			{
				writeCommand(R_REGISTER+FIFO_STATUS, dataTemp, 1); //read FIFO_STATUS
				if((dataTemp[0] & 0x01)==0) //check RX FIFO empty flag, 0 means some data in RX FIFO
				{
					writeCommand(R_RX_PL_WID, dataTemp, 1); //Read RX-payload width
					if(dataTemp[0]<=32) //if answer is less than 32 byte
					{
						receiveBytesAvailable = dataTemp[0]; //number of bytes available in RX FIFO
					}
					else{
						writeCommand(FLUSH_RX, NULL, 0); //flush RX FIFO
						receiveBytesAvailable = 0; //there is no data available in FIFO
					}
				}          
			}else{ //it is not TX FIFO interrupt
				;
			}
		}                                                     
		else if(operationMode==NRF24_RECEIVER) //it is receiver
		{
			writeCommand(R_RX_PL_WID, dataTemp, 1); //Read RX-payload width
			if(dataTemp[0]<=32 && receiveBytesAvailable==0) //if answer is equal or less than 32 byte (maximum valid size)
			{
				receiveBytesAvailable = dataTemp[0]; //number of bytes available in RX FIFO
				writeCommand(R_RX_PAYLOAD, payload, receiveBytesAvailable); //read RX FIFO
			}
			else
			{
				writeCommand(FLUSH_RX, NULL, 0); //flush RX FIFO
				receiveBytesAvailable = 0; //there is no data available in FIFO
			}
		}

		//clear all interupt flags
		clearInterruptFlag(1,1,1);
		writeCommand(FLUSH_TX, NULL, 0); //flush TX FIFO
	}		
}

/**
  * @brief  Enable Interrupts of nrf24l01p module.
  *         
  * @param	RX_DR: if 1: Enable RX_DR interrupt, 0 Disable RX_DR interrupt.
  * @param	TX_DS: if 1: Enable TX_DS interrupt, 0 Disable TX_DS interrupt.
  * @param	MAX_RT: if 1: Enable MAX_RT interrupt, 0 Disable MAX_RT interrupt.
  * @retval NONE.
  */
void setInterruptMask(bool RX_DR, bool TX_DS, bool MAX_RT)
{
	char data[1];
	
	writeCommand(R_REGISTER+CONFIG, data, 1); //read current config register
	
	if(RX_DR==1)
		data[0] &= 0xBF; //clear bit 6 (enable RX_DR)
	else
		data[0] |= 0x40; //set bit 6 (disable RX_DR)
	
	if(TX_DS==1)
		data[0] &= 0xDF; //clear bit 5 (enable TX_DS)
	else
		data[0] |= 0x20; //set bit 5 (disable TX_DS)
	
	if(MAX_RT==1)
		data[0] &= 0xEF; //clear bit 4 (enable MAX_RT)
	else
		data[0] |= 0x10; //set bit 4 (disable MAX_RT)
	
	writeCommand(W_REGISTER+CONFIG, data, 1); //write data
}

/**
  * @brief  Clears pending interrupt flag if any.
  *         
  * @param	RX_DR: clear RX_DR interrupt pending flag.
  * @param	TX_DS: clear TX_DS interrupt pending flag.
  * @param	MAX_RT: clear MAX_RT interrupt pending flag.
  * @retval NONE.
  */
void clearInterruptFlag(bool RX_DR, bool TX_DS, bool MAX_RT)
{
	char data[1] = {0};
	
	if(RX_DR==1)
		data[0] |= 0x40;
	if(TX_DS==1)
		data[0] |= 0x20;
	if(MAX_RT==1)
		data[0] |= 0x10;
	
	writeCommand(W_REGISTER+STATUS, data, 1); //write 1 to clear interrupt flags
}
