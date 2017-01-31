/**
  ******************************************************************************
  * @file    nrf24L01p.h
  * @author  Saleh Mehdikhani <saleh.mehdikhani@gmail.com>, www.nooby.ir
  * @version V1.0.0
  * @date    28-Nov-2016
  * @brief   Header file of nrf24L01p configuration and operation.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NRF24L01P_H
#define __NRF24L01P_H

/* Includes ------------------------------------------------------------------*/
#include <mega88a.h>
#include <stdbool.h>

#define CE PORTB.1
#define CSN PORTB.2
#define IRQ PINB.0

/* Exported types ------------------------------------------------------------*/

/** 
  * @brief	Error Code. Return Value when a data is written to nrf24l01p memory.  
  */
typedef enum {PARAMETER_ERROR, UNKNOWN_COMMAND, BAD_COMMAND, OK} ErrorCode;

/** 
  * @brief	Mode of Operation. Indicate that nrf24 is configured as transmitter or receiver.  
  */
typedef enum {NRF24_TRANSMITTER, NRF24_RECEIVER} Mode;

/** 
  * @brief	Data rate.  How fast data moves through the air.  
  */
typedef enum {NRF24_250Kbps, NRF24_1Mbps, NRF24_2Mbps} NRF24_BaudRate;

/** 
  * @brief	Address Width. Number of bytes used to address the device.
  */
typedef enum {NRF24_3Byte, NRF24_4Byte, NRF24_5Byte} NRF24_AddressWidth;

/** 
  * @brief	Power Amplifier level.
  */
typedef enum {NRF24_m18dBm, NRF24_m12dBm, NRF24_m6dBm, NRF24_0dBm} NRF24_TXPower;

/** 
  * @brief	Answer Format. When writing to nrf24 memory, the answer is combinatioan of error code (if any) and status register.
  */
typedef struct {
    unsigned char status;
    ErrorCode error;
} WriteAnswer;

/* Exported functions --------------------------------------------------------*/

/* Initialization and configuration functions ********************************/
void nRF_Config(Mode mode);
void setMode(Mode m);
void setCRCScheme(unsigned char num);
void setPowerUp();
void setPowerDown();
void setBaudRate(NRF24_BaudRate br);
void setAutoAck(bool param);
void enableRxDataPipe(bool param);
void setAddressWidth(NRF24_AddressWidth aw);
void serRFChannel(unsigned char ch);
void setTXPower(NRF24_TXPower power);
void setDynamicPayloadLength(bool param); 

/* Input and Output operation functions **************************************/
WriteAnswer writeCommand(unsigned char ins, char* data, int size);
void sendData(char *data, int size);
unsigned char bytesAvailable();
void readRxFIFO(char* data, unsigned char size);
unsigned char getStatus();

/* Interrupt functions *******************************************************/
interrupt [PC_INT0] void pin_change_isr0(void);
void setInterruptMask(bool RX_DR, bool TX_DS, bool MAX_RT);
void clearInterruptFlag(bool RX_DR, bool TX_DS, bool MAX_RT);

/*
 * service routine of IRQ change state (this routin is called when IRQ signal of nrf24L01p has changed)
 */


/*
 * number of bytes available in RX FIFO
 */


/*
 * Initialize LED Service.
 */
 //read last RX FIFO

/*
 * Initialize LED Service.
 */
 //set operation mode of NRF24 (Transmitter or Receiver)

/*
 * Initialize LED Service.
 */
 //set CRC encoding scheme, 0=No CRC, 1 or 2 byte, others input are illegal

/*
 * Initialize LED Service.
 */
 //set device power up

/*
 * Initialize LED Service.
 */
 //set device power down

/*
 * Initialize LED Service.
 */
 //enable/disable interrupts by masking them, 1 means enable

/*
 * Initialize LED Service.
 */
 //clears interrput flag, 1=clear flag, 0=do nothing

/*
 * Initialize LED Service.
 */
 //set baudrate of on AIR data transmision

/*
 * Initialize LED Service.
 */
 //set data pipe 0 auto ack enable/disable 1=enable 0=disable

/*
 * Initialize LED Service.
 */
 //enable or disable RX data pipe 0 to receive data, 1=enable 0=disable

/*
 * Initialize LED Service.
 */
 //set RX/TX address width

/*
 * Initialize LED Service.
 */
 //set the channel, ch should be between 0 and 125 (125 channel is available), every channel has 1MHz bandwidth

/*
 * Initialize LED Service.
 */
 //set TX power

/*
 * Initialize LED Service.
 */
//enable/disable dynamic payload length on data pipe 0

/*
 * Initialize LED Service.
 */
 //get status register of NRF24L01p
#endif 