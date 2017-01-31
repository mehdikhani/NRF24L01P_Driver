# NRF24L01P_Driver
This driver initialize NRF24L01P module and sends/receives data over the air.
Using of these files are described here: http://nooby.ir/blog/project/37-nrf24l01p-opensource

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
