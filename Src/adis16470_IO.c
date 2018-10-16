/***************************************************************************

 Author        : Neil Zhao  - CAST

 Date          : Dec 22th, 2008

 File          : ADIS16209_IO.c

 Hardware      : ADuC7026 and ADIS16209

 Description   : Use the GPIO to simulate the SPI communication of ADIS16209
	
***************************************************************************/
#include "stdint.h"
#include "stdbool.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#if (((MEMS_DEVICE == ADIS16209)||(MEMS_DEVICE == ADIS16470))&&(BUS_TYPE == BUS_IO))

#include "ADIS16470_IO.h"

#define GPIO_CS16470          GPIO_PIN_4
#define PORT_CS16470          GPIOA

#define GPIO_SCK16470         GPIO_PIN_5
#define PORT_SCK16470         GPIOA

#define GPIO_MISO16470        GPIO_PIN_6
#define PORT_MISO16470        GPIOA

#define GPIO_MOSI16470        GPIO_PIN_7
#define PORT_MOSI16470        GPIOA



#define SET_CS()		HAL_GPIO_WritePin(PORT_CS16470, GPIO_CS16470, GPIO_PIN_SET);   
#define CLR_CS()		HAL_GPIO_WritePin(PORT_CS16470, GPIO_CS16470, GPIO_PIN_RESET);   

#define	SET_SCL()		HAL_GPIO_WritePin(PORT_SCK16470, GPIO_SCK16470, GPIO_PIN_SET); 
#define	CLR_SCL()		HAL_GPIO_WritePin(PORT_SCK16470, GPIO_SCK16470, GPIO_PIN_RESET); 

#define SET_SDO()		HAL_GPIO_WritePin(PORT_MOSI16470, GPIO_MOSI16470, GPIO_PIN_SET); 
#define CLR_SDO()		HAL_GPIO_WritePin(PORT_MOSI16470, GPIO_MOSI16470, GPIO_PIN_RESET);

#define GP0DAT      HAL_GPIO_ReadPin(PORT_MISO16470, GPIO_MISO16470);

void delay (signed int length)
{
	length *= 50;           // Ôö¼Ó100±¶ÑÓÊ±£¬2018.2.11
	while (length >0)
    	length--;
}
//---------------------------------
//WriteToADIS16209ViaSpi();
//---------------------------------
//Function that writes to the ADIS16209 via the SPI port. It sends first the control
//word that includes the start address and then the data to write.
//When writing registers which take up multi-bytes, ADIS16209 must be written one byte by one byte.
//CS must be pulled high and then pulled low before writing the next byte
//--------------------------------------------------------------------------------
void WriteToADIS16470ViaSpi(unsigned char RegisterAddress, unsigned char *RegisterData)
{
	unsigned	char	ControlValue = 0;
	unsigned	int		ValueToWrite = 0;
	unsigned	char	RegisterIndex = 0;
	unsigned	char	i = 0;
	

		//Create the 8-bit header
		ControlValue = (RegisterAddress ) | 0x80;
		SET_SCL();
		delay(1);
		SET_CS();
		delay(1);
		CLR_CS();
		delay(1);
		//Write out the control word
		for(i=0; i<8; i++)
		{ 	
			CLR_SCL();
			if(0x80 == (ControlValue & 0x80))
			{
				SET_SDO();		//Send one to DIN pin of ADIS16209
			}
			else
			{
				CLR_SDO();		//Send zero to DIN pin of ADIS16209	 
			}
			delay(1);
			SET_SCL();
			delay(1);
			ControlValue <<= 1;	//Rotate data	
		}
		//And then the data
		ValueToWrite = *(RegisterData);
		for (i=0; i<8; i++)
		{
			CLR_SCL();
			if(0x80 == (ValueToWrite & 0x80))
			{
				SET_SDO();	  //Send one to DIN pin	of ADIS16209
			}
			else
			{
				CLR_SDO();	  //Send zero to DIN pin of ADIS16209
			}
			delay(1);
			SET_SCL();
			delay(1);
			ValueToWrite <<= 1;	//Rotate data
		}
	    SET_CS();				//bring CS high again
	    delay(2);
	
}

//---------------------------------
//ReadFromADIS16209ViaSpi();
//---------------------------------
//Function that reads from the ADIS16209 via the SPI port. ADIS16209 supports full duplex mode operation.
//It first send the control word that includes the start address which you are going to access and then 8 clcoks
//Now,the data on DOUT pin of ADIS16209 is the register which you accessed just before this operation
//Then repeat sending control word and clock, read data at the same time and now
//the data on DOUT pin of ADIS16209 is the register which you are going to read
//CS must be pulled high and then pulled low before reading the next register
//--------------------------------------------------------------------------------
void ReadFromADIS16470ViaSpi(unsigned int RegisterAddress, unsigned int *RegisterData)
{
	unsigned	int	ControlValue = 0;
	unsigned	int	ReceiveData = 0;
	unsigned	char	i = 0, j = 0;
	unsigned	int		iTemp = 0;

	
	
	
			//Create the 8-bit header
			ControlValue = RegisterAddress ;//every register of ADIS16209 takes up two bytes
			SET_SCL();
			delay(1);	
			SET_CS();
			delay(1);
			CLR_CS();	 //bring CS low
			delay(1);
	
			//Write out the control word
			for(i=0; i<16; i++)
			{
				CLR_SCL();
				if(0x8000== (ControlValue & 0x8000))
				{
					SET_SDO();	  //Send one to DIN pin	of ADIS16209
				}
				else
				{
					CLR_SDO();	  //Send zero to DIN pin of ADIS16209
				}
				delay(1);
				SET_SCL();
				ReceiveData <<= 1;		//Rotate data
				iTemp = GP0DAT;			//Read DOUT of ADIS16209
//				if(0x00000020 == (iTemp & 0x00000020))
				if(0x00000001 == iTemp)
				{
					ReceiveData |= 1;	
				}
				delay(1);
				ControlValue <<= 1;	//Rotate data
			}
			*(RegisterData) = ReceiveData;
			SET_CS();	//bring CS high again
			delay(2);	
		
	
} 

#endif
