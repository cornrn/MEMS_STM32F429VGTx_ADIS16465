#include "Global.h"
#include "Device.h"
#include "MainEntry.h"
#include "usart.h"
#include "stdio.h"

extern unsigned char f10ms;

void HeartPulse(void);
extern ADI_16465_HandleTypeDef g_hImu16465;

unsigned short FramLength=0;
unsigned char CheckSum8(unsigned char* pBuffer, int nLength);

void MainEntry(void)
{
	unsigned char Txdata[32];
	unsigned char *pf;
	InitDevice();
  //uint8_t send_data_buf[2]={0x55,0x55};
    while (1)
    {
		HeartPulse();
  	if (f10ms==1)
		{
			f10ms = 0;
		  ADI_16465_ReadImu(&g_hImu16465);
		//HAL_Delay(1);
		//ADI_16465_ReadburstImu(&g_hImu16465);
   ADI_16465_ParseImu(&g_hImu16465);
	//		printf("%f %f %f %f %f %f %f\n",g_hImu16465.fGyrX,g_hImu16465.fGyrY,g_hImu16465.fGyrZ,g_hImu16465.fAccX,g_hImu16465.fAccY,g_hImu16465.fAccZ,g_hImu16465.fTempretrue);
  //    HAL_UART_Transmit(&huart3, (uint8_t *)&send_data_buf[0], 2, 0xffff);
			HAL_Delay(1);
	  	ADI_16465_Readreg(&g_hImu16465);
			HAL_Delay(1);
		
			
	    Txdata[0]=0x55;
		Txdata[1]=0xaa;
		Txdata[2]=0x01;
		Txdata[3]=FramLength;
		Txdata[4]=FramLength>>8;
		FramLength++;
		pf=(unsigned char*)&g_hImu16465.fGyrX;
		Txdata[5]=pf[0];
		Txdata[6]=pf[1];
		Txdata[7]=pf[2];
		Txdata[8]=pf[3];
		pf=(unsigned char*)&g_hImu16465.fGyrY;
		Txdata[9]=pf[0];
		Txdata[10]=pf[1];
		Txdata[11]=pf[2];
		Txdata[12]=pf[3];
	    pf=(unsigned char*)&g_hImu16465.fGyrZ;
		Txdata[13]=pf[0];
		Txdata[14]=pf[1];
		Txdata[15]=pf[2];
		Txdata[16]=pf[3];
		pf=(unsigned char*)&g_hImu16465.fAccX;
		Txdata[17]=pf[0];
		Txdata[18]=pf[1];
		Txdata[19]=pf[2];
		Txdata[20]=pf[3];
		pf=(unsigned char*)&g_hImu16465.fAccY;
		Txdata[21]=pf[0];
		Txdata[22]=pf[1];
		Txdata[23]=pf[2];
		Txdata[24]=pf[3];
		pf=(unsigned char*)&g_hImu16465.fAccZ;
		Txdata[25]=pf[0];
		Txdata[26]=pf[1];
		Txdata[27]=pf[2];
		Txdata[28]=pf[3];
		Txdata[29]=(int16_t)(g_hImu16465.fTempretrue*100);
		Txdata[30]=(int16_t)(g_hImu16465.fTempretrue*100)>>8;
		Txdata[31]=CheckSum8(&Txdata[2],29);
		HAL_UART_Transmit(&huart3,(uint8_t*)&Txdata[0],32,100);
		
		}
    }
}
unsigned char CheckSum8(unsigned char* pBuffer, int nLength)
{
	unsigned char retval = 0x00;
	int i;
	
	for(i=0;i<nLength;i++)
	{
		retval += pBuffer[i];
	}
	return retval;
}
void HeartPulse()
{
	static long counter = 0;
	counter++;
	if (counter>1000000)
	{
		counter = 0;
		LED_Toggle(LED1);
	}
}
