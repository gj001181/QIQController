#include "max6675.h"
#include "spi1.h"

int32_t Engine_Temperature = 0;

void MAX6675_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//CS pin 
   	//開啟Poart_B時鐘
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//將CS拉高
	MAX6675_CS_H();

	//初始化Max6675用SPI
	SPI1_Init();
}

uint8_t Read_MAX6675(int32_t* temp)
{
	s16 temparature = 0;
	u8 data = 0;
	u8  status = 0;
	//拉低CS, 停止溫度轉換
	MAX6675_CS_L();
	//讀出2個byte
	data = 	SPI1_ReadWriteByte(0xFF); //MSB
	temparature = data<<8;
	temparature |= SPI1_ReadWriteByte(0xFF); //LSB

	status = data&0x01;
	temparature = (temparature>>3)&0x0FFF;
	*temp = (int32_t)(temparature*0.25*100);
	//拉高CS, 開始下次轉換
	MAX6675_CS_H();
	return status;
}

