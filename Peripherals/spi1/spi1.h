#ifndef __SPI1_H
#define __SPI1_H
#include "stm32f10x.h"


// SPIx速度設定
#define SPI_SPEED_2   0
#define SPI_SPEED_8   1
#define SPI_SPEED_16  2
#define SPI_SPEED_256 3
						  	    													  
//函式原形宣告
void SPI1_Init(void);				 //初始化SPIx
void SPI1_SetSpeed(u8 SpeedSet);	 //設置SPIx速度
u8 SPI1_ReadWriteByte(u8 TxData);	 //從SPIx讀寫取一個Byte
		 
#endif

