#include "spi1.h"

void SPI1_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

   	//開啟Poart_A與SPI1時鐘
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1, ENABLE );	
	
   	 //SPI1腳位初始化
	/* 設置 SPI1 pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //復用推挽輸出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//將SCK,MISO腳位拉高
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
	GPIO_SetBits(GPIOA,GPIO_Pin_6);

	/* SPI1 設置 */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //設置SPI單向或是雙向的數據傳輸模式→SPI1設定為雙向雙向雙全工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//設置SPI為主SPI,Mater
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//設置SPI數據傳輸大小→SPI發送與接收8bit的Byte結構
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							//設置SPI在閒置時SCK狀態
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						//設置SPI在第一個SCK上升/下降或是第二個SCK上升/下降時傳送/讀取資料
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//NSS訊號由硬體或是軟體控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//設置SPI使用BaudRate預分頻
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//設置數據傳輸是MSB→LSB或是LSB→MSB :設定為MSB開始
	SPI_InitStructure.SPI_CRCPolynomial = 7;							//CRC值計算的多項式
	SPI_Init(SPI1, &SPI_InitStructure);  								//根據SPI_IniStruct指定的參數寫入SPIx寄存器

	/* 開啟 SPI1  */
	SPI_Cmd(SPI1, ENABLE); 

	SPI1_ReadWriteByte(0xff);//啟動傳輸	
}  
 
void SPI1_SetSpeed(u8 SpeedSet)
{
	SPI1->CR1&=0XFFC7;				//Fsck=Fcpu/256
	if(SpeedSet==SPI_SPEED_2)		//2分頻
		{
		SPI1->CR1|=0<<3;			//Fsck=Fpclk/2=36Mhz	
		}
	else if(SpeedSet==SPI_SPEED_8)//8分頻 
		{
		SPI1->CR1|=2<<3;			//Fsck=Fpclk/8=9Mhz	
		}
	else if(SpeedSet==SPI_SPEED_16)//16分頻
		{
		SPI1->CR1|=3<<3;			//Fsck=Fpclk/16=4.5Mhz
		}
	else			 	 			//256分頻
		{
		SPI1->CR1|=7<<3; 			//Fsck=Fpclk/256=281.25Khz 低速模式
		}	
	/*開啟 SPI1  */
	SPI_Cmd(SPI1, ENABLE); 
} 

u8 SPI1_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;
	u8 SPIx_Rx = 0;	
		 
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //檢查指定的SPI旗標設置，檢查發送暫存區為空旗標
	{
		retry++;
		if(retry>200) return 0;
	}			  
	/* Send byte through the SPI1 peripheral */
	SPI_I2S_SendData(SPI1, TxData); //通過SPI1
	retry=0;
	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //檢查指定的SPI旗標設置，檢查接收暫存區為非空旗標
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	/* Get the byte read from the SPI bus */
	SPIx_Rx = SPI_I2S_ReceiveData(SPI1); //儲存SPI1接收到的數據
	
	//回傳SPIx接收到的數據
	return SPIx_Rx ;  				    
}










