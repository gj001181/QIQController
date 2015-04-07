#ifndef __I2C_H
#define __I2C_H
#include "stm32f10x.h"
#include "stm32f10x_conf.h"

//I2C delay interval
#define I2C_Delay 1

/*I2C pin definition*/
//SCL pin
#define I2C_SCL_PORT              GPIOB
#define I2C_SCL_CLK               RCC_APB2Periph_GPIOB  
#define I2C_SCL_PIN               GPIO_Pin_2

#define Set_I2C_SCL  			  {GPIO_SetBits(I2C_SCL_PORT,I2C_SCL_PIN);}
#define Clr_I2C_SCL  			  {GPIO_ResetBits(I2C_SCL_PORT,I2C_SCL_PIN);} 
//SDA pin
#define I2C_SDA_PORT              GPIOB
#define I2C_SDA_CLK               RCC_APB2Periph_GPIOB  
#define I2C_SDA_PIN               GPIO_Pin_1

	 
#define Set_I2C_SDA  			  {GPIO_SetBits(I2C_SDA_PORT,I2C_SDA_PIN);}
#define Clr_I2C_SDA  			  {GPIO_ResetBits(I2C_SDA_PORT,I2C_SDA_PIN);}

#define READ_SDA    			  (GPIO_ReadInputDataBit(I2C_SDA_PORT, I2C_SDA_PIN))

//***************************************************************
/*Function prototype*/
//I2C.c
void I2C_Init(void);			   
uint8_t I2C_Start(void);			   
void I2C_Stop(void);			   
uint8_t I2C_Wait_Ack(void);		   
void I2C_Ack(void);				  
void I2C_NAck(void);			   
void I2C_Send_Byte(uint8_t txd);		   
uint8_t   I2C_Read_Byte(void);        
//*********************************************************

#endif
