#include "i2c.h"
/**************************************************************
//I2C.c
void I2C_Init(void);				  
uint8_t I2C_Start(void);				   
void I2C_Stop(void);				   
uint8_t I2C_Wait_Ack(void);			   
void I2C_Ack(void);				       
void I2C_NAck(void);				   
void I2C_Send_Byte(uint8_t txd);		       
uint8_t   I2C_Read_Byte(void);              
**************************************************************/
static void DELAY_uS(uint16_t us)
{
	uint8_t i =0;
	while(us--)
	{
		for(i=0;i<8;i++);
	}
}

void I2C_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	//SCL pin Clock Enable 
	RCC_APB2PeriphClockCmd(I2C_SCL_CLK, ENABLE );
	//SDA pin Clock Enable 
	RCC_APB2PeriphClockCmd(I2C_SDA_CLK, ENABLE );	
	//SCL					 		   
	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD ;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C_SCL_PORT, &GPIO_InitStructure);
	//SDA
	GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD ;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C_SDA_PORT, &GPIO_InitStructure);

	Set_I2C_SCL;
	Set_I2C_SDA;
	Clr_I2C_SDA;
}

//I2C Start
uint8_t I2C_Start(void)
{
	Set_I2C_SDA;
	Set_I2C_SCL;
	DELAY_uS(I2C_Delay);
	if(!READ_SDA)return 0;	
 	Clr_I2C_SDA;  
	DELAY_uS(I2C_Delay);
	if(READ_SDA) return 0;	
	Clr_I2C_SCL
	DELAY_uS(I2C_Delay);
	return 1;
}	  
//I2C Stop
void I2C_Stop(void)
{
	Clr_I2C_SCL; 	DELAY_uS(I2C_Delay);
	Clr_I2C_SDA;    DELAY_uS(I2C_Delay);//STOP:when CLK is high DATA change form low to high
	Set_I2C_SCL;  	DELAY_uS(I2C_Delay);
	Set_I2C_SDA;//Sending I2C end bit							   	
}
//waiting for ACK
//return value¡G0¡Afailed
//              1¡Aok
uint8_t I2C_Wait_Ack(void)
{
	Clr_I2C_SCL;	DELAY_uS(I2C_Delay);
	Set_I2C_SDA;	DELAY_uS(I2C_Delay);
	Set_I2C_SCL;	DELAY_uS(I2C_Delay);
 if(READ_SDA)
 {
    Clr_I2C_SCL;	DELAY_uS(I2C_Delay); 
      return 0;
 }
	Clr_I2C_SCL;	DELAY_uS(I2C_Delay);
	return 1; 
} 

void I2C_Ack(void)
{
	Clr_I2C_SCL;	DELAY_uS(I2C_Delay);
	Clr_I2C_SDA;	DELAY_uS(I2C_Delay);
	Set_I2C_SCL;	DELAY_uS(I2C_Delay);
	Clr_I2C_SCL;	DELAY_uS(I2C_Delay);
}
	    
void I2C_NAck(void)
{
	Clr_I2C_SCL;	DELAY_uS(I2C_Delay);
	Set_I2C_SDA;	DELAY_uS(I2C_Delay);
	Set_I2C_SCL;	DELAY_uS(I2C_Delay);
	Clr_I2C_SCL;	DELAY_uS(I2C_Delay);

}					 				     
			  
void I2C_Send_Byte(uint8_t txd)
{                        
    uint8_t i=8;
    while(i--)
    {
       Clr_I2C_SCL;
       DELAY_uS(I2C_Delay);
      if(txd&0x80)
        {Set_I2C_SDA;}  
      else 
        {Clr_I2C_SDA;}   
        txd<<=1;
       DELAY_uS(I2C_Delay);
		Set_I2C_SCL;
       DELAY_uS(I2C_Delay);
    }
    Clr_I2C_SCL;

} 	    
uint8_t I2C_Read_Byte(void)
{
//
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    Set_I2C_SDA;				
    while(i--)
    {
      ReceiveByte<<=1;      
      Clr_I2C_SCL;
      DELAY_uS(I2C_Delay);
	  Set_I2C_SCL;
      DELAY_uS(I2C_Delay);
      if(READ_SDA)
      {
        ReceiveByte|=0x01;
      }
    }
    Clr_I2C_SCL;
    return ReceiveByte;
}


