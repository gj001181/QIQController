#include "24cxx.h" 

static void delay_ms(uint16_t ms)
{
	uint16_t i =0;
	while(ms--)
	{
		for(i=0;i<8000;i++);
	}
}


void AT24CXX_Init(void)
{
	I2C_Init();
}

uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr)
{				  
	uint8_t temp=0;		  	    																 
    I2C_Start();  
	if(EE_TYPE>AT24C16)
	{
		I2C_Send_Byte(0XA0);	   
		I2C_Wait_Ack();
		I2C_Send_Byte(ReadAddr>>8);
		I2C_Wait_Ack();		 
	}else I2C_Send_Byte(0XA0+((ReadAddr/256)<<1));    	 

	I2C_Wait_Ack(); 
    I2C_Send_Byte(ReadAddr%256);   
	I2C_Wait_Ack();	    
	I2C_Start();  	 	   
	I2C_Send_Byte(0XA1);           			   
	I2C_Wait_Ack();	 
    temp=I2C_Read_Byte();	
    I2C_NAck();	   
    I2C_Stop();	    
	return temp;
}


void AT24CXX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite)
{				   	  	    																 
    I2C_Start();  
	if(EE_TYPE>AT24C16)
	{
		I2C_Send_Byte(0XA0);	    
		I2C_Wait_Ack();
		I2C_Send_Byte(WriteAddr>>8);
 	}else
	{
		I2C_Send_Byte(0XA0+((WriteAddr/256)<<1));   
	}	 
	I2C_Wait_Ack();	   
    I2C_Send_Byte(WriteAddr%256);  
	I2C_Wait_Ack(); 	 										  		   
	I2C_Send_Byte(DataToWrite);    							   
	I2C_Wait_Ack();  		    	   
    I2C_Stop();
	delay_ms(10);	 
}

void AT24CXX_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len)
{  	
	uint8_t t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

uint32_t AT24CXX_ReadLenByte(uint16_t ReadAddr,uint8_t Len)
{  	
	uint8_t t;
	uint32_t temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}

//check if value of position 255 is 0xFF
//return: 0=success, 1=failed 
uint8_t AT24CXX_Check(void)
{
	uint8_t temp;
	temp=AT24CXX_ReadOneByte(255);		   
	if(temp==0X55)return 0;		   
	else
	{
		AT24CXX_WriteOneByte(255,0X55);
	    temp=AT24CXX_ReadOneByte(255);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

void AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  

void AT24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}
 











