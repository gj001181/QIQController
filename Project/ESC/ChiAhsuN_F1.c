#include "ChiAhsuN_F1.h"

uint8_t commanddata[8],transdata[8];
uint32_t msgid;
uint8_t command;

uint8_t engcommand(uint8_t num)
{
	Can_Receive_Msg(commanddata,0x000001AA);
	//Read Engine1 Command
	if(num==1)
	{
		//Trottle Cut
		if(commanddata[1]==0)
		{
			command=0;
			commanddata[1]=1;
		}
		//MCU mode
		else if(commanddata[1]==1)
		{
			command=1;
		}
		//RC mode
		else if(commanddata[1]==3)
		{
			command=3;
		}
		//Control RPM mode
		else
		{
			command=2;
		}
	}
	//Read Engine2 Command
	if(num==2)
	{
		//Trottle Cut
		if(commanddata[2]==0)
		{
			command=0;
			commanddata[2]=1;
		}
		//MCU mode
		else if(commanddata[2]==1)
		{
			command=1;
		}
		//RC mode
		else if(commanddata[2]==3)
		{
			command=3;
		}
		//Control RPM mode
		else
		{
			command=2;
		}
	}
	//Read Engine3 Command
	if(num==3)
	{
		//Trottle Cut
		if(commanddata[3]==0)
		{
			command=0;
			commanddata[3]=1;
		}
		//MCU mode
		else if(commanddata[3]==1)
		{
			command=1;
		}
		//RC mode
		else if(commanddata[3]==3)
		{
			command=3;
		}
		//Control RPM mode
		else
		{
			command=2;
		}
	}
	//Read Engine4 Command
	if(num==4)
	{
		//Trottle Cut
		if(commanddata[4]==0)
		{
			command=0;
			commanddata[4]=1;
		}
		//MCU mode
		else if(commanddata[4]==1)
		{
			command=1;
		}
		//RC mode
		else if(commanddata[4]==3)
		{
			command=3;
		}
		//Control RPM mode
		else
		{
			command=2;
		}
	}
	return command;
}

void engtransdata(uint16_t rpm,uint16_t servopwm,float servoV,uint8_t servoT,uint16_t throPa,uint8_t statusnum)
{
	transdata[0]=rpm/256;
	transdata[1]=rpm%256;
	transdata[2]=servopwm/256;
	transdata[3]=servopwm%256;
	transdata[4]=servoV;
	transdata[5]=statusnum;
	transdata[6]=throPa/256;
	transdata[7]=throPa%256;

	if(statusnum==1)
	{
		msgid=0x020100AA;
	}
	if(statusnum==2)
	{
		msgid=0x020200AA;
	}
	if(statusnum==3)
	{
		msgid=0x020300AA;
	}
	if(statusnum==4)
	{
		msgid=0x020400AA;
	}

	Can_Send_Msg(transdata,msgid);
}