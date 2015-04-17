#include "ChiAhsuN_F1.h"

void check_ground_command(uint8_t engine_ID)
{
	CanRxMsg RxMessage;
	uint32_t TX_id;
	uint32_t RX_id;
	uint32_t Msg_type_id;
	uint32_t Eng_num_id;
	
	//Check CAN have Data
	if(CAN_MessagePending(CAN1,CAN_FIFO0)==0)
	{
    	//NO Data
	}
	else
	{
		//Read CAN Data
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

		//Convert Msg ID
		TX_id=(RxMessage.ExtId & 0xFF000000)>>24;
		RX_id=(RxMessage.ExtId & 0x00FF0000)>>16;
		Msg_type_id=(RxMessage.ExtId & 0x0000FF00)>>8;
		Eng_num_id=(RxMessage.ExtId & 0x000000FF);

		//Check Msg_TX is from ground station
		if(TX_id==CAN_ID_GROUND_STATION)
		{
			//Check Msg_RX is for engine
			if(RX_id==CAN_ID_ENGINE)
			{
				//Check Eng_ID is true
				if(Eng_num_id==engine_ID)
				{
					enginedata_array[Eng_num_id][Msg_type_id]=(uint32_t)(RxMessage.Data[0])<<8|(uint32_t)(RxMessage.Data[1]);
					enginedata_flag[engine_ID][Msg_type_id]=1;
				}
			}
		}
	}
}

void enginedata_flag_reset(uint8_t engine_ID,uint8_t msg_type)
{
	enginedata_flag[engine_ID][msg_type]=0;
}

uint8_t engcommand(uint8_t num)
{
	uint8_t commanddata[8];
	uint8_t command=4;

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
		//Control RPM mode
		else if(commanddata[1]==2)
		{
			command=2;
		}
		//RC mode
		else if(commanddata[1]==3)
		{
			command=3;
		}
	}
	//Read Engine2 Command
	else if(num==2)
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
		//Control RPM mode
		else if(commanddata[2]==2)
		{
			command=2;
		}
		//RC mode
		else if(commanddata[2]==3)
		{
			command=3;
		}
	}
	//Read Engine3 Command
	else if(num==3)
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
		//Control RPM mode
		else if(commanddata[3]==2)
		{
			command=2;
		}
		//RC mode
		else if(commanddata[3]==3)
		{
			command=3;
		}
	}
	//Read Engine4 Command
	else if(num==4)
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
		//Control RPM mode
		else if(commanddata[4]==2)
		{
			command=2;
		}
		//RC mode
		else if(commanddata[4]==3)
		{
			command=3;
		}
	}
	return command;
}


void engine_transdata(uint8_t engine_ID,uint32_t rpm,uint32_t servo_pwm,uint32_t servo_vol,uint32_t Temperature,uint32_t throttle_precent)
{
	uint8_t transdata[8];
	uint32_t msgid;

	msgid=(CAN_ID_ENGINE<<24)|(CAN_ID_ENGINE_CONTROLLER<<16)|(CAN_ID_ENGINE_TRANSDATA<<8)|engine_ID;

	transdata[0]=rpm/256;
	transdata[1]=rpm%256;
	transdata[2]=servo_pwm/256;
	transdata[3]=servo_pwm%256;
	transdata[4]=servo_vol;
	transdata[5]=Temperature;
	transdata[6]=throttle_precent/256;
	transdata[7]=throttle_precent%256;

	Can_Send_Msg(transdata,msgid);
}

void engine_trans_set_point(uint8_t engine_ID,uint32_t high_point,uint32_t low_point,uint32_t end_point,uint32_t idle_point)
{
	uint8_t transdata[8];
	uint32_t msgid;

	msgid=(CAN_ID_ENGINE<<24)|(CAN_ID_ENGINE_CONTROLLER<<16)|(CAN_ID_ENGINE_TRANS_SET_POINT<<8)|engine_ID;

	transdata[0]=high_point/256;
	transdata[1]=high_point%256;
	transdata[2]=low_point/256;
	transdata[3]=low_point%256;
	transdata[4]=end_point/256;
	transdata[5]=end_point%256;
	transdata[6]=idle_point/256;
	transdata[7]=idle_point%256;

	Can_Send_Msg(transdata,msgid);
}

void engine_trans_parameter(uint8_t engine_ID,uint32_t kp,uint32_t kd,uint32_t ax,uint32_t b)
{
	uint8_t transdata[8];
	uint32_t msgid;

	msgid=(CAN_ID_ENGINE<<24)|(CAN_ID_ENGINE_CONTROLLER<<16)|(CAN_ID_ENGINE_TRANS_PARAMETER<<8)|engine_ID;

	transdata[0]=kp/256;
	transdata[1]=kp%256;
	transdata[2]=kd/256;
	transdata[3]=kd%256;
	transdata[4]=ax/256;
	transdata[5]=ax%256;
	transdata[6]=b/256;
	transdata[7]=b%256;

	Can_Send_Msg(transdata,msgid);
}
