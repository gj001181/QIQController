#include "can.h"

uint8_t engcommand(uint8_t num);

void engine_transdata(uint8_t engine_ID,uint32_t rpm,uint32_t servo_pwm,uint32_t servo_vol,uint32_t Temperature,uint32_t throttle_precent);
void engine_trans_set_point(uint8_t engine_ID,uint32_t high_point,uint32_t low_point,uint32_t end_point,uint32_t idle_point);
void engine_trans_parameter(uint8_t engine_ID,uint32_t kp,uint32_t kd,uint32_t ax,uint32_t b);
void check_ground_command(uint8_t engine_ID);
void enginedata_flag_reset(uint8_t engine_ID,uint8_t msg_type);

//===================CANbus Msg ID Set===========================
//CAN_ID Tx and Rx Type
typedef enum ChiaHsun_CAN_TX_RX
{
	CAN_ID_GROUND_STATION=0,
	CAN_ID_ENGINE,
	CAN_ID_SENSOR,
	CHIAHSUN_CAN_TX_RX_SIZE
}
ChiaHsun_CAN_TX_RX;

//CAN_ID Engine Sub Type
typedef enum ChiaHsun_CAN_ENGINE_SUB
{
	CAN_ID_ENGINE_CONTROLLER=0,
	CHIAHSUN_CAN_ENGINE_SUB_SIZE
}
ChiaHsun_CAN_ENGINE_SUB;

//CAN_ID Engine ID
typedef enum ChiaHsun_CAN_ENGINE_ID
{
	CAN_ID_ENGINE_ALL=0,
	CAN_ID_ENGINE_1,
	CAN_ID_ENGINE_2,
	CAN_ID_ENGINE_3,
	CAN_ID_ENGINE_4,
	CHIAHSUN_CAN_ENGINE_ID_SIZE
}
ChiaHsun_CAN_ENGINE_ID;

//CAN_ID Engine Controller Msg Type
typedef enum ChiaHsun_CAN_ENGINE_CONTROLLER_MSG
{
	//EngineCommand
	CAN_ID_ENGINE_CONTROLLER_MODE=0,
	CAN_ID_MCU_PWM,
	CAN_ID_REQUIRE,
	//Engine Set Point
	CAN_ID_ENGINE_TRANS_SET_POINT,	
	CAN_ID_HIGH_POINT,
	CAN_ID_LOW_POINT,
	CAN_ID_END_POINT,
	CAN_ID_IDLE_POINT,
	//Engine Parameter
	CAN_ID_ENGINE_TRANS_PARAMETER,
	CAN_ID_KP,
	CAN_ID_KI,
	CAN_ID_KD,
	CAN_ID_A1x,
	CAN_ID_B1,
    CAN_ID_A2x,
	CAN_ID_B2,
	//Engine Data
	CAN_ID_ENGINE_TRANSDATA,
	CAN_ID_ENGINE_RPM,
	CAN_ID_ENGINE_SERVO_PWM,
	CAN_ID_ENGINE_SERVO_VOL,
	CAN_ID_ENGINE_TEMPERATURE,
	CAN_ID_ENGINE_THROTTLE_PERCENT,
	//SIZE
	CHIAHSUN_CAN_ENGINE_CONTROLLER_MSG_SIZE
}
ChiaHsun_CAN_ENGINE_CONTROLLER_MSG;

//Set Engine Data Array
uint32_t enginedata_array[CHIAHSUN_CAN_ENGINE_ID_SIZE][CHIAHSUN_CAN_ENGINE_CONTROLLER_MSG_SIZE];
uint32_t enginedata_flag[CHIAHSUN_CAN_ENGINE_ID_SIZE][CHIAHSUN_CAN_ENGINE_CONTROLLER_MSG_SIZE];

//CAN_Data Engine Controller Mode
typedef enum ChiaHsun_CAN_ENGINE_CONTROLLER_MODE
{
	CAN_DATA_THROTTLE_CUT=0,
	CAN_DATA_MCU_MODE,
	CAN_DATA_RPMCTRL_MODE,
	CAN_DATA_RC_MODE
}
ChiaHsun_CAN_ENGINE_CONTROLLER_MODE;
