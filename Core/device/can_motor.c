#include "can_motor.h"
#include "supervise.h"
#include "IMU.h"

struct CAN_Motor can1_motor_5;
struct CAN_Motor can1_motor_6;
struct CAN_Motor can1_motor_7;
uint8_t can_rx_data_buffer[8];

void CanDataReceive(int motor_index);
void CanDataEncoderProcess(struct CAN_Motor *motor);

/*
 * @brief CAN外设过滤器初始化
 * @param can结构体
 * @retval None
 */
HAL_StatusTypeDef CanFilterInit(CAN_HandleTypeDef* hcan)
{
	CAN_FilterTypeDef  sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	if(hcan == &hcan1)
	{
			sFilterConfig.FilterBank = 0;
	}
	if(hcan == &hcan2)
  {
    sFilterConfig.FilterBank = 14;
  }
	if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
	{
			Error_Handler();
	}
	if (HAL_CAN_Start(hcan) != HAL_OK)
	{
			Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
			Error_Handler();
	}
	return HAL_OK;
}

/*
 * @brief CAN通信接收中断回调函数
 * @param CAN序号
 * @retval None
 */
extern encoder_t motor_left_encoder, motor_right_encoder, motor_climb_encoder;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, can_rx_data_buffer) != HAL_OK)
	{
			Error_Handler();            //如果CAN通信数据接收出错，则进入死循环
	}
	if(hcan == &hcan1)
	{
		switch(rx_header.StdId)
		{
			case 0x201:
				MotorMsgUnpack(&motor_left_encoder, can_rx_data_buffer, 0);
				break;
			
			case 0x202:
				MotorMsgUnpack(&motor_right_encoder, can_rx_data_buffer, 0);
				break;
			
			case 0x203:
				MotorMsgUnpack(&motor_climb_encoder, can_rx_data_buffer, 0);
				break;
			
			default:
				break;
		}
	}
}

/*
 * @brief 根据电机信息的ID号进行对应的数据解析
 * @param 电机ID号
 * @retval None
 */
void CanDataReceive(int motor_index)
{
	switch(motor_index)
	{
		case CAN_MOTOR5_ID:
			LostCounterFeed(CAN_MOTOR5_INDEX);
			CanDataEncoderProcess(&can1_motor_5);break;    //电机数据具体解析函数
		case CAN_MOTOR6_ID:
			LostCounterFeed(CAN_MOTOR6_INDEX);
			CanDataEncoderProcess(&can1_motor_6);break;
		case CAN_MOTOR7_ID:
			LostCounterFeed(CAN_MOTOR7_INDEX);
			CanDataEncoderProcess(&can1_motor_7);break;
		default:;
	}
}

void MotorMsgUnpack(encoder_t *encoder, uint8_t *buffer, uint8_t type)
{
	if(type == 0)
	{
		float theta = (uint16_t)(buffer[0] << 8 | buffer[1]) * 2 * PI/8192.0f;
		encoder_process(encoder, theta, 1e-3);
		float omiga = (int16_t)(buffer[2] << 8 | buffer[3]) / 9.55f;
		encoder->omiga = omiga;
	}
}

/*
 * @brief CAN通信电机的反馈数据具体解析函数
 * @param 电机数据结构体
 * @retval None
 */
void CanDataEncoderProcess(struct CAN_Motor *motor)
{
    motor->last_encoder = motor->encoder;
    motor->encoder = can_rx_data_buffer[0]<<8|can_rx_data_buffer[1];
    motor->angular_spd_rpm = can_rx_data_buffer[2]<<8|can_rx_data_buffer[3];
		/* 电机位置数据过零处理，避免出现位置突变的情况 */
    if(motor->encoder - motor->last_encoder > 4096)
    {
        motor->round--;
    }
    else if(motor->encoder - motor->last_encoder < -4096)
    {
        motor->round++;
    }
		motor->angle = (motor->encoder - motor->encoder_offset) / 22.756f;
    motor->encoder_inf = motor->encoder + motor->round * 8192 - motor->encoder_offset;
		motor->angle_inf = motor->encoder_inf / 22.756f;
		motor->angular_spd = motor->angular_spd_rpm / 60.0f * 2.0f * 360.0f;
}

/*
 * @brief ID为1~4的电机信号发送函数
 * @param ID为1~4的各个电机的电流数值
 * @retval None
 */
void CanTransmit_1234(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CAN_TxHeaderTypeDef TxMessage;
    TxMessage.DLC=0x08;
    TxMessage.StdId=0x200;
    TxMessage.IDE=CAN_ID_STD;
    TxMessage.RTR=CAN_RTR_DATA;
    uint8_t TxData[8];
    TxData[0] = (uint8_t)(cm1_iq >> 8);
    TxData[1] = (uint8_t)cm1_iq;
    TxData[2] = (uint8_t)(cm2_iq >> 8);
    TxData[3] = (uint8_t)cm2_iq;
    TxData[4] = (uint8_t)(cm3_iq >> 8);
    TxData[5] = (uint8_t)cm3_iq;
    TxData[6] = (uint8_t)(cm4_iq >> 8);
    TxData[7] = (uint8_t)cm4_iq;
    HAL_CAN_AddTxMessage(hcanx,&TxMessage,TxData,(uint32_t*)CAN_TX_MAILBOX0);
}

void CanTransmit_5678(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CAN_TxHeaderTypeDef TxMessage;
    TxMessage.DLC=0x08;
    TxMessage.StdId=0x1FF;
    TxMessage.IDE=CAN_ID_STD;
    TxMessage.RTR=CAN_RTR_DATA;
    uint8_t TxData[8];
    TxData[0] = (uint8_t)(cm1_iq >> 8);
    TxData[1] = (uint8_t)cm1_iq;
    TxData[2] = (uint8_t)(cm2_iq >> 8);
    TxData[3] = (uint8_t)cm2_iq;
    TxData[4] = (uint8_t)(cm3_iq >> 8);
    TxData[5] = (uint8_t)cm3_iq;
    TxData[6] = (uint8_t)(cm4_iq >> 8);
    TxData[7] = (uint8_t)cm4_iq;
    HAL_CAN_AddTxMessage(hcanx,&TxMessage,TxData,(uint32_t*)CAN_TX_MAILBOX0);
}
