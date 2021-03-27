/**
  ******************************************************************************
  * 文件名        ：mpu6050.c
	* 创建时间      ：2019.4.10
	* 作者          ：谢胜
	*-----------------------------------------------------------------------------
	* 最近修改时间  ：2019.11.24
	* 修改人        ：刘文熠
  ******************************************************************************
	* 1.本代码基于STM32F427IIH6开发，编程环境位Keil 5，基于FreeRTOS进行开发
	* 2.本代码只适用于Robomaster机器人
	* 3.本代码未经允许禁止私自转发、禁止私自使用
	* 4.本代码包含大量中文注释，请以ANSI编码格式打开
	* 5.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
	* 
	* Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
	******************************************************************************
  */

#include "mpu6050.h"
#include <math.h>
#include "Kalman.h"
#include "soft_i2c.h"

#ifdef SPI_MPU
#include "spi.h"
#endif

#ifdef HARD_I2C
#include "i2c.h"
#endif

#define DEG2RAD		0.017453293f	/* 度转弧度 */
#define RAD2DEG		57.29578f		  /* 弧度转度 */

struct MPU6500_t mpu6050;
struct MPU6500_t mpu6500_pitch;

/**
* @brief 陀螺仪结构体初始化函数
* @param 陀螺仪结构体
* @retval None
*/
//void MPU6500_t_init(struct MPU6500_t *MPU6500)
//{
//	MPU6500->Quaternion.q0 = 1.0f;
//	MPU6500->Quaternion.q1 = 0.0f;
//	MPU6500->Quaternion.q2 = 0.0f;
//	MPU6500->Quaternion.q3 = 0.0f;
//	MPU6500->Integral.Kp = 0.3f;
//	MPU6500->Integral.Ki = 0.01f;
//	MPU6500->Integral.exInt = 0.0f;
//	MPU6500->Integral.eyInt = 0.0f;
//	MPU6500->Integral.ezInt = 0.0f;
//	MPU6500->other.accScale = 1;
//	
//	KalmanInit(&Kalman_yaw);         //卡尔曼结构体参数初始化
//	KalmanInit(&Kalman_pitch);
//	KalmanInit(&Kalman_roll);
//}

//#ifdef SPI_MPU
///**
//* @brief SPI的一些函数，可连续读取多个数据或写入多个数据
//* @param None
//* @retval None
//*/
//static uint8_t tx, rx;
//static uint8_t        tx_buff[14] = { 0xff };
//uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
//{
//	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
//	tx = reg & 0x7F;
//	HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
//	tx = data;
//	HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
//	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
//	return 0;
//}
//uint8_t mpu_read_byte(uint8_t const reg)
//{
//    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
//    tx = reg | 0x80;
//    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
//    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
//    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
//    return rx;
//}
//uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)
//{
//    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
//    tx         = regAddr | 0x80;
//    tx_buff[0] = tx;
//    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
//    HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
//    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
//    return 0;
//}

///**
//* @brief 陀螺仪初始化函数
//* @param SPI通信
//* @retval 通信成功标志，0表示成功
//*/
//int MPU6500_Init_SPI(void)
//{
//	HAL_Delay(100);
//	uint8_t id = mpu_read_byte(MPU6500_WHO_AM_I);

//	uint8_t i                        = 0;
//	uint8_t MPU6500_Init_Data[10][2] = {{ MPU6500_PWR_MGMT_1, 0x80 },     /* Reset Device */ 
//																			{ MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */ 
//																			{ MPU6500_PWR_MGMT_2, 0x00 },     /* Enable Acc & Gyro */ 
//																			{ MPU6500_CONFIG, 0x04 },         /* LPF 41Hz */ 
//																			{ MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */ 
//																			{ MPU6500_ACCEL_CONFIG, 0x10 },   /* +-8G */ 
//																			{ MPU6500_ACCEL_CONFIG_2, 0x02 }, /* enable LowPassFilter  Set Acc LPF */ 
//																			{ MPU6500_USER_CTRL, 0x20 },};    /* Enable AUX */ 
//	for (i = 0; i < 10; i++)
//	{
//		mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
//		HAL_Delay(1);
//	}
//	mpu_write_byte(MPU6500_GYRO_CONFIG, 3 << 3);
//	mpu_write_byte(MPU6500_ACCEL_CONFIG, 2 << 3); 
//	
//	return 0;
//}

///**
//* @brief 获取陀螺仪数据（SPI）
//* @param None
//* @retval 反馈1表示读取成功
//*/
//int GetImuDataSPI(unsigned char *imu_datda_)
//{
//	mpu_read_bytes(MPU6500_ACCEL_XOUT_H, imu_datda_, 14);
//	return 1;
//}
//#endif

//#ifdef Soft_I2c
///**
//* @brief 陀螺仪初始化函数
//* @param 软件IIC对应通道
//* @retval 通信成功标志，0表示成功
//*/
//int MPU6500_Init_Soft(char choice)
//{
//	unsigned char pdata;
//	
//	if(choice == 1)
//	{
//		IIC_GPIO = GPIO_1;
//		SDA = SDA_1;
//		SCL = SCL_1;
//	}
//	else if(choice == 2)
//	{
//		IIC_GPIO = GPIO_2;
//		SDA = SDA_2;
//		SCL = SCL_2;
//	}
//	
//	pdata=0x80; //复位MPU
//	IIC_WriteData(MPU6500_ADDRESS,MPU_PWR_MGMT1_REG,pdata);
//  HAL_Delay(500);  //复位后需要等待一段时间，等待芯片复位完成
//	pdata=0x00;	//唤醒MPU
//	IIC_WriteData(MPU6500_ADDRESS,MPU_PWR_MGMT1_REG,pdata);
//	IIC_ReadData(MPU6500_ADDRESS, MPU_WHO_I_AM, &pdata, 1);
//	#ifdef equipment_mpu6050
//		if(pdata != 0x68) return 1;//mpu6500
//	#endif
//	#ifdef equipment_mpu9250
//		if(pdata != 0x71) return 1;//mpu9250
//	#endif
//	pdata=0x18; //设置量程2000
//	IIC_WriteData(MPU6500_ADDRESS,MPU_GYRO_CFG_REG,pdata);	
//	pdata=0x08;	//设置角速度传感器量程±4g	
//	IIC_WriteData(MPU6500_ADDRESS,MPU_ACCEL_CFG_REG,pdata);	
//	pdata=0;    //陀螺仪采样分频设置
//	IIC_WriteData(MPU6500_ADDRESS,MPU_SAMPLE_RATE_REG,pdata);	
//	pdata=0;	  //关闭所有中断
//	IIC_WriteData(MPU6500_ADDRESS,MPU_INT_EN_REG,pdata);	
//	pdata=0;	  //关闭FIFO
//	IIC_WriteData(MPU6500_ADDRESS,MPU_FIFO_EN_REG,pdata);
//	pdata = 6;	//设置mpu6500的数字低通滤波器
//  IIC_WriteData(MPU6500_ADDRESS,MPU_CFG_REG,pdata);
//	pdata = 0x0C;
//	IIC_WriteData(MPU6500_ADDRESS,MPU_ACCEL_DLPF_DERG,pdata);
//	pdata=0;	  //使能陀螺仪和加速度工作
//	IIC_WriteData(MPU6500_ADDRESS,MPU_PWR_MGMT2_REG,pdata);
//	#ifdef Nine_axis
//	pdata=0X02;	//设置旁路模式，直接读取AK8963磁力计数据
//	IIC_WriteData(MPU6500_ADDRESS, MPU_INTBP_CFG_REG,pdata);
//	HAL_Delay(10);	//需要延时一段时间让磁力计工作
//	pdata = 0x01;
//	IIC_WriteData(AK8963_ADDRESS,AK8963_CNTL1,pdata);
//	HAL_Delay(10);
//	#endif

//	return 0;
//}

///**
//* @brief 获取陀螺仪数据（软件IIC）
//* @param None
//* @retval 反馈1表示读取成功
//*/
//int GetImuDataSoft(uint8_t *imu_datda_)
//{
//	if(IIC_ReadData(MPU6500_ADDRESS, MPU_ACCEL_XOUTH_REG, imu_datda_, 14) == 0)
//		return 1;
//	else
//		return 0;
//}
//#endif

//#ifdef HARD_I2C
///**
//* @brief 陀螺仪初始化函数
//* @param None
//* @retval 通信成功标志，0表示成功
//*/
//int MPU6500_Init_Hard(void)
//{
//	unsigned char pdata;
//	
//	pdata=0x80; //复位MPU
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_PWR_MGMT1_REG, 1, &pdata, 1, 10);
//	HAL_Delay(500);  
//	pdata=0x00;	//唤醒MPU
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_PWR_MGMT1_REG, 1, &pdata, 1, 10);
//	HAL_I2C_Mem_Read(&i2c, MPU6500_ADDRESS, MPU_WHO_I_AM, 1, &pdata, 1, 10);
//	#ifdef equipment_mpu6050
//		if(pdata != 0x68) return 1;//mpu6500
//	#endif
//	#ifdef equipment_mpu9250
//		if(pdata != 0x71) return 1;//mpu9250
//	#endif
//	pdata=0x18; //设置量程2000
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_GYRO_CFG_REG, 1, &pdata, 1, 10); 
//	pdata=0x08;	//设置角速度传感器量程±4g	
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_ACCEL_CFG_REG, 1, &pdata, 1, 10); 
//	pdata=0;    //陀螺仪采样分频设置
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_SAMPLE_RATE_REG, 1, &pdata, 1, 10); 
//	pdata=0;	  //关闭所有中断
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_INT_EN_REG, 1, &pdata, 1, 10); 
//	pdata=0;	  //关闭FIFO
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_FIFO_EN_REG, 1, &pdata, 1, 10); 	
//	pdata = 6;	//设置mpu6500的数字低通滤波器
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_CFG_REG, 1, &pdata, 1, 10);
//	pdata = 0x0C;
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_ACCEL_DLPF_DERG, 1, &pdata, 1, 10);
//	pdata=0;	  //使能陀螺仪和加速度工作
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_PWR_MGMT2_REG, 1, &pdata, 1, 10);
//	#ifdef Nine_axis
//	pdata=0X02;	//设置旁路模式，直接读取AK8963磁力计数据
//	HAL_I2C_Mem_Write(&i2c, MPU9250_ADDRESS, MPU_INTBP_CFG_REG, 1, &pdata, 1, 10); 
//	HAL_Delay(10);	//需要延时一段时间让磁力计工作
//	pdata = 0x01;
//	HAL_I2C_Mem_Write(&i2c, AK8963_ADDRESS, AK8963_CNTL1, 1, &pdata, 1, 10);
//	HAL_Delay(10);
//	#endif 
//	
//	return 0;
//}

///**
//* @brief 获取陀螺仪数据（硬件IIC）
//* @param None
//* @retval 反馈1表示读取成功
//*/
//int GetImuDataHard(uint8_t *imu_datda_)
//{
//	if(HAL_I2C_Mem_Read(&i2c,MPU6500_ADDRESS,MPU_ACCEL_XOUTH_REG,1,imu_datda_,14,10) == HAL_OK)
//		return 1;
//	else
//		return 0;
//}
//#endif

///**
//* @brief 计算方差和平均值
//* @param 陀螺仪结构体里面的数据
//* @retval None
//*/
//static void sensorsCalculateVarianceAndMean(BiasObj* bias, struct Axisf* varOut, struct Axisf* meanOut)
//{
//	uint32_t i;
//	int64_t sum[3] = {0};
//	int64_t sumsq[3] = {0};

//	for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
//	{
//		sum[0] += bias->buffer[i].x;
//		sum[1] += bias->buffer[i].y;
//		sum[2] += bias->buffer[i].z;
//		sumsq[0] += bias->buffer[i].x * bias->buffer[i].x;
//		sumsq[1] += bias->buffer[i].y * bias->buffer[i].y;
//		sumsq[2] += bias->buffer[i].z * bias->buffer[i].z;
//	}

//	varOut->x = (sumsq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
//	varOut->y = (sumsq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
//	varOut->z = (sumsq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

//	meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
//	meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
//	meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
//}

///**
//* @brief 传感器查找配置值
//* @param None
//* @retval 配置成功返回1，否则返回0
//*/
//static int sensorsFindBiasValue(BiasObj *bias)
//{
//	int foundbias = 0;

//	if (bias->isBufferFilled)
//	{
//		struct Axisf mean;
//		struct Axisf variance;
//		sensorsCalculateVarianceAndMean(bias, &variance, &mean);

//		if (variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE && variance.z < GYRO_VARIANCE_BASE)
//		{
//			bias->bias.x = mean.x;
//			bias->bias.y = mean.y;
//			bias->bias.z = mean.z;
//			bias->isBiasValueFound = foundbias= 1;
//		}
//		else
//		{
//			if(bias->initcomplete == 0)
//				bias->initcomplete = 1;
//			bias->isBufferFilled=0;
//		}
//	}
//	return foundbias;
//}

///**
//* @brief 计算陀螺方差
//* @param 陀螺仪结构体里面的数据
//* @retval 
//*/
//int processGyroBias(char reset,int16_t gx, int16_t gy, int16_t gz, struct Axisf *gyroBiasOut, struct Angle_t *angle,
//	               struct Quaternion_t *Quat, BiasObj *gyroBiasRunning, struct Integral_t *Integral, struct Other_t *other)
//{
//	static int count = 0;
//	gyroBiasRunning->buffer[count].x = gx;
//	gyroBiasRunning->buffer[count].y = gy;
//	gyroBiasRunning->buffer[count].z = gz;
//	count++;
//	if(count == 1024)
//	{
//		count = 0;
//		gyroBiasRunning->isBufferFilled = 1;
//	}

//	if(reset == 1)
//	{
//		static int drift_count = 0;
//		drift_count++;
//		if(gx > 150 || gx < -150 || gy > 150 || gy < -150 || gz > 150 || gz < -150)//判断陀螺仪是否在运动
//			drift_count = count = 0;
//		if(drift_count == dirft_time)
//		{
//			angle->last_encoder_yaw = angle->encoder_yaw; //保存陀螺仪yaw的数据
//			angle->yaw_count = 0;                         //对yaw的圈数清零（因为是增量式的）
//			drift_count = 0;                              //重新计时
//			gyroBiasRunning->isBiasValueFound = 0;        //陀螺仪重新计算偏移值
//			count = 0;
//			gyroBiasRunning->isBufferFilled = 0;

//			other->accScaleSum = 0;
//			other->accScale = 1;
//			
//			/* 清空积分误差及四元数误差 */
//			Integral->exInt = Integral->eyInt = Integral->ezInt = 0.0f;
//			Quat->q0 = 1.0f;
//			Quat->q1 = Quat->q2 = Quat->q3 = 0.0f;
//		}
//	}
//	
//	if (!gyroBiasRunning->isBiasValueFound)
//		sensorsFindBiasValue(gyroBiasRunning);

//	gyroBiasOut->x = gyroBiasRunning->bias.x;
//	gyroBiasOut->y = gyroBiasRunning->bias.y;
//	gyroBiasOut->z = gyroBiasRunning->bias.z;

//	return gyroBiasRunning->isBiasValueFound;
//}

///**
//* @brief 根据样本计算重力角速度缩放因子
//* @param None
//* @retval None
//*/
//int processAccScale(int16_t ax, int16_t ay, int16_t az,struct Other_t *other)
//{
//	static int accBiasFound = 0;
//	static uint32_t accScaleSumCount = 0;
//	
//	if (accBiasFound == 0)
//	{
//		other->accScaleSum += sqrtf(powf((float)ax/8192, 2) + powf((float)ay/8192, 2) + powf((float)az/8192, 2));
//		accScaleSumCount++;

//		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
//		{
//			other->accScale = other->accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
//			accBiasFound = 1;
//		}
//	}

//	return accBiasFound;
//}

///**
//* @brief 陀螺仪和加速度计原始数据解析
//* @param None
//* @retval None
//*/
//void imuDataHandle(struct MPU6500_t *MPU6500,char reset)
//{
//	short accx,accy,accz,temp;
//	float faccx,faccy,faccz;
//	short gyrox,gyroy,gyroz;
//	float fgyrox, fgyroy, fgyroz;
//	
//	float gyro_sensitivity = 16.384f;
//	int acc_sensitivity = 8192;
//	
//	accx = (MPU6500->imu_data[0]<<8)|MPU6500->imu_data[1];
//	accy = (MPU6500->imu_data[2]<<8)|MPU6500->imu_data[3];
//	accz = (MPU6500->imu_data[4]<<8)|MPU6500->imu_data[5];
//	temp = (MPU6500->imu_data[6]<<8)|MPU6500->imu_data[7];
//	gyrox = (MPU6500->imu_data[8]<<8)|MPU6500->imu_data[9];
//	gyroy = (MPU6500->imu_data[10]<<8)|MPU6500->imu_data[11];
//	gyroz = (MPU6500->imu_data[12]<<8)|MPU6500->imu_data[13];
//	
//	MPU6500->angle.temp = 21 + temp / 333.87f;
//	
//	MPU6500->other.gyroBiasFound = processGyroBias(reset,gyrox, gyroy, gyroz, &MPU6500->gyroBias, &MPU6500->angle,
//	    &MPU6500->Quaternion, &MPU6500->gyroBiasRunning, &MPU6500->Integral,&MPU6500->other);
//	
//	if(MPU6500->other.gyroBiasFound)
//		processAccScale(accx, accy, accz,&MPU6500->other);	/* 计算accScale */
//	
//	fgyrox = -(float)(gyrox - MPU6500->gyroBias.x)/gyro_sensitivity;
//	fgyroy = (float)(gyroy - MPU6500->gyroBias.y)/gyro_sensitivity;
//	fgyroz = (float)(gyroz - MPU6500->gyroBias.z)/gyro_sensitivity;
//	
//	MPU6500->gyro.x = 0.8f*fgyrox + 0.2f*MPU6500->gyro.x;
//	MPU6500->gyro.y = 0.8f*fgyroy + 0.2f*MPU6500->gyro.y;
//	MPU6500->gyro.z = 0.8f*fgyroz + 0.2f*MPU6500->gyro.z;
//	
//	faccx = -(float)(accx)/acc_sensitivity/MPU6500->other.accScale;
//	faccy = (float)(accy)/acc_sensitivity/MPU6500->other.accScale;
//	faccz = (float)(accz)/acc_sensitivity/MPU6500->other.accScale;
//	
//	MPU6500->acc.x = 0.2f*faccx + 0.8f*MPU6500->acc.x;
//	MPU6500->acc.y = 0.2f*faccy + 0.8f*MPU6500->acc.y;
//	MPU6500->acc.z = 0.2f*faccz + 0.8f*MPU6500->acc.z;
//	
//	#ifdef Nine_axis
//	static short mag_count = 0;
//	uint8_t mag_data[6] = {0};
//	short magx,magy,magz;
//	mag_count++;
//	if(mag_count == 10)	//磁力计不能读取太频繁
//	{
//		#ifdef HARD_I2C
//		HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_HXL, 1, mag_data, 6, HAL_MAX_DELAY);	//???????
//		#endif
//		#ifdef SOFT_I2C
//		IIC_ReadData(AK8963_ADDRESS, AK8963_HXL, mag_data, 6);
//		#endif
//		magx = (mag_data[0]<<8)|mag_data[1];
//		magy = (mag_data[2]<<8)|mag_data[3];
//		magz = (mag_data[4]<<8)|mag_data[5];
//		MPU6500->mag.x = (float)magy/1000.0f;		//磁力计的坐标方位不同
//		MPU6500->mag.y = (float)magx/1000.0f;
//		MPU6500->mag.z = -(float)magz/1000.0f;
//		#ifdef HARD_I2C
//		uint8_t pdata = 1;
//		HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL1, 1, &pdata, 1, HAL_MAX_DELAY);	//??????????????
//		#endif
//		#ifdef SOFT_I2C
//		uint8_t pdata = 1;
//		IIC_WriteData(AK8963_ADDRESS,AK8963_CNTL1,pdata);
//		#endif
//		mag_count = 0;
//	}
//	#endif
//}

///**
//* @brief 根据陀螺仪数据解析出欧拉角函数
//* @param 加速度和角速度结构体
//* @retval None
//*/
//void imuUpdate(struct MPU6500_t *MPU6500, struct Axisf gyro, struct Axisf acc)
//{
//	float q0q0 = MPU6500->Quaternion.q0 * MPU6500->Quaternion.q0;
//	float q1q1 = MPU6500->Quaternion.q1 * MPU6500->Quaternion.q1;
//	float q2q2 = MPU6500->Quaternion.q2 * MPU6500->Quaternion.q2;
//	float q3q3 = MPU6500->Quaternion.q3 * MPU6500->Quaternion.q3;

//	float q0q1 = MPU6500->Quaternion.q0 * MPU6500->Quaternion.q1;
//	float q0q2 = MPU6500->Quaternion.q0 * MPU6500->Quaternion.q2;
//	float q0q3 = MPU6500->Quaternion.q0 * MPU6500->Quaternion.q3;
//	float q1q2 = MPU6500->Quaternion.q1 * MPU6500->Quaternion.q2;
//	float q1q3 = MPU6500->Quaternion.q1 * MPU6500->Quaternion.q3;
//	float q2q3 = MPU6500->Quaternion.q2 * MPU6500->Quaternion.q3;
//	
//	float normalise;
//	float ex, ey, ez;
//	float halfT;
//	float vx, vy, vz;
//	
//	MPU6500->other.now_update = HAL_GetTick(); //单位ms（时间间隔）
//	halfT = ((float)MPU6500->other.now_update - (float)MPU6500->other.last_update) / 2000.0f;
//	MPU6500->other.last_update = MPU6500->other.now_update;
//	
//	gyro.x *= DEG2RAD;	/* 度转弧度 */
//	gyro.y *= DEG2RAD;
//	gyro.z *= DEG2RAD;
//	
//	/* 对角速度计数据进行归一化处理 */
//	if(acc.x != 0 || acc.y != 0 || acc.z != 0)
//	{
//		normalise = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
//		acc.x /= normalise;
//		acc.y /= normalise;
//		acc.z /= normalise;
//	}
//	
//	/* 计算加速度计投影到物体坐标上的各个分量 */
//	vx = 2.0f*(q1q3 - q0q2);
//	vy = 2.0f*(q0q1 + q2q3);
//	vz = q0q0 - q1q1 - q2q2 + q3q3;

//	#ifdef Nine_axis
//	float hx,hy,hz,bx,bz,wx,wy,wz;
//	/* 对磁力计数据进行归一化处理 */
//	if(MPU6500->mag.x != 0 || MPU6500->mag.y != 0 || MPU6500->mag.z != 0)
//	{
//		normalise = sqrt(MPU6500->mag.x * MPU6500->mag.x + MPU6500->mag.y * MPU6500->mag.y + MPU6500->mag.z * MPU6500->mag.z);
//		MPU6500->mag.x /= normalise;
//		MPU6500->mag.y /= normalise;
//		MPU6500->mag.z /= normalise;
//	}
//	/* 计算磁力计投影到物体坐标上的各个分量 */
//	hx = 2.0f*MPU6500->mag.x*(0.5f - q2q2 - q3q3) + 2.0f*MPU6500->mag.y*(q1q2 - q0q3) + 2.0f*MPU6500->mag.z*(q1q3 + q0q2);
//	hy = 2.0f*MPU6500->mag.x*(q1q2 + q0q3) + 2.0f*MPU6500->mag.y*(0.5f - q1q1 - q3q3) + 2.0f*MPU6500->mag.z*(q2q3 - q0q1);
//	hz = 2.0f*MPU6500->mag.x*(q1q3 - q0q2) + 2.0f*MPU6500->mag.y*(q2q3 + q0q1) + 2.0f*MPU6500->mag.z*(0.5f - q1q1 - q2q2);         
//	bx = sqrt((hx*hx) + (hy*hy));
//	bz = hz;
//	/* 处理后的磁力计新分量 */
//	wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
//	wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
//	wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2); 
//	/* 叉积误差累计，用以修正陀螺仪数据 */
//	ex = (MPU6500->acc.y * vz - MPU6500->acc.z * vy) + (MPU6500->mag.y * wz - MPU6500->mag.z * wy);
//	ey = (MPU6500->acc.z * vx - MPU6500->acc.x * vz) + (MPU6500->mag.z * wx - MPU6500->mag.x * wz);
//	ez = (MPU6500->acc.x * vy - MPU6500->acc.y * vx) + (MPU6500->mag.x * wy - MPU6500->mag.y * wx);
//	#else
//	/* 叉积误差累计，用以修正陀螺仪数据 */
//	ex = (acc.y*vz - acc.z*vy);
//	ey = (acc.z*vx - acc.x*vz);
//	ez = (acc.x*vy - acc.y*vx);
//	#endif
//	/* 互补滤波 PI */
//	MPU6500->Integral.exInt += ex * MPU6500->Integral.Ki * halfT;
//	MPU6500->Integral.eyInt += ey * MPU6500->Integral.Ki * halfT;	
//	MPU6500->Integral.ezInt += ez * MPU6500->Integral.Ki * halfT;
//	
//	static int i_do_not = 0;
//	i_do_not++;
//	if(i_do_not == 1)
//	{	
//		gyro.x += MPU6500->Integral.Kp*ex + MPU6500->Integral.exInt;
//		gyro.y += MPU6500->Integral.Kp*ey + MPU6500->Integral.eyInt;
//		gyro.z += MPU6500->Integral.Kp*ez + MPU6500->Integral.ezInt;
//		i_do_not = 0;
//	}
//	
//	/* 使用一阶龙格库塔更新四元数 */
//	float qa,qb,qc;
//	qa = MPU6500->Quaternion.q0;
//	qb = MPU6500->Quaternion.q1;
//	qc = MPU6500->Quaternion.q2;
//	MPU6500->Quaternion.q0 += (-qb * gyro.x - qc * gyro.y - MPU6500->Quaternion.q3 * gyro.z) * halfT;
//	MPU6500->Quaternion.q1 += ( qa * gyro.x + qc * gyro.z - MPU6500->Quaternion.q3 * gyro.y) * halfT;
//	MPU6500->Quaternion.q2 += ( qa * gyro.y - qb * gyro.z + MPU6500->Quaternion.q3 * gyro.x) * halfT;
//	MPU6500->Quaternion.q3 += ( qa * gyro.z + qb * gyro.y - qc * gyro.x) * halfT;
//	
//	/* 对四元数进行归一化处理 */
//	normalise = sqrt(MPU6500->Quaternion.q0 * MPU6500->Quaternion.q0 + MPU6500->Quaternion.q1 * MPU6500->Quaternion.q1 
//				+ MPU6500->Quaternion.q2 * MPU6500->Quaternion.q2 + MPU6500->Quaternion.q3 * MPU6500->Quaternion.q3);
//	MPU6500->Quaternion.q0 /= normalise;
//	MPU6500->Quaternion.q1 /= normalise;
//	MPU6500->Quaternion.q2 /= normalise;
//	MPU6500->Quaternion.q3 /= normalise;
//	
//	/* 由四元数求解欧拉角 */
//	MPU6500->attitude.y = -asinf(-2*MPU6500->Quaternion.q1*MPU6500->Quaternion.q3 + 2*MPU6500->Quaternion.q0*MPU6500->Quaternion.q2) * RAD2DEG;	//pitch
//	MPU6500->attitude.x = atan2f(2*MPU6500->Quaternion.q2*MPU6500->Quaternion.q3 + 2*MPU6500->Quaternion.q0*MPU6500->Quaternion.q1,
//												-2*MPU6500->Quaternion.q1*MPU6500->Quaternion.q1 - 2*MPU6500->Quaternion.q2*MPU6500->Quaternion.q2 + 1) * RAD2DEG;	//roll
//	MPU6500->attitude.z = atan2f(2*MPU6500->Quaternion.q1*MPU6500->Quaternion.q2 + 2*MPU6500->Quaternion.q0*MPU6500->Quaternion.q3,
//												-2*MPU6500->Quaternion.q2*MPU6500->Quaternion.q2 - 2*MPU6500->Quaternion.q3*MPU6500->Quaternion.q3 + 1) * RAD2DEG;	//yaw
//	
//	MPU6500->attitude.x = Kalman_Filter(MPU6500->attitude.x,-MPU6500->gyro.x,&Kalman_pitch);//卡尔曼滤波
//	MPU6500->attitude.y = Kalman_Filter(MPU6500->attitude.y,MPU6500->gyro.y,&Kalman_roll);
////	if(MPU6500->gyroBiasRunning.initcomplete == 1)
////	{
////		MPU6500->gyroBiasRunning.initcomplete = 2;
////		MPU6500->attitude.yaw_offset = MPU6500->attitude.z;
////	}
//	
//	/* 将欧拉角的范围360扩大到8192 */
//	MPU6500->angle.last_yaw = MPU6500->angle.yaw;//yaw轴没必要加卡尔曼
//	MPU6500->angle.yaw = (MPU6500->attitude.z-MPU6500->attitude.yaw_offset) * 22.7556f;
//	if(MPU6500->angle.yaw - MPU6500->angle.last_yaw > 4096)
//		MPU6500->angle.yaw_count--;
//	else if(MPU6500->angle.yaw - MPU6500->angle.last_yaw < -4096)
//		MPU6500->angle.yaw_count++;
//	MPU6500->angle.encoder_yaw = MPU6500->angle.last_encoder_yaw + MPU6500->angle.yaw + MPU6500->angle.yaw_count * 8192;
//	MPU6500->attitude.yaw = MPU6500->attitude.z - MPU6500->attitude.yaw_offset + MPU6500->angle.yaw_count * 360.0f;
//	
//	MPU6500->angle.last_pitch = MPU6500->angle.pitch;
//	MPU6500->angle.pitch = MPU6500->attitude.x * 22.7556f;
//	if(MPU6500->angle.pitch - MPU6500->angle.last_pitch > 4096)
//		MPU6500->angle.pitch_count--;
//	else if(MPU6500->angle.pitch - MPU6500->angle.last_pitch < -4096)
//		MPU6500->angle.pitch_count++;
//	MPU6500->angle.encoder_pitch = MPU6500->angle.pitch + MPU6500->angle.pitch_count * 8192;
//	
//	MPU6500->angle.last_roll = MPU6500->angle.roll;
//	MPU6500->angle.roll = MPU6500->attitude.y * 22.7556f;
//	if(MPU6500->angle.roll - MPU6500->angle.last_roll > 4096)
//		MPU6500->angle.roll_count--;
//	else if(MPU6500->angle.roll - MPU6500->angle.last_roll < -4096)
//		MPU6500->angle.roll_count++;
//	MPU6500->angle.encoder_roll = MPU6500->angle.roll + MPU6500->angle.roll_count * 8192;
//}
