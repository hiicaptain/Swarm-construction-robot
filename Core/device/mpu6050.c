/**
  ******************************************************************************
  * �ļ���        ��mpu6050.c
	* ����ʱ��      ��2019.4.10
	* ����          ��лʤ
	*-----------------------------------------------------------------------------
	* ����޸�ʱ��  ��2019.11.24
	* �޸���        ��������
  ******************************************************************************
	* 1.���������STM32F427IIH6��������̻���λKeil 5������FreeRTOS���п���
	* 2.������ֻ������Robomaster������
	* 3.������δ�������ֹ˽��ת������ֹ˽��ʹ��
	* 4.�����������������ע�ͣ�����ANSI�����ʽ��
	* 5.���������ս���Ȩ���������ҵ��ѧ�����ڣ��Ϲ���ӥս��Critical HIT����
	* 
	* Copyright (c) ��������ҵ��ѧ�����ڣ��Ϲ���ӥս��Critical HIT ��Ȩ����
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

#define DEG2RAD		0.017453293f	/* ��ת���� */
#define RAD2DEG		57.29578f		  /* ����ת�� */

struct MPU6500_t mpu6050;
struct MPU6500_t mpu6500_pitch;

/**
* @brief �����ǽṹ���ʼ������
* @param �����ǽṹ��
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
//	KalmanInit(&Kalman_yaw);         //�������ṹ�������ʼ��
//	KalmanInit(&Kalman_pitch);
//	KalmanInit(&Kalman_roll);
//}

//#ifdef SPI_MPU
///**
//* @brief SPI��һЩ��������������ȡ������ݻ�д��������
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
//* @brief �����ǳ�ʼ������
//* @param SPIͨ��
//* @retval ͨ�ųɹ���־��0��ʾ�ɹ�
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
//* @brief ��ȡ���������ݣ�SPI��
//* @param None
//* @retval ����1��ʾ��ȡ�ɹ�
//*/
//int GetImuDataSPI(unsigned char *imu_datda_)
//{
//	mpu_read_bytes(MPU6500_ACCEL_XOUT_H, imu_datda_, 14);
//	return 1;
//}
//#endif

//#ifdef Soft_I2c
///**
//* @brief �����ǳ�ʼ������
//* @param ���IIC��Ӧͨ��
//* @retval ͨ�ųɹ���־��0��ʾ�ɹ�
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
//	pdata=0x80; //��λMPU
//	IIC_WriteData(MPU6500_ADDRESS,MPU_PWR_MGMT1_REG,pdata);
//  HAL_Delay(500);  //��λ����Ҫ�ȴ�һ��ʱ�䣬�ȴ�оƬ��λ���
//	pdata=0x00;	//����MPU
//	IIC_WriteData(MPU6500_ADDRESS,MPU_PWR_MGMT1_REG,pdata);
//	IIC_ReadData(MPU6500_ADDRESS, MPU_WHO_I_AM, &pdata, 1);
//	#ifdef equipment_mpu6050
//		if(pdata != 0x68) return 1;//mpu6500
//	#endif
//	#ifdef equipment_mpu9250
//		if(pdata != 0x71) return 1;//mpu9250
//	#endif
//	pdata=0x18; //��������2000
//	IIC_WriteData(MPU6500_ADDRESS,MPU_GYRO_CFG_REG,pdata);	
//	pdata=0x08;	//���ý��ٶȴ��������̡�4g	
//	IIC_WriteData(MPU6500_ADDRESS,MPU_ACCEL_CFG_REG,pdata);	
//	pdata=0;    //�����ǲ�����Ƶ����
//	IIC_WriteData(MPU6500_ADDRESS,MPU_SAMPLE_RATE_REG,pdata);	
//	pdata=0;	  //�ر������ж�
//	IIC_WriteData(MPU6500_ADDRESS,MPU_INT_EN_REG,pdata);	
//	pdata=0;	  //�ر�FIFO
//	IIC_WriteData(MPU6500_ADDRESS,MPU_FIFO_EN_REG,pdata);
//	pdata = 6;	//����mpu6500�����ֵ�ͨ�˲���
//  IIC_WriteData(MPU6500_ADDRESS,MPU_CFG_REG,pdata);
//	pdata = 0x0C;
//	IIC_WriteData(MPU6500_ADDRESS,MPU_ACCEL_DLPF_DERG,pdata);
//	pdata=0;	  //ʹ�������Ǻͼ��ٶȹ���
//	IIC_WriteData(MPU6500_ADDRESS,MPU_PWR_MGMT2_REG,pdata);
//	#ifdef Nine_axis
//	pdata=0X02;	//������·ģʽ��ֱ�Ӷ�ȡAK8963����������
//	IIC_WriteData(MPU6500_ADDRESS, MPU_INTBP_CFG_REG,pdata);
//	HAL_Delay(10);	//��Ҫ��ʱһ��ʱ���ô����ƹ���
//	pdata = 0x01;
//	IIC_WriteData(AK8963_ADDRESS,AK8963_CNTL1,pdata);
//	HAL_Delay(10);
//	#endif

//	return 0;
//}

///**
//* @brief ��ȡ���������ݣ����IIC��
//* @param None
//* @retval ����1��ʾ��ȡ�ɹ�
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
//* @brief �����ǳ�ʼ������
//* @param None
//* @retval ͨ�ųɹ���־��0��ʾ�ɹ�
//*/
//int MPU6500_Init_Hard(void)
//{
//	unsigned char pdata;
//	
//	pdata=0x80; //��λMPU
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_PWR_MGMT1_REG, 1, &pdata, 1, 10);
//	HAL_Delay(500);  
//	pdata=0x00;	//����MPU
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_PWR_MGMT1_REG, 1, &pdata, 1, 10);
//	HAL_I2C_Mem_Read(&i2c, MPU6500_ADDRESS, MPU_WHO_I_AM, 1, &pdata, 1, 10);
//	#ifdef equipment_mpu6050
//		if(pdata != 0x68) return 1;//mpu6500
//	#endif
//	#ifdef equipment_mpu9250
//		if(pdata != 0x71) return 1;//mpu9250
//	#endif
//	pdata=0x18; //��������2000
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_GYRO_CFG_REG, 1, &pdata, 1, 10); 
//	pdata=0x08;	//���ý��ٶȴ��������̡�4g	
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_ACCEL_CFG_REG, 1, &pdata, 1, 10); 
//	pdata=0;    //�����ǲ�����Ƶ����
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_SAMPLE_RATE_REG, 1, &pdata, 1, 10); 
//	pdata=0;	  //�ر������ж�
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_INT_EN_REG, 1, &pdata, 1, 10); 
//	pdata=0;	  //�ر�FIFO
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_FIFO_EN_REG, 1, &pdata, 1, 10); 	
//	pdata = 6;	//����mpu6500�����ֵ�ͨ�˲���
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_CFG_REG, 1, &pdata, 1, 10);
//	pdata = 0x0C;
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_ACCEL_DLPF_DERG, 1, &pdata, 1, 10);
//	pdata=0;	  //ʹ�������Ǻͼ��ٶȹ���
//	HAL_I2C_Mem_Write(&i2c, MPU6500_ADDRESS, MPU_PWR_MGMT2_REG, 1, &pdata, 1, 10);
//	#ifdef Nine_axis
//	pdata=0X02;	//������·ģʽ��ֱ�Ӷ�ȡAK8963����������
//	HAL_I2C_Mem_Write(&i2c, MPU9250_ADDRESS, MPU_INTBP_CFG_REG, 1, &pdata, 1, 10); 
//	HAL_Delay(10);	//��Ҫ��ʱһ��ʱ���ô����ƹ���
//	pdata = 0x01;
//	HAL_I2C_Mem_Write(&i2c, AK8963_ADDRESS, AK8963_CNTL1, 1, &pdata, 1, 10);
//	HAL_Delay(10);
//	#endif 
//	
//	return 0;
//}

///**
//* @brief ��ȡ���������ݣ�Ӳ��IIC��
//* @param None
//* @retval ����1��ʾ��ȡ�ɹ�
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
//* @brief ���㷽���ƽ��ֵ
//* @param �����ǽṹ�����������
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
//* @brief ��������������ֵ
//* @param None
//* @retval ���óɹ�����1�����򷵻�0
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
//* @brief �������ݷ���
//* @param �����ǽṹ�����������
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
//		if(gx > 150 || gx < -150 || gy > 150 || gy < -150 || gz > 150 || gz < -150)//�ж��������Ƿ����˶�
//			drift_count = count = 0;
//		if(drift_count == dirft_time)
//		{
//			angle->last_encoder_yaw = angle->encoder_yaw; //����������yaw������
//			angle->yaw_count = 0;                         //��yaw��Ȧ�����㣨��Ϊ������ʽ�ģ�
//			drift_count = 0;                              //���¼�ʱ
//			gyroBiasRunning->isBiasValueFound = 0;        //���������¼���ƫ��ֵ
//			count = 0;
//			gyroBiasRunning->isBufferFilled = 0;

//			other->accScaleSum = 0;
//			other->accScale = 1;
//			
//			/* ��ջ�������Ԫ����� */
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
//* @brief �������������������ٶ���������
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
//* @brief �����Ǻͼ��ٶȼ�ԭʼ���ݽ���
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
//		processAccScale(accx, accy, accz,&MPU6500->other);	/* ����accScale */
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
//	if(mag_count == 10)	//�����Ʋ��ܶ�ȡ̫Ƶ��
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
//		MPU6500->mag.x = (float)magy/1000.0f;		//�����Ƶ����귽λ��ͬ
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
//* @brief �������������ݽ�����ŷ���Ǻ���
//* @param ���ٶȺͽ��ٶȽṹ��
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
//	MPU6500->other.now_update = HAL_GetTick(); //��λms��ʱ������
//	halfT = ((float)MPU6500->other.now_update - (float)MPU6500->other.last_update) / 2000.0f;
//	MPU6500->other.last_update = MPU6500->other.now_update;
//	
//	gyro.x *= DEG2RAD;	/* ��ת���� */
//	gyro.y *= DEG2RAD;
//	gyro.z *= DEG2RAD;
//	
//	/* �Խ��ٶȼ����ݽ��й�һ������ */
//	if(acc.x != 0 || acc.y != 0 || acc.z != 0)
//	{
//		normalise = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
//		acc.x /= normalise;
//		acc.y /= normalise;
//		acc.z /= normalise;
//	}
//	
//	/* ������ٶȼ�ͶӰ�����������ϵĸ������� */
//	vx = 2.0f*(q1q3 - q0q2);
//	vy = 2.0f*(q0q1 + q2q3);
//	vz = q0q0 - q1q1 - q2q2 + q3q3;

//	#ifdef Nine_axis
//	float hx,hy,hz,bx,bz,wx,wy,wz;
//	/* �Դ��������ݽ��й�һ������ */
//	if(MPU6500->mag.x != 0 || MPU6500->mag.y != 0 || MPU6500->mag.z != 0)
//	{
//		normalise = sqrt(MPU6500->mag.x * MPU6500->mag.x + MPU6500->mag.y * MPU6500->mag.y + MPU6500->mag.z * MPU6500->mag.z);
//		MPU6500->mag.x /= normalise;
//		MPU6500->mag.y /= normalise;
//		MPU6500->mag.z /= normalise;
//	}
//	/* ���������ͶӰ�����������ϵĸ������� */
//	hx = 2.0f*MPU6500->mag.x*(0.5f - q2q2 - q3q3) + 2.0f*MPU6500->mag.y*(q1q2 - q0q3) + 2.0f*MPU6500->mag.z*(q1q3 + q0q2);
//	hy = 2.0f*MPU6500->mag.x*(q1q2 + q0q3) + 2.0f*MPU6500->mag.y*(0.5f - q1q1 - q3q3) + 2.0f*MPU6500->mag.z*(q2q3 - q0q1);
//	hz = 2.0f*MPU6500->mag.x*(q1q3 - q0q2) + 2.0f*MPU6500->mag.y*(q2q3 + q0q1) + 2.0f*MPU6500->mag.z*(0.5f - q1q1 - q2q2);         
//	bx = sqrt((hx*hx) + (hy*hy));
//	bz = hz;
//	/* �����Ĵ������·��� */
//	wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
//	wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
//	wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2); 
//	/* �������ۼƣ������������������� */
//	ex = (MPU6500->acc.y * vz - MPU6500->acc.z * vy) + (MPU6500->mag.y * wz - MPU6500->mag.z * wy);
//	ey = (MPU6500->acc.z * vx - MPU6500->acc.x * vz) + (MPU6500->mag.z * wx - MPU6500->mag.x * wz);
//	ez = (MPU6500->acc.x * vy - MPU6500->acc.y * vx) + (MPU6500->mag.x * wy - MPU6500->mag.y * wx);
//	#else
//	/* �������ۼƣ������������������� */
//	ex = (acc.y*vz - acc.z*vy);
//	ey = (acc.z*vx - acc.x*vz);
//	ez = (acc.x*vy - acc.y*vx);
//	#endif
//	/* �����˲� PI */
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
//	/* ʹ��һ���������������Ԫ�� */
//	float qa,qb,qc;
//	qa = MPU6500->Quaternion.q0;
//	qb = MPU6500->Quaternion.q1;
//	qc = MPU6500->Quaternion.q2;
//	MPU6500->Quaternion.q0 += (-qb * gyro.x - qc * gyro.y - MPU6500->Quaternion.q3 * gyro.z) * halfT;
//	MPU6500->Quaternion.q1 += ( qa * gyro.x + qc * gyro.z - MPU6500->Quaternion.q3 * gyro.y) * halfT;
//	MPU6500->Quaternion.q2 += ( qa * gyro.y - qb * gyro.z + MPU6500->Quaternion.q3 * gyro.x) * halfT;
//	MPU6500->Quaternion.q3 += ( qa * gyro.z + qb * gyro.y - qc * gyro.x) * halfT;
//	
//	/* ����Ԫ�����й�һ������ */
//	normalise = sqrt(MPU6500->Quaternion.q0 * MPU6500->Quaternion.q0 + MPU6500->Quaternion.q1 * MPU6500->Quaternion.q1 
//				+ MPU6500->Quaternion.q2 * MPU6500->Quaternion.q2 + MPU6500->Quaternion.q3 * MPU6500->Quaternion.q3);
//	MPU6500->Quaternion.q0 /= normalise;
//	MPU6500->Quaternion.q1 /= normalise;
//	MPU6500->Quaternion.q2 /= normalise;
//	MPU6500->Quaternion.q3 /= normalise;
//	
//	/* ����Ԫ�����ŷ���� */
//	MPU6500->attitude.y = -asinf(-2*MPU6500->Quaternion.q1*MPU6500->Quaternion.q3 + 2*MPU6500->Quaternion.q0*MPU6500->Quaternion.q2) * RAD2DEG;	//pitch
//	MPU6500->attitude.x = atan2f(2*MPU6500->Quaternion.q2*MPU6500->Quaternion.q3 + 2*MPU6500->Quaternion.q0*MPU6500->Quaternion.q1,
//												-2*MPU6500->Quaternion.q1*MPU6500->Quaternion.q1 - 2*MPU6500->Quaternion.q2*MPU6500->Quaternion.q2 + 1) * RAD2DEG;	//roll
//	MPU6500->attitude.z = atan2f(2*MPU6500->Quaternion.q1*MPU6500->Quaternion.q2 + 2*MPU6500->Quaternion.q0*MPU6500->Quaternion.q3,
//												-2*MPU6500->Quaternion.q2*MPU6500->Quaternion.q2 - 2*MPU6500->Quaternion.q3*MPU6500->Quaternion.q3 + 1) * RAD2DEG;	//yaw
//	
//	MPU6500->attitude.x = Kalman_Filter(MPU6500->attitude.x,-MPU6500->gyro.x,&Kalman_pitch);//�������˲�
//	MPU6500->attitude.y = Kalman_Filter(MPU6500->attitude.y,MPU6500->gyro.y,&Kalman_roll);
////	if(MPU6500->gyroBiasRunning.initcomplete == 1)
////	{
////		MPU6500->gyroBiasRunning.initcomplete = 2;
////		MPU6500->attitude.yaw_offset = MPU6500->attitude.z;
////	}
//	
//	/* ��ŷ���ǵķ�Χ360����8192 */
//	MPU6500->angle.last_yaw = MPU6500->angle.yaw;//yaw��û��Ҫ�ӿ�����
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
