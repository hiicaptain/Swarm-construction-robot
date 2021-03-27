#ifndef _MPU6050_H_
#define _MPU6050_H_

/** 用户修改区
  * 对你需要的设备取消注释
	*/
#define equipment_mpu6050 //陀螺仪mpu6050模块(GY-521)
//#define equipment_mpu9250 //陀螺仪mpu9250模块

//#define HARD_I2C
//#define Soft_I2c
#define SPI_MPU

#ifdef equipment_mpu9250
//#define Nine_axis
#endif

/** 用户修改区
  * 选择通信方式：硬件IIC，模拟IIC（软件实现）
	* 较常使用模拟IIC
	*/
#define i2c hi2c2         //如果为硬件IIC，还有选择是哪个IIC通道
#define GPIO_1 GPIOE      //若为模拟IIC，注意GPIO分组及管脚
#define SDA_1 GPIO_PIN_7  
#define SCL_1 GPIO_PIN_8
#define GPIO_2 GPIOE      //有两种选择
#define SDA_2 GPIO_PIN_8
#define SCL_2 GPIO_PIN_7
#define MPU_HSPI hspi5    //SPI通信，可选择不同的通道

#define dirft_time 8000   //单位：ms，表示陀螺仪动态调整零漂时间

/** 
  * 调用例程：
  * 初始化：MPU6500_t_init(&mpu6500_yaw);                          //陀螺仪结构体初始化
	*         while(MPU6500_Init_Soft(1)){}                          //初始化陀螺仪，先向陀螺仪寄存器写入参数
	* 数据读取及解析：GetImuDataSoft(mpu6500_yaw.imu_data);          //获取原始数据，分软硬件
	*									imuDataHandle(&mpu6500_yaw,1);                              //原始数据解析
	*									imuUpdate(&mpu6500_yaw, mpu6500_yaw.gyro, mpu6500_yaw.acc); //姿态解析
	*/
	
	

/* 在读取时，注意需要人工将设备地址左移1位（IIC读写为左对齐，第8位要存读写标志位 */
#define MPU6500_ADDRESS 0xD0        //AD0接GND时地址位0x68，接VCC时地址为0x69，注意设备左移一位
#define MPU_PWR_MGMT1_REG		0X6B	  
#define MPU_GYRO_CFG_REG		0X1B	  
#define MPU_ACCEL_CFG_REG		0X1C	  
#define MPU_ACCEL_DLPF_DERG	0x1D	  
#define MPU_SAMPLE_RATE_REG		0X19	
#define MPU_INT_EN_REG			0X38	  
#define MPU_USER_CTRL_REG		0X6A	  
#define MPU_FIFO_EN_REG			0X23	  
#define MPU_INTBP_CFG_REG		0X37	  
#define MPU_DEVICE_ID_REG		0X75	  
#define MPU_PWR_MGMT2_REG		0X6C	  
#define MPU_CFG_REG				0X1A	     

#define MPU_TEMP_OUTH_REG		0X41	  
#define MPU_TEMP_OUTL_REG		0X42	  

#define MPU_ACCEL_XOUTH_REG		0X3B	
#define MPU_ACCEL_XOUTL_REG		0X3C	
#define MPU_ACCEL_YOUTH_REG		0X3D	
#define MPU_ACCEL_YOUTL_REG		0X3E	
#define MPU_ACCEL_ZOUTH_REG		0X3F	
#define MPU_ACCEL_ZOUTL_REG		0X40	

#define MPU_GYRO_XOUTH_REG		0X43	
#define MPU_GYRO_XOUTL_REG		0X44	
#define MPU_GYRO_YOUTH_REG		0X45	
#define MPU_GYRO_YOUTL_REG		0X46	
#define MPU_GYRO_ZOUTH_REG		0X47	
#define MPU_GYRO_ZOUTL_REG		0X48	

#define MPU_WHO_I_AM	0x75	        //陀螺仪ID

#define AK8963_ADDRESS 0x18		     
#define AK8963_CNTL1		0x0A        
#define AK8963_HXL			0x03	     
#define AK8963_HXH			0x04	      
#define AK8963_HYL			0x05       
#define AK8963_HYH			0x06        
#define AK8963_HZL			0x07      	
#define AK8963_HZH			0x08	      
#define AK8963_WHO_I_AM	0x00	      

#define SENSORS_NBR_OF_BIAS_SAMPLES		1024	/* 计算方差的采样样本个数 */
#define GYRO_VARIANCE_BASE				4000	    /* 陀螺仪零漂方差阈值 */
#define SENSORS_ACC_SCALE_SAMPLES  		200		/* 加速度采样个数 */

//mpu Reg -- Map
#define MPU6500_SELF_TEST_XG        (0x00)
#define MPU6500_SELF_TEST_YG        (0x01)
#define MPU6500_SELF_TEST_ZG        (0x02)
#define MPU6500_SELF_TEST_XA        (0x0D)
#define MPU6500_SELF_TEST_YA        (0x0E)
#define MPU6500_SELF_TEST_ZA        (0x0F)
#define MPU6500_XG_OFFSET_H         (0x13)
#define MPU6500_XG_OFFSET_L         (0x14)
#define MPU6500_YG_OFFSET_H         (0x15)
#define MPU6500_YG_OFFSET_L         (0x16)
#define MPU6500_ZG_OFFSET_H         (0x17)
#define MPU6500_ZG_OFFSET_L         (0x18)
#define MPU6500_SMPLRT_DIV          (0x19)
#define MPU6500_CONFIG              (0x1A)
#define MPU6500_GYRO_CONFIG         (0x1B)
#define MPU6500_ACCEL_CONFIG        (0x1C)
#define MPU6500_ACCEL_CONFIG_2      (0x1D)
#define MPU6500_LP_ACCEL_ODR        (0x1E)
#define MPU6500_MOT_THR             (0x1F)
#define MPU6500_FIFO_EN             (0x23)
#define MPU6500_I2C_MST_CTRL        (0x24)
#define MPU6500_I2C_SLV0_ADDR       (0x25)
#define MPU6500_I2C_SLV0_REG        (0x26)
#define MPU6500_I2C_SLV0_CTRL       (0x27)
#define MPU6500_I2C_SLV1_ADDR       (0x28)
#define MPU6500_I2C_SLV1_REG        (0x29)
#define MPU6500_I2C_SLV1_CTRL       (0x2A)
#define MPU6500_I2C_SLV2_ADDR       (0x2B)
#define MPU6500_I2C_SLV2_REG        (0x2C)
#define MPU6500_I2C_SLV2_CTRL       (0x2D)
#define MPU6500_I2C_SLV3_ADDR       (0x2E)
#define MPU6500_I2C_SLV3_REG        (0x2F)
#define MPU6500_I2C_SLV3_CTRL       (0x30)
#define MPU6500_I2C_SLV4_ADDR       (0x31)
#define MPU6500_I2C_SLV4_REG        (0x32)
#define MPU6500_I2C_SLV4_DO         (0x33)
#define MPU6500_I2C_SLV4_CTRL       (0x34)
#define MPU6500_I2C_SLV4_DI         (0x35)
#define MPU6500_I2C_MST_STATUS      (0x36)
#define MPU6500_INT_PIN_CFG         (0x37)
#define MPU6500_INT_ENABLE          (0x38)
#define MPU6500_INT_STATUS          (0x3A)
#define MPU6500_ACCEL_XOUT_H        (0x3B)
#define MPU6500_ACCEL_XOUT_L        (0x3C)
#define MPU6500_ACCEL_YOUT_H        (0x3D)
#define MPU6500_ACCEL_YOUT_L        (0x3E)
#define MPU6500_ACCEL_ZOUT_H        (0x3F)
#define MPU6500_ACCEL_ZOUT_L        (0x40)
#define MPU6500_TEMP_OUT_H          (0x41)
#define MPU6500_TEMP_OUT_L          (0x42)
#define MPU6500_GYRO_XOUT_H         (0x43)
#define MPU6500_GYRO_XOUT_L         (0x44)
#define MPU6500_GYRO_YOUT_H         (0x45)
#define MPU6500_GYRO_YOUT_L         (0x46)
#define MPU6500_GYRO_ZOUT_H         (0x47)
#define MPU6500_GYRO_ZOUT_L         (0x48)
#define MPU6500_EXT_SENS_DATA_00    (0x49)
#define MPU6500_EXT_SENS_DATA_01    (0x4A)
#define MPU6500_EXT_SENS_DATA_02    (0x4B)
#define MPU6500_EXT_SENS_DATA_03    (0x4C)
#define MPU6500_EXT_SENS_DATA_04    (0x4D)
#define MPU6500_EXT_SENS_DATA_05    (0x4E)
#define MPU6500_EXT_SENS_DATA_06    (0x4F)
#define MPU6500_EXT_SENS_DATA_07    (0x50)
#define MPU6500_EXT_SENS_DATA_08    (0x51)
#define MPU6500_EXT_SENS_DATA_09    (0x52)
#define MPU6500_EXT_SENS_DATA_10    (0x53)
#define MPU6500_EXT_SENS_DATA_11    (0x54)
#define MPU6500_EXT_SENS_DATA_12    (0x55)
#define MPU6500_EXT_SENS_DATA_13    (0x56)
#define MPU6500_EXT_SENS_DATA_14    (0x57)
#define MPU6500_EXT_SENS_DATA_15    (0x58)
#define MPU6500_EXT_SENS_DATA_16    (0x59)
#define MPU6500_EXT_SENS_DATA_17    (0x5A)
#define MPU6500_EXT_SENS_DATA_18    (0x5B)
#define MPU6500_EXT_SENS_DATA_19    (0x5C)
#define MPU6500_EXT_SENS_DATA_20    (0x5D)
#define MPU6500_EXT_SENS_DATA_21    (0x5E)
#define MPU6500_EXT_SENS_DATA_22    (0x5F)
#define MPU6500_EXT_SENS_DATA_23    (0x60)
#define MPU6500_I2C_SLV0_DO         (0x63)
#define MPU6500_I2C_SLV1_DO         (0x64)
#define MPU6500_I2C_SLV2_DO         (0x65)
#define MPU6500_I2C_SLV3_DO         (0x66)
#define MPU6500_I2C_MST_DELAY_CTRL  (0x67)
#define MPU6500_SIGNAL_PATH_RESET   (0x68)
#define MPU6500_MOT_DETECT_CTRL     (0x69)
#define MPU6500_USER_CTRL           (0x6A)
#define MPU6500_PWR_MGMT_1          (0x6B)
#define MPU6500_PWR_MGMT_2          (0x6C)
#define MPU6500_FIFO_COUNTH         (0x72)
#define MPU6500_FIFO_COUNTL         (0x73)
#define MPU6500_FIFO_R_W            (0x74)
#define MPU6500_WHO_AM_I            (0x75)	// mpu6500 id = 0x70
#define MPU6500_XA_OFFSET_H         (0x77)
#define MPU6500_XA_OFFSET_L         (0x78)
#define MPU6500_YA_OFFSET_H         (0x7A)
#define MPU6500_YA_OFFSET_L         (0x7B)
#define MPU6500_ZA_OFFSET_H         (0x7D)
#define MPU6500_ZA_OFFSET_L         (0x7E)
	
#define MPU6050_ID									(0x68)
#define MPU6500_ID									(0x70)			// mpu6500 id = 0x70

struct Axisf
{
	float x;
	float y;
	float z;
};

struct Axisi
{
	int x;
	int y;
	int z;
};

struct Angle_t
{
	float temp;
	int yaw;
	int last_yaw;
	int encoder_yaw;
	int last_encoder_yaw;
	int yaw_count;
	int pitch;
	int last_pitch;
	int encoder_pitch;
	int pitch_count;
	int roll;
	int last_roll;
	int encoder_roll;
	int roll_count;
};

typedef struct
{
	struct Axisf bias;
	char isBiasValueFound;
	char isBufferFilled;
	char initcomplete;
	struct Axisi buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
}BiasObj;

struct Quaternion_t
{
	float q0;	      /* 四元数 */
	float q1;
	float q2;
	float q3;
};

struct Integral_t
{
	float Kp;		    /* 比例增益（需要调整，以提高pitch的响应速度） */
	float Ki;		    /* 积分增益 */
	float exInt;
	float eyInt;
	float ezInt;		/* 积分误差累积 */
};

struct Other_t
{
	int gyroBiasFound;
	float accScaleSum;
	float accScale;
	unsigned int last_update;
	unsigned int now_update;
};

struct MPU6500_t
{
	struct Axisf gyro;              //陀螺仪解算后的数据，表示角速度
	struct Axisf acc;               //加速度计的数据，表示三轴角速度
	#ifdef Nine_axis 
	struct Axisf mag;               //磁力计的三轴数据
	#endif
	struct 
	{
		float x;
		float y;
		float z;
		float yaw;
		float yaw_offset;
	}attitude;          //姿态角数据（0~360）
	
	struct Angle_t angle;           //姿态角数据（0~8192）
	
	unsigned char imu_data[14];     //IIC接收数组
	struct Quaternion_t Quaternion; //四元数缓存
	struct Integral_t Integral;     //积分数据缓存
	struct Axisf gyroBias;          //陀螺仪零漂数据
	struct Other_t other;           //其他数据，含陀螺仪重新初始化的标记
	BiasObj gyroBiasRunning;        //原始数据缓存
};

extern struct MPU6500_t mpu6050;
extern struct MPU6500_t mpu6500_pitch;

int MPU6500_Init_Soft(char choice);
int MPU6500_Init_Hard(void);
int MPU6500_Init_SPI(void);
void MPU6500_t_init(struct MPU6500_t *MPU6500);
int GetImuDataSoft(unsigned char *imu_datda_);
int GetImuDataHard(unsigned char *imu_datda_);
int GetImuDataSPI(unsigned char *imu_datda_);
void imuDataHandle(struct MPU6500_t *MPU6500,char reset);
void imuUpdate(struct MPU6500_t *MPU6500, struct Axisf gyro, struct Axisf acc);

#endif
