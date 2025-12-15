#include "icm42605.h"
#include "i2c.h"
#include "port.h"

uint8_t USE_IMU = 0;           //外接IMU,当检测到IMU时赋值为1

#ifndef L151_DEV
icm42605RawData_t stAccData[1],stGyroData[1];

volatile static float accSensitivity;
volatile static float gyroSensitivity;

/*******************************************************************************
* 名    称： icm42605_read_reg
* 功    能： 读取单个寄存器的值
* 入口参数： reg: 寄存器地址
* 出口参数： 当前寄存器地址的值
* 作　　者： 
* 创建日期： 2021-05-21
* 修    改：
* 修改日期：
* 备    注： 
*******************************************************************************/
static uint8_t icm42605_read_reg(uint8_t reg)
{
    uint8_t regval = 0;
    HAL_I2C_Mem_Read(&hi2c1, ICM42605_IIC_ADD, reg, I2C_MEMADD_SIZE_8BIT, &regval, 1, 0xFF);
    return regval;
}

/*******************************************************************************
* 名    称： icm42605_read_regs
* 功    能： 连续读取多个寄存器的值
* 入口参数： reg: 起始寄存器地址 *buf数据指针,uint16_t len长度
* 出口参数： 无
* 作　　者： 
* 创建日期： 2021-05-21
* 修    改：
* 修改日期：
* 备    注： 
*******************************************************************************/
static void icm42605_read_regs(uint8_t reg, uint8_t* buf, uint16_t len)
{
    HAL_I2C_Mem_Read(&hi2c1, ICM42605_IIC_ADD, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xFF);
}


/*******************************************************************************
* 名    称： icm42605_write_reg
* 功    能： 向单个寄存器写数据
* 入口参数： reg: 寄存器地址 value:数据
* 出口参数： 
* 作　　者： 
* 创建日期： 2021-05-21
* 修    改：
* 修改日期：
* 备    注： 
*******************************************************************************/
static uint8_t icm42605_write_reg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, ICM42605_IIC_ADD, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 0xFF);
}



float Icm42605SetAres(uint8_t Ascale)
{
    switch(Ascale)
    {
    // page 11
    case AFS_2G:
        accSensitivity = 16384.0f;
        break;
    case AFS_4G:
        accSensitivity = 8192.0f;
        break;
    case AFS_8G:
        accSensitivity = 4096.0f;
        break;
    case AFS_16G:
        accSensitivity = 2048.0f;
        break;
    }

    return accSensitivity;
}

float Icm42605SetGres(uint8_t Gscale)
{
    //page 10
    switch(Gscale)
    {
    case GFS_15_625DPS:
        gyroSensitivity = 2097.2f;
        break;
    case GFS_31_25DPS:
        gyroSensitivity = 1048.6f;
        break;
    case GFS_62_5DPS:
        gyroSensitivity = 524.3f;
        break;
    case GFS_125DPS:
        gyroSensitivity = 262.0f;
        break;
    case GFS_250DPS:
        gyroSensitivity = 131.0f;
        break;
    case GFS_500DPS:
        gyroSensitivity = 65.5f;
        break;
    case GFS_1000DPS:
        gyroSensitivity = 32.8f;
        break;
    case GFS_2000DPS:
        gyroSensitivity = 16.4f;
        break;
    }
    return gyroSensitivity;
}

/*******************************************************************************
* 名    称： Icm42605RegCfg
* 功    能： Icm42605 寄存器配置
* 入口参数： 无
* 出口参数： 无
* 作　　者： 
* 创建日期： 2021-05-21
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/
int8_t Icm42605RegCfg(void)
{
    uint8_t reg_val = 0;

    if (HAL_I2C_IsDeviceReady(&hi2c1, ICM42605_IIC_ADD, 1, 1000) != HAL_OK)
    {
		/* Return false */
		return -1;
	}
    
    icm42605_write_reg(ICM42605_REG_BANK_SEL, 0x00); //设置bank 0区域寄存器
    icm42605_write_reg(ICM42605_DEVICE_CONFIG, 0x01); //软复位传感器
    HAL_Delay(100);

    reg_val = icm42605_read_reg(ICM42605_WHO_AM_I); //读取 who am i 寄存器 
    if((reg_val == ICM42605_ID) || (reg_val == ICM40605_ID)) 
    {
        // icm42605_write_reg(ICM42605_REG_BANK_SEL, 1); //设置bank 1区域寄存器
        // icm42605_write_reg(ICM42605_INTF_CONFIG4, 0x02); //设置为4线SPI通信
        /*****配置FIFO模式及其中断***************************************************************************/
        /*配置FIFO模式*/
        icm42605_write_reg(ICM42605_REG_BANK_SEL, 0x00); //设置bank 0区域寄存器
        icm42605_write_reg(ICM42605_FIFO_CONFIG, 0x40); //Stream-to-FIFO Mode
        /*配置FIFO中断*/
        reg_val = icm42605_read_reg(ICM42605_INT_SOURCE0);
        icm42605_write_reg(ICM42605_INT_SOURCE0, 0x00); //写入ICM42605_FIFO_CONFIG2、3前FIFO_WM_EN 必须清零
        icm42605_write_reg(ICM42605_FIFO_CONFIG2, 0x00); // watermark
        icm42605_write_reg(ICM42605_FIFO_CONFIG3, 0x02); // watermark
        icm42605_write_reg(ICM42605_INT_SOURCE0, reg_val);  
        icm42605_write_reg(ICM42605_FIFO_CONFIG1, 0x63); // Enable the accel and gyro to the FIFO
        /*配置中断管脚*/
        icm42605_write_reg(ICM42605_REG_BANK_SEL, 0x00);
        icm42605_write_reg(ICM42605_INT_CONFIG, 0x36);
        /*使能中断*/
        icm42605_write_reg(ICM42605_REG_BANK_SEL, 0x00);
        reg_val = icm42605_read_reg(ICM42605_INT_SOURCE0);
        reg_val |= (1 << 2); //FIFO_THS_INT1_ENABLE
        icm42605_write_reg(ICM42605_INT_SOURCE0, reg_val);
        /************************************************************************************************/

        /*****配置加速度计50H*****************************************************************************/
        Icm42605SetAres(AFS_8G);
        icm42605_write_reg(ICM42605_REG_BANK_SEL, 0x00);
        reg_val = icm42605_read_reg(ICM42605_ACCEL_CONFIG0);
        reg_val |= (AFS_8G << 5);   //量程 ±8g
        reg_val |= (AODR_200Hz);     //输出速率 50HZ
        icm42605_write_reg(ICM42605_ACCEL_CONFIG0, reg_val);

        /*****配置陀螺仪4FH*******************************************************************************/
        Icm42605SetGres(GFS_1000DPS);
        icm42605_write_reg(ICM42605_REG_BANK_SEL, 0x00);
        reg_val = icm42605_read_reg(ICM42605_GYRO_CONFIG0);
        reg_val |= (GFS_1000DPS << 5);   //量程 ±1000dps
        reg_val |= (GODR_200Hz);          //输出速率 50HZ
        icm42605_write_reg(ICM42605_GYRO_CONFIG0, reg_val);

        /*****使能温度、加速度、角速度测量*****************************************************************/
        icm42605_write_reg(ICM42605_REG_BANK_SEL, 0x00);
        reg_val = icm42605_read_reg(ICM42605_PWR_MGMT0); //读取PWR—MGMT0当前寄存器的值
        reg_val &= ~(1 << 5);//使能温度测量
        reg_val |= ((3) << 2);//设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
        reg_val |= (3);//设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
        icm42605_write_reg(ICM42605_PWR_MGMT0, reg_val);
        HAL_Delay(1); //操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作

        return 0;
    }
    return -1;
}
/*******************************************************************************
* 名    称： Icm42605Init
* 功    能： Icm42605 传感器初始化
* 入口参数： 无
* 出口参数： 0: 初始化成功  其他值: 初始化失败
* 作　　者： 
* 创建日期： 2021-05-21
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/
int8_t Icm42605Init(void)
{
    uint8_t err_count = 0;
    uint8_t ret = 0;
    while(err_count < 2)
    {
        if(Icm42605RegCfg() == 0)
        {
            ret = 1;
            break;
        }
        else
        {
            err_count++;
        }
        HAL_Delay(10);
    }
    return ret;

}

/*******************************************************************************
* 名    称： IcmGetTemperature
* 功    能： 读取Icm42605 内部传感器温度
* 入口参数： 无
* 出口参数： 无
* 作　　者： 
* 创建日期： 2021-05-21
* 修    改：
* 修改日期：
* 备    注： datasheet page62
*******************************************************************************/
int8_t IcmGetTemperature(int16_t* pTemp)
{
    uint8_t buffer[2] = {0};

    icm42605_read_regs(ICM42605_TEMP_DATA1, buffer, 2);

    *pTemp = (int16_t)(((int16_t)((buffer[0] << 8) | buffer[1])) / 132.48 + 25);
    return 0;
}

/*******************************************************************************
* 名    称： IcmGetAccelerometer
* 功    能： 读取Icm42605 加速度的值
* 入口参数： 三轴加速度的值
* 出口参数： 无
* 作　　者： 
* 创建日期： 2021-05-21
* 修    改：
* 修改日期：
* 备    注： datasheet page62
*******************************************************************************/
int8_t IcmGetAccelerometer(icm42605RawData_t* accData)
{
    uint8_t buffer[6] = {0};
    short acc_x,acc_y,acc_z;


    icm42605_read_regs(ICM42605_ACCEL_DATA_X1, buffer, 6);

    acc_x = ((uint16_t)buffer[0] << 8) | buffer[1];
    acc_y = ((uint16_t)buffer[2] << 8) | buffer[3];
    acc_z = ((uint16_t)buffer[4] << 8) | buffer[5];

    accData->x = ((float)acc_x / accSensitivity);
    accData->y = ((float)acc_y / accSensitivity);
    accData->z = ((float)acc_z / accSensitivity);

    return 0;
}

/*******************************************************************************
* 名    称： IcmGetGyroscope
* 功    能： 读取Icm42605 陀螺仪的值
* 入口参数： 三轴陀螺仪的值
* 出口参数： 无
* 作　　者： 
* 创建日期： 2021-05-21
* 修    改：
* 修改日期：
* 备    注： datasheet page63
*******************************************************************************/
int8_t IcmGetGyroscope(icm42605RawData_t* GyroData)
{
    uint8_t buffer[6] = {0};
    short gyr_x,gyr_y,gyr_z;

    icm42605_read_regs(ICM42605_GYRO_DATA_X1, buffer, 6);

    gyr_x = ((uint16_t)buffer[0] << 8) | buffer[1];
    gyr_y = ((uint16_t)buffer[2] << 8) | buffer[3];
    gyr_z = ((uint16_t)buffer[4] << 8) | buffer[5];

    GyroData->x = ((float)gyr_x / gyroSensitivity);
    GyroData->y = ((float)gyr_y / gyroSensitivity);
    GyroData->z = ((float)gyr_z / gyroSensitivity);

    return 0;
}

/*******************************************************************************
* 名    称： IcmGetRawData
* 功    能： 读取Icm42605加速度陀螺仪数据
* 入口参数： 六轴
* 出口参数： 无
* 作　　者： 
* 创建日期： 2021-05-21
* 修    改：
* 修改日期：
* 备    注： datasheet page62,63
*******************************************************************************/
void IcmGetRawData(icm42605RawData_t* AccData, icm42605RawData_t* GyroData)
{
    
    uint8_t buffer[12] = {0};
    short acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z;

    // icm42605_read_regs(ICM42605_ACCEL_DATA_X1, buffer, 12);

    // icm_data->accX   = ((uint16_t)buffer[0] << 8)  | buffer[1];
    // icm_data->accY   = ((uint16_t)buffer[2] << 8)  | buffer[3];
    // icm_data->accZ   = ((uint16_t)buffer[4] << 8)  | buffer[5];
    // icm_data->gyroX  = ((uint16_t)buffer[6] << 8)  | buffer[7];
    // icm_data->gyroY  = ((uint16_t)buffer[8] << 8)  | buffer[9];
    // icm_data->gyroZ  = ((uint16_t)buffer[10] << 8) | buffer[11];

    icm42605_read_regs(ICM42605_ACCEL_DATA_X1, buffer, 6);
    acc_x   = ((uint16_t)buffer[0] << 8)  | buffer[1];
    acc_y   = ((uint16_t)buffer[2] << 8)  | buffer[3];
    acc_z   = ((uint16_t)buffer[4] << 8)  | buffer[5];

    icm42605_read_regs(ICM42605_GYRO_DATA_X1, buffer, 6);
    gyr_x   = ((uint16_t)buffer[0] << 8)  | buffer[1];
    gyr_y   = ((uint16_t)buffer[2] << 8)  | buffer[3];
    gyr_z   = ((uint16_t)buffer[4] << 8)  | buffer[5];

    AccData->x = ((float)acc_x / accSensitivity * 9.8);  //单位：g 转为(m/s^2)
    AccData->y = ((float)acc_y / accSensitivity * 9.8);
    AccData->z = ((float)acc_z / accSensitivity * 9.8);

    GyroData->x = ((float)gyr_x / gyroSensitivity * 0.01745);  //单位为：°/s 转为rad/s
    GyroData->y = ((float)gyr_y / gyroSensitivity * 0.01745);
    GyroData->z = ((float)gyr_z / gyroSensitivity * 0.01745);

}

void IcmGetS16Data(int16_t* acc, int16_t* gyro)
{
    icm42605_read_regs(ICM42605_ACCEL_DATA_X1, (uint8_t*)acc, 6);
    icm42605_read_regs(ICM42605_GYRO_DATA_X1, (uint8_t*)gyro, 6);

}

//四元数姿态解算
float NormAccz;
const float M_PI = 3.1415926535;
const float RtA = 57.2957795f;
const float Gyro_G = 0.03051756f;	//陀螺仪int16角速度除以分辨率得到度 量程1000
const float Gyro_Gr = 0.0005326f; //陀螺仪int16角速度转度再转弧度 量程1000
#define squa( Sq )   (((float)Sq)*((float)Sq))
float Q_rsqrt(float number);

void GetAngle(const ImuData_t *pMpu,angel_t *pAngE, float dt) 
{		
	volatile struct V
	{
		float x;
		float y;
		float z;
	} Gravity,Acc,Gyro,AccGravity;

	static struct V GyroIntegError = {0};
	static  float KpDef = 0.8f ;
	static  float KiDef = 0.0003f;
	static Quaternion NumQ = {1, 0, 0, 0};  // 四元素
	float q0_t,q1_t,q2_t,q3_t;
	float NormQuat; 
	float HalfTime = dt * 0.5f;  
	// 提取等效旋转矩阵中的重力分量 
	Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);								
	Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);						  
	Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);	
	
	// 加速度归一化
    NormQuat = Q_rsqrt(squa(pMpu->accX)+ squa(pMpu->accY) +squa(pMpu->accZ));
        
    Acc.x = pMpu->accX * NormQuat;
    Acc.y = pMpu->accY * NormQuat;
    Acc.z = pMpu->accZ * NormQuat;	
	
 	//向量叉乘得出的值
	AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
	AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
	AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
	
	//再做加速度积分补偿角速度的补偿值
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
        
        //角速度融合加速度积分补偿值
    Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//弧度制
    Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
    Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;		
	
	// 一阶龙格库塔法, 更新四元数
	q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
	q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
	q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
	q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;
	
	NumQ.q0 += q0_t;
	NumQ.q1 += q1_t;
	NumQ.q2 += q2_t;
	NumQ.q3 += q3_t;
	// 四元数归一化
	NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
	NumQ.q0 *= NormQuat;
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;	
	
	// 四元数转欧拉角
	{
		/*机体坐标系下的Z方向向量*/
		float vecxZ = 2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3 ;/*矩阵(3,1)项*/
		float vecyZ = 2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;/*矩阵(3,2)项*/
		float veczZ =  1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2;	/*矩阵(3,3)项*/		 
#ifdef	YAW_GYRO
		*(float *)pAngE = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;  //yaw
#else
		float yaw_G = pMpu->gyroZ * Gyro_G;
		if((yaw_G > 3.0f) || (yaw_G < -3.0f)) //数据太小可以认为是干扰，不是偏航动作
		{
			pAngE->yaw  += yaw_G * dt;			
		}
#endif
		pAngE->pitch  =  asin(vecxZ)* RtA;						
		
		pAngE->roll	= atan2f(vecyZ,veczZ) * RtA;	//PITCH 		
        
		NormAccz = pMpu->accX* vecxZ + pMpu->accY * vecyZ + pMpu->accZ * veczZ;	/*Z轴加速度*/				
	}
}

float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );
	return y;
} 



#endif