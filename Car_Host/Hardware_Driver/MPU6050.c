#include "MPU6050.h"


int32_t acc_x_offset = 0;
int32_t acc_y_offset = 0;
int32_t acc_z_offset = 0;

int32_t gyro_x_offset = 0;
int32_t gyro_y_offset = 0;
int32_t gyro_z_offset = 0;


/**
 * @brief 读取 MPU6050 寄存器
 * 
 * @param reg 寄存器地址
 * @return data 寄存器值
 */

void MPU6050_Write_Reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}

/**
 * @brief 读取 MPU6050 寄存器
 * 
 * @param reg 寄存器地址
 * @return data 寄存器值
 */
void MPU6050_Read_Reg(uint8_t reg, uint8_t *data)
{
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR_READ, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 10);
}



/**
 * @brief 在初始化后进行零漂校准
 * @brief 飞机停放稳定
 */
void MPU6050_calculate_offset(void)
{
    Gyro_Accel_Struct gyro_accel_data = {0};
    int32_t acc_x_sum = 0;
    int32_t acc_y_sum = 0;
    int32_t acc_z_sum = 0;

    int32_t gyro_x_sum = 0;
    int32_t gyro_y_sum = 0;
    int32_t gyro_z_sum = 0;
    for(uint8_t i = 0; i < 100; i ++)
    {
        //读取
        MPU6050_Get_Data(&gyro_accel_data);
        acc_x_sum += (gyro_accel_data.accel.accel_x - 0);
        acc_y_sum += (gyro_accel_data.accel.accel_y - 0);
        acc_z_sum += (gyro_accel_data.accel.accel_z - 16384);

        gyro_x_sum += (gyro_accel_data.gyro.gyro_x - 0);
        gyro_y_sum += (gyro_accel_data.gyro.gyro_y - 0);
        gyro_z_sum += (gyro_accel_data.gyro.gyro_z - 0);

        //每次测量加延时
        vTaskDelay(6);

    }
    acc_x_offset = acc_x_sum / 100;
    acc_y_offset = acc_y_sum / 100;
    acc_z_offset = acc_z_sum / 100;

    gyro_x_offset = gyro_x_sum / 100;
    gyro_y_offset = gyro_y_sum / 100;
    gyro_z_offset = gyro_z_sum / 100;
}





/**
 * @brief 初始化 MPU6050
 */
void MPU6050_Init(void)
{
    //1.重启芯片 重置所有寄存器的值
    MPU6050_Write_Reg(0x6B, 0x80);
	
	vTaskDelay(200);
	

    //唤醒MPU6050 进入正常工作状态
    MPU6050_Write_Reg(0x6B, 0x00);
	
    //2.选择合适的量程
    //2.1填写角速度量程为±2000°/s
    MPU6050_Write_Reg(0x1B, 3 << 3);

    //2.2 填写加速度量程为±2g
    MPU6050_Write_Reg(0x1C, 0x00);

    //3.关闭中断使能
    MPU6050_Write_Reg(0x38, 0x00);

    //4.用户配置寄存器 不使用FIFO 不使用扩展的I2C
    MPU6050_Write_Reg(0x6A, 0x00);

    //5.设置采样率 陀螺仪监控三轴加速度三周角速度
    //设置采样分频为2
    MPU6050_Write_Reg(0x19, 0x01);

    //6.设置低通滤波得值为184Hz
    MPU6050_Write_Reg(0x1A, 1);

    //7.配置使用的时钟系统为添加PLL的
    MPU6050_Write_Reg(0x6B,0x01);

    //8.使能加速度传感器和角速度传感器
    MPU6050_Write_Reg(0x6C, 0x00);
	
//	uint8_t device_id = 0;
//    MPU6050_Read_Reg(MPU_DEVICE_ID_REG, &device_id);  // 0x75 寄存器
//	if (device_id != 0x70) 
//	{
//		OLED_ShowString(0, 0, "0",OLED_8X16);
//		OLED_Update();
//	}

    //9.进行零漂校准
    MPU6050_calculate_offset();

         
}


/**
 * @brief 获取三轴角速度
 * 
 * @param gyro 
 */
void MPU6050_Get_Gyro(Gyro_struct *gyro)
{
    //存储角速度的寄存器地址从0x43开始 高八位在前 xyz顺序
    uint8_t hight = 0;
    uint8_t low = 0;
    //x轴
    MPU6050_Read_Reg(MPU_GYRO_XOUTH_REG, &hight);
    MPU6050_Read_Reg(MPU_GYRO_XOUTL_REG, &low);
    gyro->gyro_x = (hight << 8 | low) - gyro_x_offset;
    //y轴
    MPU6050_Read_Reg(MPU_GYRO_YOUTH_REG, &hight);
    MPU6050_Read_Reg(MPU_GYRO_YOUTL_REG, &low);
    gyro->gyro_y = (hight << 8 | low) - gyro_y_offset;
    //z轴
    MPU6050_Read_Reg(MPU_GYRO_ZOUTH_REG, &hight);
    MPU6050_Read_Reg(MPU_GYRO_ZOUTL_REG, &low);
    gyro->gyro_z = (hight << 8 | low) - gyro_z_offset;
}


/**
 * @brief 获取三轴加速度
 * 
 * @param acc 
 */
void MPU6050_Get_Acc(Accel_struct *acc)
{
    uint8_t hight = 0;
    uint8_t low = 0;
    //x轴
    MPU6050_Read_Reg(MPU_ACCEL_XOUTH_REG, &hight);
    MPU6050_Read_Reg(MPU_ACCEL_XOUTL_REG, &low);
    acc->accel_x = (hight << 8 | low - acc_x_offset);
    //y轴
    MPU6050_Read_Reg(MPU_ACCEL_YOUTH_REG, &hight);
    MPU6050_Read_Reg(MPU_ACCEL_YOUTL_REG, &low);
    acc->accel_y = (hight << 8 | low) - acc_y_offset;
    //z轴
    MPU6050_Read_Reg(MPU_ACCEL_ZOUTH_REG, &hight);
    MPU6050_Read_Reg(MPU_ACCEL_ZOUTL_REG, &low);
    acc->accel_z = (hight << 8 | low) - acc_z_offset;
}


/**
 * @brief 获取6轴数据
 * 
 * @param data 
 */
void MPU6050_Get_Data(Gyro_Accel_Struct *data)
{
    MPU6050_Get_Gyro(&data->gyro);

    MPU6050_Get_Acc(&data->accel);
}
