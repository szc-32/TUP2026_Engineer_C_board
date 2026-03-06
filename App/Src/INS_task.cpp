/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪mpu6500，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过mpu6500的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1.  mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "INS_task.h"
//#include "calibrate_task.h"
//#include "bsp_imu_pwm.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_spi.h"
#include "bsp_buzzer.h"
#include "bsp_adc.h"
#include "mpu6050driver.h"
#include "mpu6050reg.h"
#include "mpu6050driver_middleware.h"
#include "ist8310driver.h"
#include "pid.h"
#include "ahrs.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定
// IMUWarnBuzzerOn() buzzer_on(95, 10000)
#define IMUWarnBuzzerOn() buzzer_on(0, 0) //开机陀螺仪校准蜂鸣器

#define IMUWarnBuzzerOFF() buzzer_off() //开机陀螺仪校准蜂鸣器关闭

//#define gyro_cali_fun(cali_scale, cali_offset, time_count) INS_cali_gyro((cali_scale), (cali_offset), (time_count))  //陀螺仪校准函数
//#define gyro_set_cali(cali_scale, cali_offset) INS_set_cali_gyro((cali_scale), (cali_offset)) //设置陀螺仪校准值


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \
	
#define MPU6500_BOARD_INSTALL_SPIN_MATRIX    \
        { 0.0f, 0.0f, 1.0f},    \
        { -1.0f, 0.0f, 0.0f},    \
        { 0.0f, -1.0f, 0.0f}    \
		
//		#define MPU6500_BOARD_INSTALL_SPIN_MATRIX    \
//        { 0.0f, 1.0f, 0.0f},    \
//        {-1.0f, 0.0f, 0.0f},    \
//        { 0.0f, 0.0f, 1.0f}    \

extern "C"{
/**
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[out]     mag: 加上零漂和旋转
  * @param[in]      mpu6500: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
  * @retval         none
  */
static void IMU_Cali_Slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], mpu6500_real_data_t *mpu6500, ist8310_real_data_t *ist8310);
/**
  * @brief          控制mpu6500的温度
  * @param[in]      temp:mpu6500的温度
  * @retval         none
  */
//static void IMU_temp_Control(fp32 temp);
/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
//static void imu_cmd_spi_dma(void);


extern SPI_HandleTypeDef hspi5;

static TaskHandle_t INSTask_Local_Handler;
static const float TimingTime = INS_DELTA_TICK * 0.001f;   //任务运行的时间 单位 s



static uint8_t mpu6500_spi_rxbuf[23]; //保存接收的原始数据
static uint8_t mpu6500_spi_txbuf[23]={
        MPU_INT_STATUS | MPU_SPI_READ_MSB};

//判断是否第一次进入，如果第一次则初始化四元数，之后更新四元数计算角度单位rad
static uint8_t updata_count = 0;

mpu6500_real_data_t mpu6500_real_data;

static fp32 Gyro_Scale_Factor[3][3] = {MPU6500_BOARD_INSTALL_SPIN_MATRIX}; //陀螺仪校准线性度
fp32 Gyro_Offset[3];
fp32 gyro_cali_offset[3];

static fp32 Accel_Scale_Factor[3][3] = {MPU6500_BOARD_INSTALL_SPIN_MATRIX}; //加速度校准线性度
fp32 Accel_Offset[3];
fp32 accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;

static fp32 Mag_Scale_Factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX}; //磁力计校准线性度
fp32 Mag_Offset[3];
fp32 mag_cali_offset[3];

static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static PID_t imu_temp_pid;

//static const float timing_time = 0.001f;   //tast run time , unit s.任务运行的时间 单位 s






//加速度计低通滤波
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};




fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad


#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t INSTaskStack;
#endif

uint32_t current_time1,current_time2,current_time3;
void INS_Task(void const * argument)
{	
//	
	//获取当前任务的任务句柄，用于任务通知
	INSTask_Local_Handler = xTaskGetHandle(pcTaskGetName(NULL));
	//wait a time
    vTaskDelay(INS_TASK_INIT_TIME);

	  //mpu6500初始化
    while(mpu6500_init()!= MPU6500_NO_ERROR)
    {
//        osDelay(100);
		;
    }
//    while(ist8310_init())
//    {
//        osDelay(100);
//    }
	
//	//获取当前任务的任务句柄，用于任务通知
//	INSTask_Local_Handler = xTaskGetHandle(pcTaskGetName(NULL));
	 
	 //陀螺仪数据读取
	 mpu6500_read_muli_reg(MPU_INT_STATUS, mpu6500_spi_rxbuf, DMA_RX_NUM);
	 
	 //
	 IMU_Cali_Slove(INS_gyro, INS_accel, INS_mag, &mpu6500_real_data, &ist8310_real_data);
	
//	//获取当前任务的任务句柄，用于任务通知
//	INSTask_Local_Handler = xTaskGetHandle(pcTaskGetName(NULL));
	    //set spi frequency
    hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    
    if (HAL_SPI_Init(&hspi5) != HAL_OK)
    {
        Error_Handler();
    }
	
	SPI5_DMA_init((uint32_t)mpu6500_spi_txbuf, (uint32_t)mpu6500_spi_rxbuf, DMA_RX_NUM);
	

	
    while (1) 
    {
//			current_time1=xTaskGetTickCount();
		//等待外部中断中断唤醒任务
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
			;
        }
//		
//		 mpu6500_SPI_NS_L();
//         SPI5_DMA_enable((uint32_t)mpu6500_spi_txbuf,(uint32_t)mpu6500_spi_rxbuf,DMA_RX_NUM);
		
//        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
//        {
//			;
//        }
//		 vTaskDelayUntil(&INS_LastWakeTime, INS_DELTA_TICK);
		 mpu6500_read_muli_reg(MPU_INT_STATUS, mpu6500_spi_rxbuf, DMA_RX_NUM);
		
		//将读取到的mpu6500原始数据处理成国际单位的数据
        mpu6500_read_over((mpu6500_spi_rxbuf + MPU6500_RX_BUF_DATA_OFFSET), &mpu6500_real_data);
		
//		ist8310_read_over((mpu6500_spi_rxbuf + IST8310_RX_BUF_DATA_OFFSET), &ist8310_real_data);
		
		//减去零漂以及旋转坐标系
        IMU_Cali_Slove(INS_gyro, INS_accel, INS_mag, &mpu6500_real_data, &ist8310_real_data);
		

		
		if( mpu6500_real_data.status & 1 << MPU_DATA_READY_BIT)
        {
			if (updata_count == 0)
            {
                
                imu_temp_pid.init(PID_POSITION, imu_temp_PID, MPU6500_TEMPERATURE_PID_MAX_OUT, MPU6500_TEMPERATURE_PID_MAX_IOUT);

                //初始化四元数
                AHRS_init(INS_quat, INS_accel, INS_mag);
                get_angle(INS_quat, INS_angle, INS_angle + 1, INS_angle + 2);

                accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
                accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
                accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
                updata_count++;
            }
			
			            else
            {
                //加速度计低通滤波
                accel_fliter_1[0] = accel_fliter_2[0];
                accel_fliter_2[0] = accel_fliter_3[0];

                accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

                accel_fliter_1[1] = accel_fliter_2[1];
                accel_fliter_2[1] = accel_fliter_3[1];

                accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

                accel_fliter_1[2] = accel_fliter_2[2];
                accel_fliter_2[2] = accel_fliter_3[2];

                accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

                //更新四元数
                AHRS_update(INS_quat, TimingTime, INS_gyro, accel_fliter_3, INS_mag);
                get_angle(INS_quat, INS_angle, INS_angle + 1, INS_angle + 2);

                //陀螺仪开机校准
//                {
////                    static uint16_t start_gyro_cali_time = 0;
////                    if(start_gyro_cali_time == 0)
////                    {
////                        Gyro_Offset[0] = gyro_cali_offset[0];
////                        Gyro_Offset[1] = gyro_cali_offset[1];
////                        Gyro_Offset[2] = gyro_cali_offset[2];
////                        start_gyro_cali_time++;
////                    }
////                    else if (start_gyro_cali_time < GYRO_OFFSET_START_TIME)
////                    {
////                        IMUWarnBuzzerOn();
////                        if( first_temperate)
////                        {
////                            //当进入gyro_offset函数，如果无运动start_gyro_cali_time++，如果有运动 start_gyro_cali_time = 0
////                            gyro_offset(Gyro_Offset, INS_gyro, mpu6500_real_data.status, &start_gyro_cali_time);
////                        }
////                    }
////                    else if (start_gyro_cali_time == GYRO_OFFSET_START_TIME)
////                    {

////						IMUWarnBuzzerOFF();
////                        start_gyro_cali_time++;
////                    }
//                }       //陀螺仪开机校准   code end

            }           //update count if   code end
		}
		
		//请在这里添加例如温度控制代码

        //IMU_temp_Control(mpu6500_real_data.temp);
		
#if INCLUDE_uxTaskGetStackHighWaterMark
        INSTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
//		current_time2=xTaskGetTickCount();
//		current_time3=current_time2-current_time1;
    }		//while(1) end

}

/**
  * @brief          校准陀螺仪
  * @author         RM
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
  * @param[in]      陀螺仪的时刻，每次在gyro_offset调用会加1,
  * @retval         返回空
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
    if (first_temperate)
    {
        if( *time_count == 0)
        {
            Gyro_Offset[0] = gyro_cali_offset[0];
            Gyro_Offset[1] = gyro_cali_offset[1];
            Gyro_Offset[2] = gyro_cali_offset[2];
        }
        gyro_offset(Gyro_Offset, INS_gyro, mpu6500_real_data.status, time_count);

        cali_offset[0] = Gyro_Offset[0];
        cali_offset[1] = Gyro_Offset[1];
        cali_offset[2] = Gyro_Offset[2];
				
//				IMUSensor_Offset.GYRO_Offset.x= cali_offset[0];
//				IMUSensor_Offset.GYRO_Offset.y= cali_offset[1];
//				IMUSensor_Offset.GYRO_Offset.z= cali_offset[2];
		cali_scale[0] = 1.0f;
        cali_scale[1] = 1.0f;
        cali_scale[2] = 1.0f;
    }
}

/**
  * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
  * @author         RM
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂
  * @retval         返回空
  */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
	
//	Gyro_Offset[0] = gyro_cali_offset[0];
//    Gyro_Offset[1] = gyro_cali_offset[1];
//    Gyro_Offset[2] = gyro_cali_offset[2];
}

const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}
const fp32 *get_MPU6500_Gyro_Data_Point(void)
{
    return INS_gyro;
}

const fp32 *get_MPU6500_Accel_Data_Point(void)
{
    return INS_accel;
}

static void IMU_Cali_Slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], mpu6500_real_data_t *mpu6500, ist8310_real_data_t *ist8310)
{
   for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = mpu6500->gyro[0] * Gyro_Scale_Factor[i][0] + mpu6500->gyro[1] * Gyro_Scale_Factor[i][1] + mpu6500->gyro[2] * Gyro_Scale_Factor[i][2] + Gyro_Offset[i];
        accel[i] = mpu6500->accel[0] * Accel_Scale_Factor[i][0] + mpu6500->accel[1] * Accel_Scale_Factor[i][1] + mpu6500->accel[2] * Accel_Scale_Factor[i][2] + Accel_Offset[i];
        mag[i] = ist8310->mag[0] * Mag_Scale_Factor[i][0] + ist8310->mag[1] * Mag_Scale_Factor[i][1] + ist8310->mag[2] * Mag_Scale_Factor[i][2] + Mag_Offset[i];
    }
}

//static void IMU_temp_Control(fp32 temp)
//{
//    uint16_t tempPWM;
//    static uint8_t temp_constant_time = 0 ;
//    if (first_temperate)
//    {
//		PID_calc(&imu_temp_pid, temp,get_control_temperate()) ;
//        if (imu_temp_pid.out < 0.0f)
//        {
//            imu_temp_pid.out = 0.0f;
//        }
//        tempPWM = (uint16_t)imu_temp_pid.out;
//        IMU_temp_PWM(tempPWM);
//    }
//    else
//    {
//        //在没有达到设置的温度，一直最大功率加热
//        if (temp > get_control_temperate())
//        {
//            temp_constant_time ++;
//            if(temp_constant_time > 200)
//            {
//                //达到设置温度，将积分项设置为一半最大功率，加速收敛
//                first_temperate = 1;
//                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
//            }
//        }

//        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
//    }
//}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 if(GPIO_Pin == GPIO_PIN_8)
    {
//		mpu6500_SPI_NS_L();
//        SPI5_DMA_enable((uint32_t)mpu6500_spi_txbuf,(uint32_t)mpu6500_spi_rxbuf,DMA_RX_NUM);
		//唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR((INSTask_Local_Handler), &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
	}
}

void DMA2_Stream5_IRQHandler(void)
{
	if(__HAL_DMA_GET_FLAG(hspi5.hdmarx, DMA_FLAG_TCIF1_5) != RESET)
    {
		__HAL_DMA_CLEAR_FLAG(hspi5.hdmarx,DMA_FLAG_TCIF1_5);
		mpu6500_SPI_NS_H();
		
        //唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INSTask_Local_Handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
	}
	
}

}
