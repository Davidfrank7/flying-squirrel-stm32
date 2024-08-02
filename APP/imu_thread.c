#include "main.h"
#include "wit_c_sdk.h"
#include "REG.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

osEventFlagsId_t imuEventHandle;
const osEventFlagsAttr_t imuEvent_attributes = {
  .name = "imuEvent"
};

osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

uint8_t imu_rx_data;

// 注册回调函数
static void imu_serial_tx(uint8_t *data, uint32_t len);
static void imu_data_update_handler(uint32_t reg, uint32_t reg_num);

void imu_thread(void *argument)
{
    uint32_t imu_event_flags = 0;
    
    float acc[3], gyro[3], angle[3];

    UNUSED(argument);

    // 创建IMU事件组
    imuEventHandle = osEventFlagsNew(&imuEvent_attributes);

	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(imu_serial_tx);
    WitRegisterCallBack(imu_data_update_handler);

    // 开始UART接收中断
    HAL_UART_Receive_IT(&huart2, &imu_rx_data, 1);

    while (1)
    {
        // 等待IMU数据更新事件
        imu_event_flags = osEventFlagsWait(imuEventHandle, ACC_UPDATE | GYRO_UPDATE | ANGLE_UPDATE | MAG_UPDATE | READ_UPDATE, osFlagsWaitAny, osWaitForever);

        if (imu_event_flags)
        {
            for (int i = 0; i < 3; i++)
            {
                acc[i] = sReg[AX + i] / 32768.0f * 16.0f;;
                gyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
                angle[i] = sReg[Yaw + i] / 32768.0f * 180.0f;
            }
            if (imu_event_flags & ACC_UPDATE)
            {
                // 处理加速度数据
                // TODO: 通过ROS上报加速度数据
            }
            if (imu_event_flags & GYRO_UPDATE)
            {
                // 处理陀螺仪数据
            }
            if (imu_event_flags & ANGLE_UPDATE)
            {
                // 处理角度数据
            }
            if (imu_event_flags & MAG_UPDATE)
            {
                // 处理磁场数据
            }
        }

    }
}

void imu_thread_init(void)
{
    imuTaskHandle = osThreadNew(imu_thread, NULL, &imuTask_attributes);
    if (imuTaskHandle == NULL)
    {
        Error_Handler();
    }
}

// 串口接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        // 调用IMU接口解析数据
        WitSerialDataIn(imu_rx_data);
        
        // 重新启动UART接收中断
        HAL_UART_Receive_IT(&huart2, &imu_rx_data, 1);
    }
}

static void imu_serial_tx(uint8_t *data, uint32_t len)
{
    HAL_UART_Transmit(&huart2, data, len, 1000);
}

static void imu_data_update_handler(uint32_t reg, uint32_t reg_num)
{
    uint32_t i = 0;
    for(i = 0; i < reg_num; i++)
    {
        switch(reg)
        {
            case AZ:
                osEventFlagsSet(imuEventHandle, ACC_UPDATE);
                break;
            case GZ:
                osEventFlagsSet(imuEventHandle, GYRO_UPDATE);
                break;
            case HZ:
                osEventFlagsSet(imuEventHandle, MAG_UPDATE);
                break;
            case Yaw:
                osEventFlagsSet(imuEventHandle, ANGLE_UPDATE);
                break;
            default:
                osEventFlagsSet(imuEventHandle, READ_UPDATE);
                break;
        }
        reg++;
    }
}