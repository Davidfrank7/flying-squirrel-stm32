#include "app.h"
// #include "i2c.h"
#include "vl53l0x.h"
// #include "vl53l0x_api.h"

osThreadId_t tof_task_handle;
const osThreadAttr_t tof_task_attributes = {
  .name = "tof_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

uint8_t Message[64];
uint8_t MessageLen;

extern VL53L0X_Dev_t vl53l0x_dev;

VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_Dev_t  vl53l0x_c; // center module
VL53L0X_DEV    Dev = &vl53l0x_c;


void tof_thread(void *argument)
{
    // uint32_t refSpadCount;
    // uint8_t isApertureSpads;
    // uint8_t VhvSettings;
    // uint8_t PhaseCal;

    // UNUSED(argument);

    // // Dev->I2cHandle = &tof_hi2c;
    // Dev->I2cDevAddr = 0x52;
    
    // HAL_GPIO_WritePin(TOF_XSHUT_GPIO_Port, TOF_XSHUT_Pin, GPIO_PIN_RESET); // Disable XSHUT
    // osDelay(20);
    // HAL_GPIO_WritePin(TOF_XSHUT_GPIO_Port, TOF_XSHUT_Pin, GPIO_PIN_SET); // Enable XSHUT
    // osDelay(20);
    // //
    // // VL53L0X init for Single Measurement
    // //

    // VL53L0X_WaitDeviceBooted( Dev );
    // VL53L0X_DataInit( Dev );
    // VL53L0X_StaticInit( Dev );
    // VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
    // VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
    // VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);

    // // Enable/Disable Sigma and Signal check
    // VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    // VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    // VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
    // VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
    // VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
    // VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    // VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

    while (1)
    {
        vl53l0x_test();
        // VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);
        // if(RangingData.RangeStatus == 0)
        // {
        //     MessageLen = sprintf((char*)Message, "Measured distance: %i\n\r", RangingData.RangeMilliMeter);
        //     HAL_UART_Transmit(&log_huart, Message, MessageLen, 100);
        // }
    }
}

void tof_thread_init(void)
{
    tof_task_handle = osThreadNew(tof_thread, NULL, &tof_task_attributes);
    if (tof_task_handle == NULL)
    {
        Error_Handler();
    }
}