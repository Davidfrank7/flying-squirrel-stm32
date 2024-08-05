#include "app.h"

#ifdef BSP_USE_IMU
extern void imu_thread_init(void);
#endif
#ifdef BSP_USE_TOF
extern void tof_thread_init(void);
#endif

void Main() {

    HAL_UART_Transmit(&log_huart, (uint8_t *)"Flying squirrel\n", 16, 1000);

#ifdef BSP_USE_IMU
    imu_thread_init();
#endif
#ifdef BSP_USE_TOF
    tof_thread_init();
#endif
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
}