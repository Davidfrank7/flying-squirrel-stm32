#include "app.h"


#ifdef BSP_USE_IMU
extern void imu_thread_init(void);
#endif
#ifdef BSP_USE_TOF
extern void tof_thread_init(void);
#endif

static const char *TAG = "main";

void elog_harware_init(void) {
  /* Definitions for elog */

  elog_init();

  elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL & ~ELOG_FMT_P_INFO);
  elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~(ELOG_FMT_FUNC | ELOG_FMT_P_INFO));
  elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL & ~(ELOG_FMT_FUNC | ELOG_FMT_P_INFO));
}


/**
 * @brief board's hardware init before os init, such as uart, i2c, etc.
 * 
 */
void board_init(void) {
    elog_harware_init();
}

/**
 * @brief app before after os start, add threads, queues, etc.
 * 
 */
void app_init(void) {
    elog_start();
}

void Main() {
    // HAL_UART_Transmit(&log_huart, (uint8_t *)"Flying squirrel\n", 16, 1000);
    elog_i(TAG, "Flying squirrel");

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