/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2015, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2015-04-28
 */
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "usart.h"
#include <elog.h>
#include <stdio.h>
//! log_uart互斥信号量句柄
osSemaphoreId_t log_uart_mutex;

/**
 * EasyLogger port initialize
 *
 * @return result
 */
ElogErrCode elog_port_init(void) {
  ElogErrCode result = ELOG_NO_ERR;

  /* add your code here */
  //! 创建互斥信号值
  log_uart_mutex = osSemaphoreNew(1, 1, NULL);
  // log_uart_mutex = xSemaphoreCreateMutex();
  if (log_uart_mutex == NULL) {
    printf("elog sem create fail\r\n");
    //result = ELOG_SEM_FAIL ;//!< 注意：ElogErrCode
    }

  return result;
}

/**
 * EasyLogger port deinitialize
 *
 */
void elog_port_deinit(void) { /* add your code here */ }

/**
 * output log port interface
 *
 * @param log output of log
 * @param size log size
 */
void elog_port_output(const char *log, size_t size) {

  /* add your code here */
  //! %s 表示字符串输出，
  //! .<十进制数> 是精度控制格式符，输出字符时表示输出字符的位数，
  //! 在精度控制时，小数点后的十进制数可以使用 * 来占位，
  //! 在后面提供一个变量作为精度控制的具体值
  HAL_UART_Transmit(&huart1, (uint8_t *)log, size, HAL_MAX_DELAY);
}

/**
 * output lock
 */
void elog_port_output_lock(void) {

  /* add your code here */
  if (NULL != log_uart_mutex) {
    // xSemaphoreTake(log_uart_mutex, portMAX_DELAY); //!< 等待互斥信号量
    osSemaphoreAcquire(log_uart_mutex, osWaitForever);
  }
}

/**
 * output unlock
 */
void elog_port_output_unlock(void) {

  /* add your code here */
  if (NULL != log_uart_mutex) {
    // xSemaphoreGive(log_uart_mutex); //!< 发送互斥信号量
    osSemaphoreRelease(log_uart_mutex);
  }
}

/**
 * get current time interface
 *
 * @return current time
 */
const char *elog_port_get_time(void) {

  /* add your code here */
  static char cur_system_time[16] = {0};

#if (INCLUDE_xTaskGetSchedulerState == 1)
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
#endif
    TickType_t tick = xTaskGetTickCount();
    snprintf(cur_system_time, 16, "%ld.%.3ld", (tick / configTICK_RATE_HZ),
             tick % configTICK_RATE_HZ);
#if (INCLUDE_xTaskGetSchedulerState == 1)
  }
#endif

  return cur_system_time;
}

/**
 * get current process name interface
 *
 * @return current process name
 */
const char *elog_port_get_p_info(void) {

  /* add your code here */
  return "";
}

/**
 * get current thread name interface
 *
 * @return current thread name
 */
const char *elog_port_get_t_info(void) {

  /* add your code here */
#if (INCLUDE_xTaskGetSchedulerState == 1)
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
#endif
    return pcTaskGetName(xTaskGetCurrentTaskHandle());
#if (INCLUDE_xTaskGetSchedulerState == 1)
  }
#endif

  return "";
}

/**
 * init
 * easylogger,这个函数是我们自己添加的，便于用户直接调用，需要在elog.h中添加声明
 */
void easylogger_init(void) {
  /* init Easylogger */
  elog_init();

  /* set EasyLogger log format */
  elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
  elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TIME | ELOG_FMT_T_INFO);
  elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_T_INFO);
  elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL);
  elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~ELOG_FMT_FUNC);

  /*Eenbale color*/
  elog_set_text_color_enabled(true);

  /* start EasyLogger */
  elog_start();
}
