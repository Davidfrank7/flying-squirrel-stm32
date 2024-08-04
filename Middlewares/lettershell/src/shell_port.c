/**
 * @file shell_port.c
 * @author Letter (NevermindZZT@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-22
 * 
 * @copyright (c) 2019 Letter
 * 
 */

#include "FreeRTOS.h"
#include "semphr.h"
#include "shell.h"
#include "stm32f1xx_hal.h"
#include "task.h"
#include "usart.h"
#include "shell_port.h"

Shell shell;
char shellBuffer[512];

static SemaphoreHandle_t shellMutex;

/**
 * @brief 用户shell写
 * 
 * @param data 数据
 * @param len 数据长度
 * 
 * @return short 实际写入的数据长度
 */
short ShellWrite(char *data, unsigned short len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 0x1FF);
    return len;
}


/**
 * @brief 用户shell读
 * 
 * @param data 数据
 * @param len 数据长度
 * 
 * @return short 实际读取到
 */
short ShellRead(char *data, unsigned short len)
{
  if (HAL_UART_Receive(&huart1, (uint8_t *)data, len, 0x1FF) != HAL_OK) {
    return 0;
  }
  else {
    return 1;
  }
}

/**
 * @brief 用户shell上锁
 * 
 * @param shell shell
 * 
 * @return int 0
 */
int ShellLock(Shell *shell)
{
    xSemaphoreTakeRecursive(shellMutex, portMAX_DELAY);
    return 0;
}

/**
 * @brief 用户shell解锁
 * 
 * @param shell shell
 * 
 * @return int 0
 */
int ShellUnlock(Shell *shell)
{
    xSemaphoreGiveRecursive(shellMutex);
    return 0;
}

/**
 * @brief 用户shell初始化
 * 
 */
void ShellInit(void)
{
    shellMutex = xSemaphoreCreateMutex();

    shell.write = ShellWrite;
    shell.read = ShellRead;
    shell.lock = ShellLock;
    shell.unlock = ShellUnlock;
    shellInit(&shell, shellBuffer, 512);
    // if (xTaskCreate(shellTask, "shell", 256, &shell, 5, NULL) != pdPASS)
    // {
    //     logError("shell task creat failed");
    // }
}
// CEVENT_EXPORT(EVENT_INIT_STAGE2, userShellInit);
