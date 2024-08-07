#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "elog.h"


void board_init(void);
void app_init(void);
void Main(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
