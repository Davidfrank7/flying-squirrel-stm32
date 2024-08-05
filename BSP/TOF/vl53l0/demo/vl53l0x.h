#ifndef __VL53L0X_H
#define __VL53L0X_H

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "stm32f1xx.h"

//vl53l0x传感器校准信息结构体定义
typedef __packed struct
{
	uint8_t  adjustok;                    //校准成功标志，0XAA，已校准;其他，未校准
	uint8_t  isApertureSpads;             //ApertureSpads值
	uint8_t  VhvSettings;                 //VhvSettings值
	uint8_t  PhaseCal;                    //PhaseCal值
	uint32_t XTalkCalDistance;            //XTalkCalDistance值
	uint32_t XTalkCompensationRateMegaCps;//XTalkCompensationRateMegaCps值
	uint32_t CalDistanceMilliMeter;       //CalDistanceMilliMeter值
	int32_t  OffsetMicroMeter;            //OffsetMicroMeter值
	uint32_t refSpadCount;                //refSpadCount值
	
}_VL53L0X_adjust;


extern VL53L0X_RangingMeasurementData_t vl53l0x_data;

VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,uint8_t mode);
void vl53l0x_general_test(VL53L0X_Dev_t *dev);
void Show_GenTask_Message(void);

//VL53L0X传感器上电默认IIC地址为0X52
#define VL53L0X_Addr 0x52


//测量模式
#define Default_Mode   0// 默认
#define HIGH_ACCURACY  1//高精度
#define LONG_RANGE     2//长距离
#define HIGH_SPEED     3//高速

#define   LV_XSH_PORT_CLK_ENABLE       __HAL_RCC_GPIOA_CLK_ENABLE
#define   LV_XSH_PORT                  GPIOA
#define   LV_XSH_PIN                   GPIO_PIN_8  

#define   LV_DISABLE(LV_XSH_PORT, LV_XSH_PIN)             HAL_GPIO_WritePin( LV_XSH_PORT ,LV_XSH_PIN ,GPIO_PIN_RESET);
#define   LV_ENABLE(LV_XSH_PORT, LV_XSH_PIN)              HAL_GPIO_WritePin(LV_XSH_PORT ,LV_XSH_PIN ,GPIO_PIN_SET);


//vl53l0x模式配置参数集
typedef __packed struct
{
	FixPoint1616_t signalLimit;    //Signal极限数值 
	FixPoint1616_t sigmaLimit;     //Sigmal极限数值
	uint32_t timingBudget;         //采样时间周期
	uint8_t preRangeVcselPeriod ;  //VCSEL脉冲周期
	uint8_t finalRangeVcselPeriod ;//VCSEL脉冲周期范围
	
}mode_data;


extern mode_data Mode_data[];
extern uint8_t AjustOK;

VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev);//初始化vl53l0x
void print_pal_error(VL53L0X_Error Status);//错误信息打印
void mode_string(uint8_t mode,char *buf);//模式字符串显示
void vl53l0x_test(void);//vl53l0x测试
void vl53l0x_reset(VL53L0X_Dev_t *dev);//vl53l0x复位

void vl53l0x_info(void);//获取vl53l0x设备ID信息
void One_measurement(uint8_t mode);//获取一次测量距离数据
void Show_Mode_Message(void);
#endif


