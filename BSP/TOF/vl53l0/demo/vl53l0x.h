#ifndef __VL53L0X_H
#define __VL53L0X_H

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "stm32f1xx.h"

//vl53l0x������У׼��Ϣ�ṹ�嶨��
typedef __packed struct
{
	uint8_t  adjustok;                    //У׼�ɹ���־��0XAA����У׼;������δУ׼
	uint8_t  isApertureSpads;             //ApertureSpadsֵ
	uint8_t  VhvSettings;                 //VhvSettingsֵ
	uint8_t  PhaseCal;                    //PhaseCalֵ
	uint32_t XTalkCalDistance;            //XTalkCalDistanceֵ
	uint32_t XTalkCompensationRateMegaCps;//XTalkCompensationRateMegaCpsֵ
	uint32_t CalDistanceMilliMeter;       //CalDistanceMilliMeterֵ
	int32_t  OffsetMicroMeter;            //OffsetMicroMeterֵ
	uint32_t refSpadCount;                //refSpadCountֵ
	
}_VL53L0X_adjust;


extern VL53L0X_RangingMeasurementData_t vl53l0x_data;

VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,uint8_t mode);
void vl53l0x_general_test(VL53L0X_Dev_t *dev);
void Show_GenTask_Message(void);

//VL53L0X�������ϵ�Ĭ��IIC��ַΪ0X52
#define VL53L0X_Addr 0x52


//����ģʽ
#define Default_Mode   0// Ĭ��
#define HIGH_ACCURACY  1//�߾���
#define LONG_RANGE     2//������
#define HIGH_SPEED     3//����

#define   LV_XSH_PORT_CLK_ENABLE       __HAL_RCC_GPIOA_CLK_ENABLE
#define   LV_XSH_PORT                  GPIOA
#define   LV_XSH_PIN                   GPIO_PIN_8  

#define   LV_DISABLE(LV_XSH_PORT, LV_XSH_PIN)             HAL_GPIO_WritePin( LV_XSH_PORT ,LV_XSH_PIN ,GPIO_PIN_RESET);
#define   LV_ENABLE(LV_XSH_PORT, LV_XSH_PIN)              HAL_GPIO_WritePin(LV_XSH_PORT ,LV_XSH_PIN ,GPIO_PIN_SET);


//vl53l0xģʽ���ò�����
typedef __packed struct
{
	FixPoint1616_t signalLimit;    //Signal������ֵ 
	FixPoint1616_t sigmaLimit;     //Sigmal������ֵ
	uint32_t timingBudget;         //����ʱ������
	uint8_t preRangeVcselPeriod ;  //VCSEL��������
	uint8_t finalRangeVcselPeriod ;//VCSEL�������ڷ�Χ
	
}mode_data;


extern mode_data Mode_data[];
extern uint8_t AjustOK;

VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev);//��ʼ��vl53l0x
void print_pal_error(VL53L0X_Error Status);//������Ϣ��ӡ
void mode_string(uint8_t mode,char *buf);//ģʽ�ַ�����ʾ
void vl53l0x_test(void);//vl53l0x����
void vl53l0x_reset(VL53L0X_Dev_t *dev);//vl53l0x��λ

void vl53l0x_info(void);//��ȡvl53l0x�豸ID��Ϣ
void One_measurement(uint8_t mode);//��ȡһ�β�����������
void Show_Mode_Message(void);
#endif


