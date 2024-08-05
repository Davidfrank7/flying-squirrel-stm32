#ifndef __VL53L0_I2C_H
#define __VL53L0_I2C_H
#include "stm32f1xx.h"
// #include "SysTick/bsp_SysTick.h"


//#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
//#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
//#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
////IO�ڵ�ַӳ��
//#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
//#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
//#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
//#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
//#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
//#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
//#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

//#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
//#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
//#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
//#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
//#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
//#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
//#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
// 
////IO�ڲ���,ֻ�Ե�һ��IO��!
////ȷ��n��ֵС��16!
//#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
//#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

//#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
//#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

//#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
//#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

//#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
//#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

//#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
//#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

//#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
//#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

//#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
//#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����

/* �����дSCL��SDA�ĺ꣬�����Ӵ���Ŀ���ֲ�ԺͿ��Ķ��� */
#if 0	/* �������룺 1 ѡ��GPIO�Ŀ⺯��ʵ��IO��д */
	#define VL_I2C_SCL_1()  GPIO_SetBits( VL5310X_I2C_GPIO_PORT,  VL5310X_I2C_SCL_PIN)		/* SCL = 1 */
	#define VL_I2C_SCL_0()  GPIO_ResetBits( VL5310X_I2C_GPIO_PORT, VL5310X_I2C_SCL_PIN)		/* SCL = 0 */
	
	#define VL_I2C_SDA_1()  GPIO_SetBits( VL5310X_I2C_GPIO_PORT,  VL5310X_I2C_SDA_PIN)		/* SDA = 1 */
	#define VL_I2C_SDA_0()  GPIO_ResetBits( VL5310X_I2C_GPIO_PORT, VL5310X_I2C_SDA_PIN)		/* SDA = 0 */
	
	#define VL_I2C_SDA_READ()  GPIO_ReadInputDataBit( VL5310X_I2C_GPIO_PORT,  VL5310X_I2C_SDA_PIN)	/* ��SDA����״̬ */
#else	/* �����֧ѡ��ֱ�ӼĴ�������ʵ��IO��д */
    /*��ע�⣺����д������IAR��߼����Ż�ʱ���ᱻ�����������Ż� */
	#define VL_I2C_SCL_1()  VL5310X_I2C_GPIO_PORT->BSRR =  VL5310X_I2C_SCL_PIN				/* SCL = 1 */
	#define VL_I2C_SCL_0()  VL5310X_I2C_GPIO_PORT->BRR =  VL5310X_I2C_SCL_PIN			/* SCL = 0 */
	
	#define VL_I2C_SDA_1()  VL5310X_I2C_GPIO_PORT->BSRR =  VL5310X_I2C_SDA_PIN			/* SDA = 1 */
	#define VL_I2C_SDA_0()  VL5310X_I2C_GPIO_PORT->BRR =  VL5310X_I2C_SDA_PIN				/* SDA = 0 */
	
	#define VL_I2C_SDA_READ()  ((VL5310X_I2C_GPIO_PORT->IDR & VL5310X_I2C_SDA_PIN) != 0)	/* ��SDA����״̬ */
#endif




//IO��������
#define VL_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define VL_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}
////IO��������	 
//#define VL_IIC_SCL    PBout(11) 		//SCL
//#define VL_IIC_SDA    PBout(10) 		//SDA	 
//#define VL_READ_SDA   PBin(10) 		    //����SDA 


//״̬
#define STATUS_OK       0x00
#define STATUS_FAIL     0x01

/* ����I2C�������ӵ�GPIO�˿�, �û�ֻ��Ҫ�޸�����4�д��뼴������ı�SCL��SDA������ */
#define VL5310X_I2C_GPIO_PORT				GPIOB			/* GPIO�˿� */
#define VL5310X_I2C_GPIO_CLK_ENABLE	__HAL_RCC_GPIOB_CLK_ENABLE		 		/* GPIO�˿�ʱ�� */
#define VL5310X_I2C_SCL_PIN					GPIO_PIN_10	/* ���ӵ�SCLʱ���ߵ�GPIO */
#define VL5310X_I2C_SDA_PIN					GPIO_PIN_11		/* ���ӵ�SDA�����ߵ�GPIO */

///IIC��������
void VL53L0X_i2c_init(void);//��ʼ��IIC��IO��

uint8_t VL53L0X_write_byte(uint8_t address,uint8_t index,uint8_t data);              //IICдһ��8λ����
uint8_t VL53L0X_write_word(uint8_t address,uint8_t index,uint16_t data);             //IICдһ��16λ����
uint8_t VL53L0X_write_dword(uint8_t address,uint8_t index,uint32_t data);            //IICдһ��32λ����
uint8_t VL53L0X_write_multi(uint8_t address, uint8_t index,uint8_t *pdata,uint16_t count);//IIC����д

uint8_t VL53L0X_read_byte(uint8_t address,uint8_t index,uint8_t *pdata);             //IIC��һ��8λ����
uint8_t VL53L0X_read_word(uint8_t address,uint8_t index,uint16_t *pdata);            //IIC��һ��16λ����
uint8_t VL53L0X_read_dword(uint8_t address,uint8_t index,uint32_t *pdata);           //IIC��һ��32λ����
uint8_t VL53L0X_read_multi(uint8_t address,uint8_t index,uint8_t *pdata,uint16_t count);  //IIC������

#endif 


