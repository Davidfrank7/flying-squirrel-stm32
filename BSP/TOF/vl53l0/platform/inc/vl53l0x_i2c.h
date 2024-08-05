#ifndef __VL53L0_I2C_H
#define __VL53L0_I2C_H
#include "stm32f1xx.h"
// #include "SysTick/bsp_SysTick.h"


//#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
//#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
//#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
////IO口地址映射
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
////IO口操作,只对单一的IO口!
////确保n的值小于16!
//#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
//#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

//#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
//#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

//#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
//#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

//#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
//#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

//#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
//#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

//#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
//#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

//#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
//#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

/* 定义读写SCL和SDA的宏，已增加代码的可移植性和可阅读性 */
#if 0	/* 条件编译： 1 选择GPIO的库函数实现IO读写 */
	#define VL_I2C_SCL_1()  GPIO_SetBits( VL5310X_I2C_GPIO_PORT,  VL5310X_I2C_SCL_PIN)		/* SCL = 1 */
	#define VL_I2C_SCL_0()  GPIO_ResetBits( VL5310X_I2C_GPIO_PORT, VL5310X_I2C_SCL_PIN)		/* SCL = 0 */
	
	#define VL_I2C_SDA_1()  GPIO_SetBits( VL5310X_I2C_GPIO_PORT,  VL5310X_I2C_SDA_PIN)		/* SDA = 1 */
	#define VL_I2C_SDA_0()  GPIO_ResetBits( VL5310X_I2C_GPIO_PORT, VL5310X_I2C_SDA_PIN)		/* SDA = 0 */
	
	#define VL_I2C_SDA_READ()  GPIO_ReadInputDataBit( VL5310X_I2C_GPIO_PORT,  VL5310X_I2C_SDA_PIN)	/* 读SDA口线状态 */
#else	/* 这个分支选择直接寄存器操作实现IO读写 */
    /*　注意：如下写法，在IAR最高级别优化时，会被编译器错误优化 */
	#define VL_I2C_SCL_1()  VL5310X_I2C_GPIO_PORT->BSRR =  VL5310X_I2C_SCL_PIN				/* SCL = 1 */
	#define VL_I2C_SCL_0()  VL5310X_I2C_GPIO_PORT->BRR =  VL5310X_I2C_SCL_PIN			/* SCL = 0 */
	
	#define VL_I2C_SDA_1()  VL5310X_I2C_GPIO_PORT->BSRR =  VL5310X_I2C_SDA_PIN			/* SDA = 1 */
	#define VL_I2C_SDA_0()  VL5310X_I2C_GPIO_PORT->BRR =  VL5310X_I2C_SDA_PIN				/* SDA = 0 */
	
	#define VL_I2C_SDA_READ()  ((VL5310X_I2C_GPIO_PORT->IDR & VL5310X_I2C_SDA_PIN) != 0)	/* 读SDA口线状态 */
#endif




//IO方向设置
#define VL_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define VL_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}
////IO操作函数	 
//#define VL_IIC_SCL    PBout(11) 		//SCL
//#define VL_IIC_SDA    PBout(10) 		//SDA	 
//#define VL_READ_SDA   PBin(10) 		    //输入SDA 


//状态
#define STATUS_OK       0x00
#define STATUS_FAIL     0x01

/* 定义I2C总线连接的GPIO端口, 用户只需要修改下面4行代码即可任意改变SCL和SDA的引脚 */
#define VL5310X_I2C_GPIO_PORT				GPIOB			/* GPIO端口 */
#define VL5310X_I2C_GPIO_CLK_ENABLE	__HAL_RCC_GPIOB_CLK_ENABLE		 		/* GPIO端口时钟 */
#define VL5310X_I2C_SCL_PIN					GPIO_PIN_10	/* 连接到SCL时钟线的GPIO */
#define VL5310X_I2C_SDA_PIN					GPIO_PIN_11		/* 连接到SDA数据线的GPIO */

///IIC操作函数
void VL53L0X_i2c_init(void);//初始化IIC的IO口

uint8_t VL53L0X_write_byte(uint8_t address,uint8_t index,uint8_t data);              //IIC写一个8位数据
uint8_t VL53L0X_write_word(uint8_t address,uint8_t index,uint16_t data);             //IIC写一个16位数据
uint8_t VL53L0X_write_dword(uint8_t address,uint8_t index,uint32_t data);            //IIC写一个32位数据
uint8_t VL53L0X_write_multi(uint8_t address, uint8_t index,uint8_t *pdata,uint16_t count);//IIC连续写

uint8_t VL53L0X_read_byte(uint8_t address,uint8_t index,uint8_t *pdata);             //IIC读一个8位数据
uint8_t VL53L0X_read_word(uint8_t address,uint8_t index,uint16_t *pdata);            //IIC读一个16位数据
uint8_t VL53L0X_read_dword(uint8_t address,uint8_t index,uint32_t *pdata);           //IIC读一个32位数据
uint8_t VL53L0X_read_multi(uint8_t address,uint8_t index,uint8_t *pdata,uint16_t count);  //IIC连续读

#endif 


