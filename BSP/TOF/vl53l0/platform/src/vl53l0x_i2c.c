#include "vl53l0x_i2c.h"

/**
 * @brief  VL53L0X I2C初始化
 * @param  无
 * @retval 无
 */
void VL53L0X_i2c_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能GPIO时钟 */
    VL5310X_I2C_GPIO_CLK_ENABLE();
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pin = VL5310X_I2C_SCL_PIN | VL5310X_I2C_SDA_PIN;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(VL5310X_I2C_GPIO_PORT, &GPIO_InitStructure);
}

/**
 * @brief  产生IIC起始信号
 * @param  无
 * @retval 无
 */
void VL_IIC_Start(void)
{
    VL_SDA_OUT(); /*sda线输出*/

    VL_I2C_SDA_1();
    VL_I2C_SCL_1();
    delay_us(4);

    VL_I2C_SDA_0(); /*START:when CLK is high,DATA change form high to low*/
    delay_us(4);

    VL_I2C_SCL_0(); /*钳住I2C总线，准备发送或接收数据*/
}

/**
 * @brief  产生IIC停止信号
 * @param  无
 * @retval 无
 */
void VL_IIC_Stop(void)
{
    VL_SDA_OUT(); /*sda线输出*/

    VL_I2C_SCL_0();
    VL_I2C_SDA_0(); /*STOP:when CLK is high DATA change form low to high*/
    delay_us(4);

    VL_I2C_SCL_1();
    VL_I2C_SDA_1(); /*发送I2C总线结束信号*/

    delay_us(4);
}

/**
 * @brief  等待应答信号到来
 * @param  无
 * @retval 1，接收应答失败
 * @retval 0，接收应答成功
 */
uint8_t VL_IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;

    VL_SDA_IN(); /*SDA设置为输入*/

    VL_I2C_SDA_1();
    delay_us(1);
    VL_I2C_SCL_1();
    delay_us(1);

    while (VL_I2C_SDA_READ())
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            VL_IIC_Stop();
            return 1;
        }
    }
    VL_I2C_SCL_0(); /*时钟输出0 */
    return 0;
}

/**
 * @brief  产生ACK应答
 * @param  无
 * @retval 无
 */
void VL_IIC_Ack(void)
{
    VL_I2C_SCL_0();
    VL_SDA_OUT();

    VL_I2C_SDA_0();
    delay_us(2);

    VL_I2C_SCL_1();
    delay_us(2);

    VL_I2C_SCL_0();
}

/**
 * @brief  不产生ACK应答
 * @param  无
 * @retval 无
 */
void VL_IIC_NAck(void)
{
    VL_I2C_SCL_0();
    VL_SDA_OUT();

    VL_I2C_SDA_1();
    delay_us(2);

    VL_I2C_SCL_1();
    delay_us(2);

    VL_I2C_SCL_0();
}

/**
 * @brief  IIC发送一个字节
 * @param  要传输的数据
 * @retval 1：有应答，0：无应答
 */
void VL_IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;

    VL_SDA_OUT();
    VL_I2C_SCL_0(); /*拉低时钟开始数据传输*/

    for (t = 0; t < 8; t++)
    {
        if ((txd & 0x80) >> 7)
            VL_I2C_SDA_1();
        else
            VL_I2C_SDA_0();
        txd <<= 1;
        delay_us(2);

        VL_I2C_SCL_1();
        delay_us(2);

        VL_I2C_SCL_0();
        delay_us(2);
    }
}

/**
 * @brief  读1个字节
 * @param  ack=1时，发送ACK
 * @param  ack=0时，发送nACK
 * @retval 无
 */
uint8_t VL_IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;

    VL_SDA_IN(); /*SDA设置为输入*/

    for (i = 0; i < 8; i++)
    {
        VL_I2C_SCL_0();
        delay_us(4);
        VL_I2C_SCL_1();
        receive <<= 1;
        if (VL_I2C_SDA_READ())
            receive++;
        delay_us(4);
    }

    if (!ack)
        VL_IIC_NAck(); /*发送nACK*/

    else
        VL_IIC_Ack(); /*发送ACK */

    return receive;
}

/**
 * @brief  IIC写一个字节数据
 * @param  SlaveAddress：从机地址
 * @param  REG：寄存器地址
 * @param  REG_data：写数据
 * @retval 应答状态信息
 */
uint8_t VL_IIC_Write_1Byte(uint8_t SlaveAddress,
                           uint8_t REG_Address,
                           uint8_t REG_data)
{
    VL_IIC_Start();

    VL_IIC_Send_Byte(SlaveAddress);
    if (VL_IIC_Wait_Ack())
    {
        VL_IIC_Stop(); /*释放总线*/
        return 1;      /*没应答则退出*/
    }
    VL_IIC_Send_Byte(REG_Address);
    VL_IIC_Wait_Ack();

    VL_IIC_Send_Byte(REG_data);
    VL_IIC_Wait_Ack();

    VL_IIC_Stop();

    return 0;
}

/**
 * @brief  IIC读一个字节数据
 * @param  SlaveAddress：从机地址
 * @param  REG：寄存器地址
 * @param  REG_data：写数据
 * @retval 应答状态信息
 */
uint8_t VL_IIC_Read_1Byte(uint8_t SlaveAddress,
                          uint8_t REG_Address,
                          uint8_t *REG_data)
{
    VL_IIC_Start();

    VL_IIC_Send_Byte(SlaveAddress); /*发写命令*/
    if (VL_IIC_Wait_Ack())
    {
        VL_IIC_Stop(); /*释放总线*/
        return 1;      /*没应答则退出*/
    }
    VL_IIC_Send_Byte(REG_Address);
    VL_IIC_Wait_Ack();

    VL_IIC_Start();
    VL_IIC_Send_Byte(SlaveAddress | 0x01); /*发读命令*/
    VL_IIC_Wait_Ack();

    *REG_data = VL_IIC_Read_Byte(0);
    VL_IIC_Stop();

    return 0;
}

/**
 * @brief  IIC写n字节数据
 * @param  SlaveAddress：从机地址
 * @param  REG：寄存器地址
 * @param  len：数据长度
 * @param  *buf：数据指针
 * @retval 应答状态信息
 */
uint8_t VL_IIC_Write_nByte(uint8_t SlaveAddress,
                           uint8_t REG_Address,
                           uint16_t len,
                           uint8_t *buf)
{
    VL_IIC_Start();
    VL_IIC_Send_Byte(SlaveAddress); /*发写命令*/

    if (VL_IIC_Wait_Ack())
    {
        VL_IIC_Stop(); /*释放总线*/
        return 1;      /*没应答则退出*/
    }

    VL_IIC_Send_Byte(REG_Address);
    VL_IIC_Wait_Ack();

    while (len--)
    {
        VL_IIC_Send_Byte(*buf++); /*发送buff的数据*/
        VL_IIC_Wait_Ack();
    }
    VL_IIC_Stop(); /*释放总线*/

    return 0;
}

/**
 * @brief  IIC读n字节数据
 * @param  SlaveAddress：从机地址
 * @param  REG：寄存器地址
 * @param  len：数据长度
 * @param  *buf：数据指针
 * @retval 应答状态信息
 */
uint8_t VL_IIC_Read_nByte(uint8_t SlaveAddress,
                          uint8_t REG_Address,
                          uint16_t len,
                          uint8_t *buf)
{
    VL_IIC_Start();
    VL_IIC_Send_Byte(SlaveAddress); /*发写命令*/

    if (VL_IIC_Wait_Ack())
    {
        VL_IIC_Stop(); /*释放总线*/
        return 1;      /*没应答则退出*/
    }

    VL_IIC_Send_Byte(REG_Address);
    VL_IIC_Wait_Ack();

    VL_IIC_Start();
    VL_IIC_Send_Byte(SlaveAddress | 0x01); /*发读命令*/
    VL_IIC_Wait_Ack();

    while (len)
    {
        if (len == 1)
        {
            *buf = VL_IIC_Read_Byte(0);
        }
        else
        {
            *buf = VL_IIC_Read_Byte(1);
        }
        buf++;
        len--;
    }
    VL_IIC_Stop(); /*释放总线*/

    return 0;
}

/**
 * @brief  VL53L0X 写多个数据
 * @param  address:地址
 * @param  index:偏移地址
 * @param  pdata:数据指针
 * @param  count:长度 最大65535
 * @retval 应答状态信息
 */
uint8_t VL53L0X_write_multi(uint8_t address,
                            uint8_t index,
                            uint8_t *pdata,
                            uint16_t count)
{
    uint8_t status = STATUS_OK;

    if (VL_IIC_Write_nByte(address, index, count, pdata))
    {
        status = STATUS_FAIL;
    }

    return status;
}

/**
 * @brief  VL53L0X 读多个数据
 * @param  address:地址
 * @param  index:偏移地址
 * @param  pdata:数据指针
 * @param  count:长度 最大65535
 * @retval 应答状态信息
 */
uint8_t VL53L0X_read_multi(uint8_t address,
                           uint8_t index,
                           uint8_t *pdata,
                           uint16_t count)
{
    uint8_t status = STATUS_OK;

    if (VL_IIC_Read_nByte(address, index, count, pdata))
    {
        status = STATUS_FAIL;
    }

    return status;
}

/**
 * @brief  写1个数据(单字节)
 * @param  address:地址
 * @param  index:偏移地址
 * @param  data:数据(8位)
 * @retval 应答状态信息
 */
uint8_t VL53L0X_write_byte(uint8_t address,
                           uint8_t index,
                           uint8_t data)
{
    uint8_t status = STATUS_OK;

    status = VL53L0X_write_multi(address, index, &data, 1);

    return status;
}

/**
 * @brief  写1个数据(双字节)
 * @param  address:地址
 * @param  index:偏移地址
 * @param  data:数据(8位)
 * @retval 应答状态信息
 */
uint8_t VL53L0X_write_word(uint8_t address,
                           uint8_t index,
                           uint16_t data)
{
    uint8_t status = STATUS_OK;
    uint8_t buffer[2];

    /*将16位数据拆分成8位*/
    buffer[0] = (uint8_t)(data >> 8);   /*高八位*/
    buffer[1] = (uint8_t)(data & 0xff); /*低八位*/

    if (index % 2 == 1)
    {
        /*串行通信不能处理对非2字节对齐寄存器的字节*/
        status = VL53L0X_write_multi(address, index, &buffer[0], 1);
        status = VL53L0X_write_multi(address, index, &buffer[0], 1);
    }
    else
    {
        status = VL53L0X_write_multi(address, index, buffer, 2);
    }

    return status;
}

/**
 * @brief  写数据(4个字节)
 * @param  address:地址
 * @param  index:偏移地址
 * @param  data:数据(8位)
 * @retval 应答状态信息
 */
uint8_t VL53L0X_write_dword(uint8_t address,
                            uint8_t index,
                            uint32_t data)
{
    uint8_t status = STATUS_OK;
    uint8_t buffer[4];

    /*将32位数据拆分成8位*/
    buffer[0] = (uint8_t)(data >> 24);
    buffer[1] = (uint8_t)((data & 0xff0000) >> 16);
    buffer[2] = (uint8_t)((data & 0xff00) >> 8);
    buffer[3] = (uint8_t)(data & 0xff);

    status = VL53L0X_write_multi(address, index, buffer, 4);

    return status;
}

/**
 * @brief  读数据(1个字节)
 * @param  address:地址
 * @param  index:偏移地址
 * @param  data:数据(8位)
 * @retval 应答状态信息
 */
uint8_t VL53L0X_read_byte(uint8_t address,
                          uint8_t index,
                          uint8_t *pdata)
{
    uint8_t status = STATUS_OK;

    status = VL53L0X_read_multi(address, index, pdata, 1);

    return status;
}

/**
 * @brief  读数据(2个字节)
 * @param  address:地址
 * @param  index:偏移地址
 * @param  data:数据(8位)
 * @retval 应答状态信息
 */
uint8_t VL53L0X_read_word(uint8_t address,
                          uint8_t index,
                          uint16_t *pdata)
{
    uint8_t status = STATUS_OK;
    uint8_t buffer[2];

    status = VL53L0X_read_multi(address, index, buffer, 2);

    *pdata = ((uint16_t)buffer[0] << 8) + (uint16_t)buffer[1];

    return status;
}

/**
 * @brief  读数据(4个字节)
 * @param  address:地址
 * @param  index:偏移地址
 * @param  data:数据(8位)
 * @retval 应答状态信息
 */
uint8_t VL53L0X_read_dword(uint8_t address,
                           uint8_t index,
                           uint32_t *pdata)
{
    uint8_t status = STATUS_OK;
    uint8_t buffer[4];

    status = VL53L0X_read_multi(address, index, buffer, 4);

    *pdata = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) +
             ((uint32_t)buffer[2] << 8) + ((uint32_t)buffer[3]);

    return status;
}

uint8_t IIC_Cheak(void)
{
    VL_IIC_Start();

    VL_IIC_Send_Byte(0X52);
    if (VL_IIC_Wait_Ack())
    {
        VL_IIC_Stop(); /*释放总线*/
        return 1;      /*没应答则退出*/
    }

    VL_IIC_Stop();

    return 0;
}
