#include "vl53l0x_i2c.h"

/**
 * @brief  VL53L0X I2C��ʼ��
 * @param  ��
 * @retval ��
 */
void VL53L0X_i2c_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* ʹ��GPIOʱ�� */
    VL5310X_I2C_GPIO_CLK_ENABLE();
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pin = VL5310X_I2C_SCL_PIN | VL5310X_I2C_SDA_PIN;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(VL5310X_I2C_GPIO_PORT, &GPIO_InitStructure);
}

/**
 * @brief  ����IIC��ʼ�ź�
 * @param  ��
 * @retval ��
 */
void VL_IIC_Start(void)
{
    VL_SDA_OUT(); /*sda�����*/

    VL_I2C_SDA_1();
    VL_I2C_SCL_1();
    delay_us(4);

    VL_I2C_SDA_0(); /*START:when CLK is high,DATA change form high to low*/
    delay_us(4);

    VL_I2C_SCL_0(); /*ǯסI2C���ߣ�׼�����ͻ��������*/
}

/**
 * @brief  ����IICֹͣ�ź�
 * @param  ��
 * @retval ��
 */
void VL_IIC_Stop(void)
{
    VL_SDA_OUT(); /*sda�����*/

    VL_I2C_SCL_0();
    VL_I2C_SDA_0(); /*STOP:when CLK is high DATA change form low to high*/
    delay_us(4);

    VL_I2C_SCL_1();
    VL_I2C_SDA_1(); /*����I2C���߽����ź�*/

    delay_us(4);
}

/**
 * @brief  �ȴ�Ӧ���źŵ���
 * @param  ��
 * @retval 1������Ӧ��ʧ��
 * @retval 0������Ӧ��ɹ�
 */
uint8_t VL_IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;

    VL_SDA_IN(); /*SDA����Ϊ����*/

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
    VL_I2C_SCL_0(); /*ʱ�����0 */
    return 0;
}

/**
 * @brief  ����ACKӦ��
 * @param  ��
 * @retval ��
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
 * @brief  ������ACKӦ��
 * @param  ��
 * @retval ��
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
 * @brief  IIC����һ���ֽ�
 * @param  Ҫ���������
 * @retval 1����Ӧ��0����Ӧ��
 */
void VL_IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;

    VL_SDA_OUT();
    VL_I2C_SCL_0(); /*����ʱ�ӿ�ʼ���ݴ���*/

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
 * @brief  ��1���ֽ�
 * @param  ack=1ʱ������ACK
 * @param  ack=0ʱ������nACK
 * @retval ��
 */
uint8_t VL_IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;

    VL_SDA_IN(); /*SDA����Ϊ����*/

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
        VL_IIC_NAck(); /*����nACK*/

    else
        VL_IIC_Ack(); /*����ACK */

    return receive;
}

/**
 * @brief  IICдһ���ֽ�����
 * @param  SlaveAddress���ӻ���ַ
 * @param  REG���Ĵ�����ַ
 * @param  REG_data��д����
 * @retval Ӧ��״̬��Ϣ
 */
uint8_t VL_IIC_Write_1Byte(uint8_t SlaveAddress,
                           uint8_t REG_Address,
                           uint8_t REG_data)
{
    VL_IIC_Start();

    VL_IIC_Send_Byte(SlaveAddress);
    if (VL_IIC_Wait_Ack())
    {
        VL_IIC_Stop(); /*�ͷ�����*/
        return 1;      /*ûӦ�����˳�*/
    }
    VL_IIC_Send_Byte(REG_Address);
    VL_IIC_Wait_Ack();

    VL_IIC_Send_Byte(REG_data);
    VL_IIC_Wait_Ack();

    VL_IIC_Stop();

    return 0;
}

/**
 * @brief  IIC��һ���ֽ�����
 * @param  SlaveAddress���ӻ���ַ
 * @param  REG���Ĵ�����ַ
 * @param  REG_data��д����
 * @retval Ӧ��״̬��Ϣ
 */
uint8_t VL_IIC_Read_1Byte(uint8_t SlaveAddress,
                          uint8_t REG_Address,
                          uint8_t *REG_data)
{
    VL_IIC_Start();

    VL_IIC_Send_Byte(SlaveAddress); /*��д����*/
    if (VL_IIC_Wait_Ack())
    {
        VL_IIC_Stop(); /*�ͷ�����*/
        return 1;      /*ûӦ�����˳�*/
    }
    VL_IIC_Send_Byte(REG_Address);
    VL_IIC_Wait_Ack();

    VL_IIC_Start();
    VL_IIC_Send_Byte(SlaveAddress | 0x01); /*��������*/
    VL_IIC_Wait_Ack();

    *REG_data = VL_IIC_Read_Byte(0);
    VL_IIC_Stop();

    return 0;
}

/**
 * @brief  IICдn�ֽ�����
 * @param  SlaveAddress���ӻ���ַ
 * @param  REG���Ĵ�����ַ
 * @param  len�����ݳ���
 * @param  *buf������ָ��
 * @retval Ӧ��״̬��Ϣ
 */
uint8_t VL_IIC_Write_nByte(uint8_t SlaveAddress,
                           uint8_t REG_Address,
                           uint16_t len,
                           uint8_t *buf)
{
    VL_IIC_Start();
    VL_IIC_Send_Byte(SlaveAddress); /*��д����*/

    if (VL_IIC_Wait_Ack())
    {
        VL_IIC_Stop(); /*�ͷ�����*/
        return 1;      /*ûӦ�����˳�*/
    }

    VL_IIC_Send_Byte(REG_Address);
    VL_IIC_Wait_Ack();

    while (len--)
    {
        VL_IIC_Send_Byte(*buf++); /*����buff������*/
        VL_IIC_Wait_Ack();
    }
    VL_IIC_Stop(); /*�ͷ�����*/

    return 0;
}

/**
 * @brief  IIC��n�ֽ�����
 * @param  SlaveAddress���ӻ���ַ
 * @param  REG���Ĵ�����ַ
 * @param  len�����ݳ���
 * @param  *buf������ָ��
 * @retval Ӧ��״̬��Ϣ
 */
uint8_t VL_IIC_Read_nByte(uint8_t SlaveAddress,
                          uint8_t REG_Address,
                          uint16_t len,
                          uint8_t *buf)
{
    VL_IIC_Start();
    VL_IIC_Send_Byte(SlaveAddress); /*��д����*/

    if (VL_IIC_Wait_Ack())
    {
        VL_IIC_Stop(); /*�ͷ�����*/
        return 1;      /*ûӦ�����˳�*/
    }

    VL_IIC_Send_Byte(REG_Address);
    VL_IIC_Wait_Ack();

    VL_IIC_Start();
    VL_IIC_Send_Byte(SlaveAddress | 0x01); /*��������*/
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
    VL_IIC_Stop(); /*�ͷ�����*/

    return 0;
}

/**
 * @brief  VL53L0X д�������
 * @param  address:��ַ
 * @param  index:ƫ�Ƶ�ַ
 * @param  pdata:����ָ��
 * @param  count:���� ���65535
 * @retval Ӧ��״̬��Ϣ
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
 * @brief  VL53L0X ���������
 * @param  address:��ַ
 * @param  index:ƫ�Ƶ�ַ
 * @param  pdata:����ָ��
 * @param  count:���� ���65535
 * @retval Ӧ��״̬��Ϣ
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
 * @brief  д1������(���ֽ�)
 * @param  address:��ַ
 * @param  index:ƫ�Ƶ�ַ
 * @param  data:����(8λ)
 * @retval Ӧ��״̬��Ϣ
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
 * @brief  д1������(˫�ֽ�)
 * @param  address:��ַ
 * @param  index:ƫ�Ƶ�ַ
 * @param  data:����(8λ)
 * @retval Ӧ��״̬��Ϣ
 */
uint8_t VL53L0X_write_word(uint8_t address,
                           uint8_t index,
                           uint16_t data)
{
    uint8_t status = STATUS_OK;
    uint8_t buffer[2];

    /*��16λ���ݲ�ֳ�8λ*/
    buffer[0] = (uint8_t)(data >> 8);   /*�߰�λ*/
    buffer[1] = (uint8_t)(data & 0xff); /*�Ͱ�λ*/

    if (index % 2 == 1)
    {
        /*����ͨ�Ų��ܴ���Է�2�ֽڶ���Ĵ������ֽ�*/
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
 * @brief  д����(4���ֽ�)
 * @param  address:��ַ
 * @param  index:ƫ�Ƶ�ַ
 * @param  data:����(8λ)
 * @retval Ӧ��״̬��Ϣ
 */
uint8_t VL53L0X_write_dword(uint8_t address,
                            uint8_t index,
                            uint32_t data)
{
    uint8_t status = STATUS_OK;
    uint8_t buffer[4];

    /*��32λ���ݲ�ֳ�8λ*/
    buffer[0] = (uint8_t)(data >> 24);
    buffer[1] = (uint8_t)((data & 0xff0000) >> 16);
    buffer[2] = (uint8_t)((data & 0xff00) >> 8);
    buffer[3] = (uint8_t)(data & 0xff);

    status = VL53L0X_write_multi(address, index, buffer, 4);

    return status;
}

/**
 * @brief  ������(1���ֽ�)
 * @param  address:��ַ
 * @param  index:ƫ�Ƶ�ַ
 * @param  data:����(8λ)
 * @retval Ӧ��״̬��Ϣ
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
 * @brief  ������(2���ֽ�)
 * @param  address:��ַ
 * @param  index:ƫ�Ƶ�ַ
 * @param  data:����(8λ)
 * @retval Ӧ��״̬��Ϣ
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
 * @brief  ������(4���ֽ�)
 * @param  address:��ַ
 * @param  index:ƫ�Ƶ�ַ
 * @param  data:����(8λ)
 * @retval Ӧ��״̬��Ϣ
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
        VL_IIC_Stop(); /*�ͷ�����*/
        return 1;      /*ûӦ�����˳�*/
    }

    VL_IIC_Stop();

    return 0;
}
