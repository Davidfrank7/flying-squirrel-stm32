#include "vl53l0x.h"
#include <stdio.h>

#define  delay_ms(x) HAL_Delay(x)

_VL53L0X_adjust Vl53l0x_adjust;
_VL53L0X_adjust Vl53l0x_data;
VL53L0X_Dev_t vl53l0x_dev;                     /*�豸I2C���ݲ���*/
VL53L0X_DeviceInfo_t vl53l0x_dev_info;         /*�豸ID�汾��Ϣ*/
uint8_t AjustOK = 0;                           /*У׼��־λ*/
VL53L0X_RangingMeasurementData_t vl53l0x_data; /*�������ṹ��*/
volatile uint16_t Distance_data = 0;           /*����������*/

/*0��Ĭ��;1:�߾���;2:������;3:����*/
mode_data Mode_data[] =
    {
        {(FixPoint1616_t)(0.25 * 65536),
         (FixPoint1616_t)(18 * 65536),
         33000,
         14,
         10}, /*Ĭ��*/

        {(FixPoint1616_t)(0.25 * 65536),
         (FixPoint1616_t)(18 * 65536),
         200000,
         14,
         10}, /*�߾���*/

        {(FixPoint1616_t)(0.1 * 65536),
         (FixPoint1616_t)(60 * 65536),
         33000,
         18,
         14}, /*������*/

        {(FixPoint1616_t)(0.25 * 65536),
         (FixPoint1616_t)(32 * 65536),
         20000,
         14,
         10}, /*����*/
};

/**
 * @brief  ��ӡ������Ϣ
 * @param  Status���鿴VL53L0X_Error�����Ķ���
 * @retval ��
 */
void print_pal_error(VL53L0X_Error Status)
{
    char buf[VL53L0X_MAX_STRING_LENGTH];

    /*����Status״̬��ȡ������Ϣ�ַ���*/
    VL53L0X_GetPalErrorString(Status, buf);

    /*��ӡ״̬�ʹ�����Ϣ*/
    printf("API Status: %i : %s\r\n", Status, buf);
}

/**
 * @brief  ����VL53L0X�豸I2C��ַ
 * @param  dev:�豸I2C�����ṹ��
 * @param  Status�����鿴VL53L0X_Error�����Ķ���
 * @param  newaddr:�豸��I2C��ַ
 * @retval ������Ϣ
 */
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev, uint8_t newaddr)
{
    uint16_t Id;
    uint8_t FinalAddress;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t sta = 0x00;

    FinalAddress = newaddr;

    /*���豸I2C��ַ��ɵ�ַһ��,ֱ���˳�*/
    if (FinalAddress == dev->I2cDevAddr)
        return VL53L0X_ERROR_NONE;

    /*�ڽ��е�һ���Ĵ�������֮ǰ����I2C��׼ģʽ(400Khz)*/
    Status = VL53L0X_WrByte(dev, 0x88, 0x00);
    if (Status != VL53L0X_ERROR_NONE)
    {
        /*����I2C��׼ģʽ����*/
        sta = 0x01;
        goto set_error;
    }
    /*����ʹ��Ĭ�ϵ�0x52��ַ��ȡһ���Ĵ���*/
    Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
    if (Status != VL53L0X_ERROR_NONE)
    {
        /*��ȡ�Ĵ�������*/
        sta = 0x02;
        goto set_error;
    }
    if (Id == 0xEEAA)
    {
        /*�����豸�µ�I2C��ַ*/
        Status = VL53L0X_SetDeviceAddress(dev, FinalAddress);
        if (Status != VL53L0X_ERROR_NONE)
        {
            /*����I2C��ַ����*/
            sta = 0x03;
            goto set_error;
        }
        /*�޸Ĳ����ṹ���I2C��ַ*/
        dev->I2cDevAddr = FinalAddress;
        /*����µ�I2C��ַ��д�Ƿ�����*/
        Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
        if (Status != VL53L0X_ERROR_NONE)
        {
            /*��I2C��ַ��д����*/
            sta = 0x04;
            goto set_error;
        }
    }
set_error:
    if (Status != VL53L0X_ERROR_NONE)
    {
        /*��ӡ������Ϣ*/
        print_pal_error(Status);
    }
    if (sta != 0)
        printf("sta:0x%x\r\n", sta);
    return Status;
}

/**
 * @brief  vl53l0x��λ����
 * @param  dev:�豸I2C�����ṹ��
 * @retval ��
 */
void vl53l0x_reset(VL53L0X_Dev_t *dev)
{
    uint8_t addr;

    /*�����豸ԭI2C��ַ*/
    addr = dev->I2cDevAddr;
    /*ʧ��VL53L0X*/
    LV_DISABLE(LV_XSH_PORT, LV_XSH_PIN);
    delay_ms(30);

    /*ʹ��VL53L0X,�ô��������ڹ���(I2C��ַ��ָ�Ĭ��0X52)*/
    LV_ENABLE(LV_XSH_PORT, LV_XSH_PIN);
    delay_ms(30);

    dev->I2cDevAddr = 0x52;

    /*����VL53L0X������ԭ���ϵ�ǰԭI2C��ַ*/
    vl53l0x_Addr_set(dev, addr);
    VL53L0X_DataInit(dev);
}

/**
 * @brief  ��ʼ��vl53l0x
 * @param  dev:�豸I2C�����ṹ��
 * @retval ״̬��Ϣ
 */
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t *pMyDevice = dev;

    GPIO_InitTypeDef GPIO_InitStructure;

    /*��ʼ��XSH���� */
    LV_XSH_PORT_CLK_ENABLE();
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pin = LV_XSH_PIN;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LV_XSH_PORT, &GPIO_InitStructure);

    /*I2C��ַ(�ϵ�Ĭ��0x52)*/
    pMyDevice->I2cDevAddr = VL53L0X_Addr;
    /*I2Cͨ��ģʽ*/
    pMyDevice->comms_type = 1;
    /*I2Cͨ������*/
    pMyDevice->comms_speed_khz = 400;

    /*��ʼ��IIC*/
    VL53L0X_i2c_init();

    /*ʧ��VL53L0X*/
    LV_DISABLE(LV_XSH_PORT, LV_XSH_PIN);
    delay_ms(30);

    /*ʹ��VL53L0X,�ô��������ڹ���*/
    LV_ENABLE(LV_XSH_PORT, LV_XSH_PIN);
    delay_ms(30);

    /*����VL53L0X������I2C��ַ*/
    vl53l0x_Addr_set(pMyDevice, 0x54);
    if (Status != VL53L0X_ERROR_NONE)
        goto error;

    /*�豸��ʼ��*/
    Status = VL53L0X_DataInit(pMyDevice);
    if (Status != VL53L0X_ERROR_NONE)
        goto error;
    delay_ms(2);

    /*��ȡ�豸ID��Ϣ*/
    Status = VL53L0X_GetDeviceInfo(pMyDevice, &vl53l0x_dev_info);
    if (Status != VL53L0X_ERROR_NONE)
        goto error;

    /*��У׼*/
    if (Vl53l0x_data.adjustok == 0xAA)
        AjustOK = 1;
    /*ûУ׼	*/
    else
        AjustOK = 0;

error:
    if (Status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(Status); /*��ӡ������Ϣ*/
        return Status;
    }

    return Status;
}

/**
 * @brief  VL53L0X�����Գ���
 * @param  ��
 * @retval ��
 */
void vl53l0x_test(void)
{
    /*vl53l0x��ʼ��*/
    while (vl53l0x_init(&vl53l0x_dev))
    {
        printf("��ʼ��ʧ��\r\n");
        printf("��������\r\n");
    }
    printf("��ʼ���ɹ�\r\n");

    while (1)
    {
        vl53l0x_general_test(&vl53l0x_dev);
    }
}

/**
 * @brief  VL53L0X ����ģʽ����
 * @param  dev:�豸I2C�����ṹ��
* @param  mode: 0:Ĭ��;1:�߾���;2:������;3:����
 * @retval ״̬��Ϣ
 */
VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev, uint8_t mode)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;

    /*��λvl53l0x(Ƶ���л�����ģʽ���׵��²ɼ��������ݲ�׼���������һ����)*/
    vl53l0x_reset(dev);
    status = VL53L0X_StaticInit(dev);

    /*��У׼����,д��У׼ֵ*/
    if (AjustOK != 0)
    {
        /*�趨SpadsУ׼ֵ*/
        status = VL53L0X_SetReferenceSpads(dev, Vl53l0x_data.refSpadCount,
                                           Vl53l0x_data.isApertureSpads);
        if (status != VL53L0X_ERROR_NONE)
            goto error;
        delay_ms(2);

        /*�趨RefУ׼ֵ*/
        status = VL53L0X_SetRefCalibration(dev, Vl53l0x_data.VhvSettings,
                                           Vl53l0x_data.PhaseCal);
        if (status != VL53L0X_ERROR_NONE)
            goto error;
        delay_ms(2);

        /*�趨ƫ��У׼ֵ*/
        status = VL53L0X_SetOffsetCalibrationDataMicroMeter(dev,
                                                            Vl53l0x_data.OffsetMicroMeter);
        if (status != VL53L0X_ERROR_NONE)
            goto error;
        delay_ms(2);

        /*�趨����У׼ֵ*/
        status = VL53L0X_SetXTalkCompensationRateMegaCps(dev,
                                                         Vl53l0x_data.XTalkCompensationRateMegaCps);
        if (status != VL53L0X_ERROR_NONE)
            goto error;
        delay_ms(2);
    }
    else
    {
        /*Ref�ο�У׼*/
        status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);
        if (status != VL53L0X_ERROR_NONE)
            goto error;
        delay_ms(2);

        /*ִ�вο�SPAD����*/
        status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount,
                                                  &isApertureSpads);
        if (status != VL53L0X_ERROR_NONE)
            goto error;
        delay_ms(2);
    }

    /*ʹ�ܵ��β���ģʽ*/
    status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE)
        goto error;
    delay_ms(2);

    /*ʹ��SIGMA��Χ���*/
    status = VL53L0X_SetLimitCheckEnable(dev,
                                         VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                         1);
    if (status != VL53L0X_ERROR_NONE)
        goto error;
    delay_ms(2);

    /*ʹ���ź����ʷ�Χ���*/
    status = VL53L0X_SetLimitCheckEnable(dev,
                                         VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                         1);
    if (status != VL53L0X_ERROR_NONE)
        goto error;
    delay_ms(2);

    /*�趨SIGMA��Χ*/
    status = VL53L0X_SetLimitCheckValue(dev,
                                        VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                        Mode_data[mode].sigmaLimit);
    if (status != VL53L0X_ERROR_NONE)
        goto error;
    delay_ms(2);

    /*�趨�ź����ʷ�Χ��Χ*/
    status = VL53L0X_SetLimitCheckValue(dev,
                                        VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                        Mode_data[mode].signalLimit);
    if (status != VL53L0X_ERROR_NONE)
        goto error;
    delay_ms(2);

    /*�趨��������ʱ��*/
    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,
                                                            Mode_data[mode].timingBudget);
    if (status != VL53L0X_ERROR_NONE)
        goto error;
    delay_ms(2);

    /*�趨VCSEL��������*/
    status = VL53L0X_SetVcselPulsePeriod(dev,
                                         VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                         Mode_data[mode].preRangeVcselPeriod);
    if (status != VL53L0X_ERROR_NONE)
        goto error;
    delay_ms(2);

    /*�趨VCSEL�������ڷ�Χ*/
    status = VL53L0X_SetVcselPulsePeriod(dev,
                                         VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
                                         Mode_data[mode].finalRangeVcselPeriod);

error: /*������Ϣ*/
    if (status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(status);
        return status;
    }
    return status;
}

/**
 * @brief  VL53L0X ���ξ����������
 * @param  dev:�豸I2C�����ṹ��
 * @param  pdata:����������ݽṹ��
 * @retval ״̬��Ϣ
 */
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,
                                        VL53L0X_RangingMeasurementData_t *pdata,
                                        char *buf)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t RangeStatus;

    /*ִ�е��β�ಢ��ȡ����������*/
    status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);
    if (status != VL53L0X_ERROR_NONE)
        return status;

    /*��ȡ��ǰ����״̬*/
    RangeStatus = pdata->RangeStatus;
    memset(buf, 0x00, VL53L0X_MAX_STRING_LENGTH);
    /*���ݲ���״̬��ȡ״̬�ַ���*/
    VL53L0X_GetRangeStatusString(RangeStatus, buf);
    /*�������һ�β���������*/
    Distance_data = pdata->RangeMilliMeter;

    return status;
}

/**
 * @brief  ������ͨ����
 * @param  dev���豸I2C�����ṹ��
 * @param  modeģʽ���� 0:Ĭ��;1:�߾���;2:������
 * @retval ��
 */
void vl53l0x_general_start(VL53L0X_Dev_t *dev, uint8_t mode)
{
    /*����ģʽ�ַ����ַ�������*/
    static char buf[VL53L0X_MAX_STRING_LENGTH];
    /*����״̬*/
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t i = 0;

    /*���ò���ģʽ*/
    while (vl53l0x_set_mode(dev, mode))
    {
        printf("ģʽ����ʧ��!!!\r\n");
        delay_ms(500);
        i++;
        if (i == 2)
        {
            return;
        }
    }
    printf("ģʽ���óɹ�!!!\r\n");
    /*ִ��5�β���	*/
    for(int i=0;i<5;i++)
    {
        if (Status == vl53l0x_start_single_test(dev, &vl53l0x_data, buf))
        {
            /*��ӡ��������*/
            printf("��������d: %4imm\r\n", Distance_data);
        } 
        delay_ms(1000);
    }
}

/**
 * @brief  ����ģʽ��Ϣ����
 * @param  ��
 * @retval ��
 */
void Show_GenTask_Message(void)
{
    printf("   ָ��   ------      ���� \r\n");
    printf("    0     ------    Ĭ�ϲ���ģʽ \r\n");
    printf("    1     ------    �߾��Ȳ���ģʽ \r\n");
    printf("    2     ------    ���������ģʽ \r\n");
    printf("    3     ------    ���ٲ���ģʽ \r\n");
    printf("***��������ָ��󣬰��س����ٷ���*** \r\n");
}

/**
 * @brief  vl53l0x��ͨ����ģʽ����
 * @param  dev:�豸I2C�����ṹ��
 * @retval ��
 */
void vl53l0x_general_test(VL53L0X_Dev_t *dev)
{
    uint8_t ch;

    /*��ʾ��ͨ����ģʽ*/
    Show_GenTask_Message();

    while (1)
    {
        /* ��ȡ�ַ�ָ�� */
        ch = getchar();
        printf("���յ��ַ���%c\n", ch);

        switch (ch)
        {
        case '0':
        {
            vl53l0x_general_start(dev, 0);
            Show_GenTask_Message();
        }
        break;

        case '1':
        {
            vl53l0x_general_start(dev, 1);
            Show_GenTask_Message();
        }
        break;

        case '2':
        {
            vl53l0x_general_start(dev, 2);
            Show_GenTask_Message();
        }
        break;

        case '3':
        {
            vl53l0x_general_start(dev, 3);
            Show_GenTask_Message();
        }
        break;
        }
    }
}

/*********************************************END OF FILE**********************/
