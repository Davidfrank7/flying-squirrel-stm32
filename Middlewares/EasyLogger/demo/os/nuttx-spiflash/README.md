# Nuttx SPI Flash demo

---

## 1�����

ͨ�� `apps/examples/easylogger/elog_main.c` �� `test_elog()` ������������־�������Ĭ�Ͽ������첽���ģʽ���û����Խ����ն���������־�����������������á�
`test_env()` ��������ʾ���������Ķ�ȡ���޸Ĺ��ܣ�ÿ��ϵͳ�������ҳ�ʼ��EasyFlash�ɹ������ø÷�����

- ƽ̨������NuttX-12.0.0��NuttX-10.X.X��Ҫ�޸�Makefile���include�����÷�����
- Ӳ��������STM32F103��Nuttx��apps������Ӳ��ƽ̨��
- Flash������W25QXX��Nuttx֧�ֵ�SPI Flash�������`nuttx/drivers/mtd`�������ļ���

## 2��ʹ�÷���

�����������ɺ���nsh����̨����`elog`�س������ű����۲���������

![ElogNuttxSpiFlashDemo](https://raw.githubusercontent.com/armink/EasyLogger/master/docs/zh/images/ElogNuttxSpiFlashDemo.png)

## 3���ļ����У�˵��

|Դ�ļ����У�                            |����   |
|:------------------------------         |:----- |
|apps                                    |nuttx-appsӦ�ò�Ŀ¼|
|apps/examples/                          |nuttxӦ�ò�ʾ������Ŀ¼|
|apps/examples/easylogger/elog_main.c    |Easylogger��Easyflash��Example Demo����|
|apps/system/                            |nuttxӦ�ò�ϵͳ��Ŀ¼|
|apps/system/easylogger/inc/elog_cfg.h   |Easylogger�����ļ�|
|apps/system/easylogger/port/elog_port.c |Easylogger��ֲ�ο��ļ�|
|apps/system/easyflash/inc/ef_cfg.h      |Easyflash�����ļ�|
|apps/system/easyflash/port/ef_port.c    |Easyflash��ֲ�ο��ļ�|

## 4������˵��

### 4.1����ֲ˵��

1���Ѹ�Ŀ¼`EasyLogger/`���������Щ�ļ�������ָ��λ�ã�
```
cd EasyLogger/
cp easylogger/inc/elog.h demo/os/nuttx-spiflash/apps/system/easylogger/inc
cp easylogger/plugins/flash/elog_flash.* demo/os/nuttx-spiflash/apps/system/easylogger/plugins/flash/
cp -R easylogger/src/ demo/os/nuttx-spiflash/apps/system/easylogger/
```

2������apps������Ŀ¼���ǵ�nuttx��appsĿ¼�¡�

3��nuttxĿ¼�����������appsĿ¼�ṹ���档
```
make apps_distclean
make menuconfig
```

4��`make menuconfig`ѡ���������ã���Easylogger Demo Example��
```
CONFIG_EXAMPLES_EASYLOGGER=y
```
���Զ�������
```
CONFIG_MTD_W25=y
```

5�����mtd ioctl��ָ��ID��������ر�Easyflash���ܿ���ʡ�ԣ�
- ��`nuttx/include/nuttx/mtd/mtd.h`�ļ�������ӣ�
```C
#define MTDIOC_GETMTDDEV  _MTDIOC(0x000c)
```

6����¶w25��mtd�豸�ڵ㣺������ر�Easyflash���ܿ���ʡ�ԣ�
- ��`nuttx/drivers/mtd/w25.c`�ļ�
- ��ӱ���������
```C
static FAR struct mtd_dev_s *mtd_w25;
```
- �ҵ�`w25_ioctl`��������`switch (cmd)`������ӣ�
```C
      case MTDIOC_GETMTDDEV:{
          FAR struct mtd_dev_s **mtd =
            (FAR struct mtd_dev_s *)((uintptr_t)arg);
          DEBUGASSERT(*mtd != NULL);
          *mtd = mtd_w25;
          ret = OK;
      }break;
```
- �ҵ�`w25_initialize`��������`ret = w25_readid(priv)`��ID��ȷ���ص���������ӣ�
```C
mtd_w25=&priv->mtd;
```

7������`make -j4`�����ص����Ӻ���NSH����̨����`elog`�����ɲ鿴��������
