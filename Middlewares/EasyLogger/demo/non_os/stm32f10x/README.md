# stm32f10x ��� demo

---

## 1�����

ͨ�� `app\src\app.c` �� `test_elog()` ������������־�������

### 1.1��ʹ�÷���

�򿪵��Ե��ն���demo�Ĵ���1�������ӣ��������� 115200 8 1 N����ʱ�������ն��Ͽ���demo�Ĵ�ӡ��־

> ע�⣺�����޷������ն˵��û���Ҳ����ʹ�÷�������demoƽ̨�������ӣ����Թ۲���־����������

## 2���ļ����У�˵��

`components\easylogger\port\elog_port.c` ��ֲ�ο��ļ�

`RVMDK` ��ΪKeil�����ļ�

`EWARM` ��ΪIAR�����ļ�

## 3����������

���Դ� `app\src\app.c` �еĲ���ע�ͣ����������¹��ܡ�

- `elog_set_output_enabled(false);` ����̬ʹ�ܻ�ʧ����־���
- `elog_set_filter_lvl(ELOG_LVL_WARN);` ����̬���ù������ȼ�
- `elog_set_filter_tag("main");` ����̬���ù��˱�ǩ
- `elog_set_filter_kw("Hello");` ����̬���ù��˹ؼ���
- `elog_set_filter_tag_lvl("main", ELOG_LVL_WARN);` ����̬���ù��˹ؼ��ʼ���