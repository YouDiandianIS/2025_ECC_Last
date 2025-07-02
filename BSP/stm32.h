/**
 * @file stm32.h
 * @brief STM32΢������ͨ��ͷ�ļ��������˶�����ơ����������ݴ���ʹ���ͨ����صĺꡢ�ṹ��ͺ���������
 * @author ֣���� (Zhongyue Zheng)
 * @date 2025-07-01
 * @version 1.0
 * @note ��ͷ�ļ�������STM32΢�������������������������ļ�����bh1750.c��sht30.c�ȣ�ʹ�á�
 *       ȷ������ʹ�������ȷ��ʼ����
 * @license MIT License
 */

#ifndef __STM32_H__
#define __STM32_H__

/* ͷ�ļ����� */
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "adc.h"
#include "tim.h"
#include "bh1750.h"
#include "sht30.h"
#include "ds18b20.h"
#include <stdarg.h>
#include <stdbool.h>

/* �궨�� */
#define len2                        100      // ���ڽ��ջ���������
#define FRAME_HEADER                0x55     // ͨ��Э��֡ͷ
#define CMD_SERVO_MOVE              0x03     // ����ƶ�����
#define CMD_ACTION_GROUP_RUN        0x04     // ���ж���������
#define CMD_ACTION_GROUP_STOP       0x05     // ֹͣ����������
#define CMD_ACTION_GROUP_SPEED      0x06     // ���ö������ٶ�����
#define CMD_GET_BATTERY_VOLTAGE     0x0F     // ��ȡ��ص�ѹ����

/* �ṹ�嶨�� */
/**
 * @brief ������ƽṹ��
 * @details �������ID��Ŀ��λ�ã����ڶ������ơ�
 */
typedef struct {
    uint8_t id;         // ���ID (0 <= id <= 31)
    uint16_t position;  // Ŀ��λ�� (0-1000����Ӧ0-180��)
} LobotServo;

/* �������� */
/**
 * @brief ��ʼ����������ʹ�����
 * @details ��ʼ��GY30���մ�������SHT30��ʪ�ȴ�������DS18B20�¶ȴ�����������UART2�жϣ�������ʱ��3������ʼ�����λ�á�
 */
void All_Init(void);

/**
 * @brief ��ȡADCֵ
 * @details ����ADCת������ѯ��ȡ���������ת��ֵ��
 * @return ADCת��ֵ
 */
uint32_t ADC_Read(void);

/**
 * @brief ���Ƶ������ת��
 * @details ���ݶ��ID��Ŀ��λ�ú�ת��ʱ�����ɿ���ָ�ͨ�����ڷ��͡�
 * @param servoID ���ID (0 <= servoID <= 31)
 * @param position Ŀ��λ��
 * @param time ת��ʱ�� (time > 0)
 */
void moveServo(uint8_t servoID, uint16_t position, uint16_t time);

/**
 * @brief ���ƶ�����ת����ͨ�����飩
 * @details ���ݶ���ṹ�����顢���������ת��ʱ�����ɿ���ָ����͡�
 * @param servos ����ṹ������
 * @param num ������� (0 < num <= 32)
 * @param time ת��ʱ�� (time > 0)
 */
void moveServosByArray(LobotServo servos[], uint8_t num, uint16_t time);

/**
 * @brief ���ƶ�����ת�����ɱ������
 * @details ���ݶ��������ת��ʱ��Ϳɱ���������ID��Ŀ��λ�ý������У����ɿ���ָ����͡�
 * @param num ������� (0 < num <= 32)
 * @param time ת��ʱ�� (time > 0)
 * @param ... ���ID��Ŀ��λ�ý�������
 */
void moveServos(uint8_t num, uint16_t time, ...);

/**
 * @brief ����ָ��������
 * @details ��������ָ���������ָ����ͣ��������ѭ��ִ��ָ��������
 * @param numOfAction ���������
 * @param times ִ�д��� (times = 0 ʱ����ѭ��)
 */
void runActionGroup(uint8_t numOfAction, uint16_t times);

/**
 * @brief ֹͣ����������
 * @details ����ֹͣ�������ָ����͡�
 */
void stopActionGroup(void);

/**
 * @brief ����ָ��������������ٶ�
 * @details �������ö������ٶȵ�ָ����͡�
 * @param numOfAction ���������
 * @param speed Ŀ���ٶ�
 */
void setActionGroupSpeed(uint8_t numOfAction, uint16_t speed);

/**
 * @brief �������ж�����������ٶ�
 * @details ����setActionGroupSpeed�������������ж�������ٶȣ����0xFF����
 * @param speed Ŀ���ٶ�
 */
void setAllActionGroupSpeed(uint16_t speed);

/**
 * @brief ���ͻ�ȡ��ص�ѹ����
 * @details ���ɻ�ȡ��ص�ѹ��ָ����͡�
 */
void getBatteryVoltage(void);

/**
 * @brief �����ڽ�������
 * @details ���ݽ��յ�����������ݣ�Ŀǰ֧�ֻ�ȡ��ص�ѹ�Ĵ���
 */
void receiveHandle(void);

/**
 * @brief ͨ�����ڷ�������
 * @details ʹ��UART6����ָ�����ȵ����ݡ�
 * @param buf ���ݻ�����
 * @param len ���ݳ���
 */
void uartWriteBuf(uint8_t *buf, uint16_t len);

#endif /* __STM32_H__ */
