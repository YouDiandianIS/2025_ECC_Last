/**
 * @file sht30.h
 * @brief SHT30��ʪ�ȴ���������ͷ�ļ���������������͡���ͺ������������ڳ�ʼ���Ͳ���SHT30��������
 * @author ֣���� (Zhongyue Zheng)
 * @date 2025-07-01
 * @version 1.0
 * @note ��ͷ�ļ�������STM32F4xx΢������ͨ��I2C�ӿ���SHT30������ͨ�ŵĻ�����
 *       ȷ��I2C������ȷ��ʼ����������Ҫ��STM32 HAL�⡣
 * @license MIT License
 */

#ifndef __SHT30_H
#define __SHT30_H

/* ͷ�ļ����� */
#include "main.h"
#include "i2c.h"
#include "stdio.h"
#include <stdint.h>

/* �궨�� */
#define SHT30_ADDR         (0x44)          // SHT30 I2C��ַ
#define SHT30_ADDR_W       (SHT30_ADDR << 1) // SHT30д��ַ������1λ����HAL�⣩
#define SHT30_ADDR_R       (SHT30_ADDR << 1) // SHT30����ַ����д��ַ��ͬ��HAL���Զ������дλ��

/* �������� */
/**
 * @brief ��λSHT30������
 * @details ͨ��I2C�ӿڷ��͸�λ������ô�����״̬��
 */
void SHT30_reset(void);

/**
 * @brief ��ʼ��SHT30������
 * @details ���ô�����Ϊ�߾������ڲ���ģʽ��
 */
void Init_SHT30(void);

/**
 * @brief ��ȡSHT30��ʪ������
 * @details ͨ��I2C�ӿڶ�ȡ�¶Ⱥ�ʪ�����ݣ�������CRCУ�顣
 * @param Humidity ָ��洢ʪ��ֵ��ָ�루��λ��%RH��
 * @param Temperature ָ��洢�¶�ֵ��ָ�루��λ�����϶ȣ�
 */
void SHT30_ReadData(float *Humidity, float *Temperature);

/**
 * @brief ����SHT30���ݵ�CRCУ��ֵ
 * @details ʹ�ö���ʽ��ָ�����ݽ���CRC-8У�飬�Ƚϼ���������յ���У��ֵ��
 * @param data ��������ָ��
 * @param nbrOfBytes �����ֽ���
 * @param checksum ���յ���У��ֵ
 * @return 0��ʾУ��ɹ���1��ʾУ��ʧ��
 */
unsigned char SHT30_CheckCrc(char data[], char nbrOfBytes, char checksum);

/**
 * @brief �����¶�ֵ�����϶ȣ�
 * @details ����SHT30�Ĺ�ʽ��16λԭʼ����ת��Ϊ�¶�ֵ��
 * @param u16sT 16λ�¶�ԭʼ����
 * @return �¶�ֵ�����϶ȣ�
 */
float SHT30_CalcTemperatureC(unsigned short u16sT);

/**
 * @brief �������ʪ��ֵ
 * @details ����SHT30�Ĺ�ʽ��16λԭʼ����ת��Ϊ���ʪ��ֵ��
 * @param u16sRH 16λʪ��ԭʼ����
 * @return ʪ��ֵ��%RH��
 */
float SHT30_CalcRH(unsigned short u16sRH);

#endif /* __SHT30_H */
