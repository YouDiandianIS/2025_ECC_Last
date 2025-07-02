/**
 * @file bh1750.h
 * @brief BH1750���մ���������ͷ�ļ�����������غꡢ�ⲿ�����ͺ������������ڳ�ʼ���Ͳ���BH1750��������
 * @author ֣���� (Zhongyue Zheng)
 * @date 2025-07-01
 * @version 1.0
 * @note ��ͷ�ļ�������STM32΢������ͨ��I2C�ӿ���BH1750������ͨ�ŵĻ�����
 *       ȷ��I2C������ȷ��ʼ����������Ҫ�������⡣
 * @license MIT License
 */
#ifndef __BH1750_H_
#define __BH1750_H_

/* ͷ�ļ����� */
#include "i2c.h"
#include "stdint.h"

/* �궨�� */
#define BH1750_ADDRESS       (0x23 << 1)  // BH1750 I2C��ַ������1λ������HAL�⣩
#define POWER_ON             0x01        // ��Դ��������
#define RESET                0x07        // ��λ����
#define CONT_H_RES_MODE      0x10        // �����߷ֱ���ģʽ����

/* �������� */
/**
 * @brief ��ʼ��BH1750���մ�����
 * @details ���ô��������������߷ֱ���ģʽ��
 */
void GY30_Init(void);

/**
 * @brief ��ȡBH1750����ǿ������
 * @details ͨ��I2C�ӿڶ�ȡ����ǿ��ֵ����λ���տ�˹����
 * @return ����ǿ��ֵ���տ�˹��������ȡʧ�ܷ���-1.0f��
 */
float Multiple_read_BH1750(void);

/**
 * @brief ��BH1750���͵��ֽ�����
 * @details ͨ��I2C�ӿ���BH1750����ָ���������ֽڡ�
 * @param cmd �����͵������ֽ�
 */
void Single_Write_BH1750(unsigned char cmd);

#endif /* __BH1750_H_ */
