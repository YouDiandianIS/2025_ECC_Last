/**
 * @file bh1750.c
 * @brief BH1750���մ����������������ڳ�ʼ���������Ͷ�ȡ����ǿ�����ݡ�
 * @author ֣���� (Zhongyue Zheng)
 * @date 2025-07-01
 * @version 1.0
 * @note �ô�����ͨ�����ԣ�������STM32΢������ͨ��I2C�ӿ���BH1750������ͨ�ŵĻ�����
 *       ȷ��I2C������ȷ��ʼ��������BH1750��������
 * @license MIT License
 */

/* ͷ�ļ����� */
#include "bh1750.h"

/**
 * @brief ��ʼ��BH1750���մ�����
 * @details ͨ��I2C�ӿڷ��͵�Դ��������������߷ֱ���ģʽ�����ʼ����������
 */
void GY30_Init(void)
{
    uint8_t cmd;
    HAL_Delay(10); // �ȴ�10ms��ȷ���������ȶ�

    cmd = POWER_ON; // ���õ�Դ��������
    HAL_I2C_Master_Transmit(&hi2c2, BH1750_ADDRESS, &cmd, 1, HAL_MAX_DELAY); // ���͵�Դ��������
    HAL_Delay(10); // �ȴ�10ms

    cmd = CONT_H_RES_MODE; // ���������߷ֱ���ģʽ����
    HAL_I2C_Master_Transmit(&hi2c2, BH1750_ADDRESS, &cmd, 1, HAL_MAX_DELAY); // ����ģʽ����
}

/**
 * @brief ��ȡBH1750����ǿ������
 * @details ͨ��I2C�ӿڶ�ȡ2�ֽ�ԭʼ���ݣ�ת��Ϊ����ǿ��ֵ����λ���տ�˹����
 * @return ����ǿ��ֵ���տ�˹��������ȡʧ�ܷ���-1.0f��
 */
float Multiple_read_BH1750(void)
{
    uint8_t buf[2];
    if (HAL_I2C_Master_Receive(&hi2c2, BH1750_ADDRESS, buf, 2, HAL_MAX_DELAY) != HAL_OK)
        return -1.0f; // I2C��ȡʧ�ܣ����ش���ֵ

    uint16_t raw = (buf[0] << 8) | buf[1]; // �ϲ��ߵ��ֽ�Ϊ16λԭʼֵ
    return raw / 1.2f; // ת��Ϊ����ǿ�ȣ��տ�˹��
}

/**
 * @brief ��BH1750���͵��ֽ�����
 * @details ͨ��I2C�ӿ���BH1750����ָ���������ֽڡ�
 * @param cmd �����͵������ֽ�
 */
void Single_Write_BH1750(unsigned char cmd)
{
    HAL_I2C_Master_Transmit(&hi2c2, BH1750_ADDRESS, &cmd, 1, HAL_MAX_DELAY); // ���͵��ֽ�����
}
