/**
 * @file sht30.c
 * @brief SHT30��ʪ�ȴ����������������ڳ�ʼ������������λ����ȡ��ʪ�����ݣ�������CRCУ�顣
 * @author ֣���� (Zhongyue Zheng)
 * @date 2025-07-01
 * @version 1.0
 * @note �ô�����ͨ�����ԣ�������STM32΢������ͨ��I2C�ӿ���SHT30������ͨ�ŵĻ�����
 *       ȷ��I2C������ȷ��ʼ��������SHT30��������
 * @license MIT License
 */

/* ͷ�ļ����� */
#include "sht30.h"

/* �궨��ͳ��� */
const int16_t SHT30_POLYNOMIAL = 0x131; // CRCУ�����ʽ
#define SHT30_ADDR      (0x44)          // SHT30 I2C��ַ
#define SHT30_ADDR_W    (SHT30_ADDR << 1) // SHT30д��ַ������1λ����HAL�⣩
#define SHT30_ADDR_R    (SHT30_ADDR << 1) // SHT30����ַ����д��ַ��ͬ��HAL���Զ������дλ��

/**
 * @brief ��λSHT30������
 * @details ͨ��I2C�ӿڷ��͸�λ���0x30A2�������ȴ�50ms��ȷ����λ��ɡ�
 */
void SHT30_reset(void)
{
    unsigned char cmd[2] = {0x30, 0xA2}; // ��λ����
    if (HAL_I2C_Master_Transmit(&hi2c1, SHT30_ADDR_W, cmd, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        // ���������գ������û���չ��
    }
    HAL_Delay(50); // �ȴ�50ms����ɸ�λ
}

/**
 * @brief ��ʼ��SHT30������
 * @details ��λ�����������ͳ�ʼ�����0x2236��������Ϊ�߾������ڲ���ģʽ��
 */
void Init_SHT30(void)
{
    unsigned char cmd[2] = {0x22, 0x36}; // ��ʼ������߾������ڲ�����
    SHT30_reset(); // ִ�и�λ
    if (HAL_I2C_Master_Transmit(&hi2c1, SHT30_ADDR_W, cmd, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        printf("Init_SHT30_error: init fail\r\n"); // ��ʼ��ʧ�ܣ���ӡ������Ϣ
    }
    HAL_Delay(200); // �ȴ�200ms��ȷ����ʼ�����
}

/**
 * @brief ��ȡSHT30��ʪ������
 * @details ���Ͷ�ȡ���0xE000��������6�ֽ����ݣ��¶�+CRC��ʪ��+CRC����
 *          ����CRCУ�������¶Ⱥ�ʪ��ֵ��
 * @param Humidity ָ��洢ʪ��ֵ��ָ�루��λ��%RH��
 * @param Temperature ָ��洢�¶�ֵ��ָ�루��λ�����϶ȣ�
 */
void SHT30_ReadData(float *Humidity, float *Temperature)
{
    unsigned char cmd[2] = {0xE0, 0x00}; // ��ȡ����
    unsigned char rx_buffer[6]; // ���ջ��������¶�2�ֽ�+CRC1�ֽ�+ʪ��2�ֽ�+CRC1�ֽڣ�
    unsigned short int dat;

    if (HAL_I2C_Master_Transmit(&hi2c1, SHT30_ADDR_W, cmd, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        printf("SHT30_ReadData error: transmit fail\r\n"); // ��������ʧ�ܣ���ӡ������Ϣ
        return;
    }
    HAL_Delay(30); // �ȴ�30ms��ȷ������׼����

    if (HAL_I2C_Master_Receive(&hi2c1, SHT30_ADDR_R, rx_buffer, 6, HAL_MAX_DELAY) != HAL_OK)
    {
        return; // ��������ʧ�ܣ�ֱ�ӷ���
    }

    // У���¶�����
    if (SHT30_CheckCrc((char *)rx_buffer, 2, rx_buffer[2]) == 0)
    {
        dat = ((unsigned short int)rx_buffer[0] << 8) | rx_buffer[1]; // �ϲ��¶ȸߵ��ֽ�
        *Temperature = SHT30_CalcTemperatureC(dat); // �����¶�ֵ
    }
    else
    {
        printf("SHT30_ReadData error: temperature CRC fail\r\n"); // �¶�CRCУ��ʧ��
    }

    // У��ʪ������
    if (SHT30_CheckCrc((char *)(rx_buffer + 3), 2, rx_buffer[5]) == 0)
    {
        dat = ((unsigned short int)rx_buffer[3] << 8) | rx_buffer[4]; // �ϲ�ʪ�ȸߵ��ֽ�
        *Humidity = SHT30_CalcRH(dat); // ����ʪ��ֵ
    }
    else
    {
        printf("SHT30_ReadData error: humidity CRC fail\r\n"); // ʪ��CRCУ��ʧ��
    }
}

/**
 * @brief ����SHT30���ݵ�CRCУ��ֵ
 * @details ʹ�ö���ʽ0x131��ָ�����ݽ���CRC-8У�飬�Ƚϼ���������յ���У��ֵ��
 * @param data ��������ָ��
 * @param nbrOfBytes �����ֽ���
 * @param checksum ���յ���У��ֵ
 * @return 0��ʾУ��ɹ���1��ʾУ��ʧ��
 */
unsigned char SHT30_CheckCrc(char data[], char nbrOfBytes, char checksum)
{
    char crc = 0xFF; // ��ʼCRCֵ
    char bit;
    char byteCtr;

    for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
    {
        crc ^= data[byteCtr]; // ��ÿ���ֽڽ������
        for (bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ SHT30_POLYNOMIAL; // ������λΪ1�����ж���ʽ����
            else
                crc = (crc << 1); // ��������
        }
    }
    return (crc != checksum) ? 1 : 0; // �Ƚϼ����CRC����յ�У��ֵ
}

/**
 * @brief �����¶�ֵ�����϶ȣ�
 * @details ����SHT30�Ĺ�ʽ��16λԭʼ����ת��Ϊ�¶�ֵ����λ�����϶ȣ���
 * @param u16sT 16λ�¶�ԭʼ����
 * @return �¶�ֵ�����϶ȣ�
 */
float SHT30_CalcTemperatureC(unsigned short u16sT)
{
    u16sT &= ~0x0003; // ������2λ��״̬λ��
    return (175 * (float)u16sT / 65535 - 45); // ת��Ϊ���϶�
}

/**
 * @brief �������ʪ��ֵ
 * @details ����SHT30�Ĺ�ʽ��16λԭʼ����ת��Ϊ���ʪ��ֵ����λ��%RH����
 * @param u16sRH 16λʪ��ԭʼ����
 * @return ʪ��ֵ��%RH��
 */
float SHT30_CalcRH(unsigned short u16sRH)
{
    u16sRH &= ~0x0003; // ������2λ��״̬λ��
    return (100 * (float)u16sRH / 65535); // ת��Ϊ���ʪ��
}
