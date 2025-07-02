/**
 * @file stm32_servo_control.c
 * @brief STM32΢�������������ڿ��ƶ�������������ݲɼ��ʹ���������ʪ�ȡ�����ǿ�ȡ��ƾ�Ũ�ȵ����ݵĲɼ��봫�䣬
 *        �Լ�ͨ������ͨ�ſ��ƶ���˶��Ͷ��������С�
 * @author ֣���� (Zhongyue Zheng)
 * @date 2025-07-01
 * @version 1.0
 * @note �ô�����ͨ�����ԣ�������ʵ��Ӳ�������С�ȷ��Ӳ����ʼ����ȷ��������ش������Ͷ����
 * @license MIT License
 */

/* ͷ�ļ����� */
#include "stm32.h"

/* ȫ�ֱ������� */
bool start, isUartRxCompleted;						// ��־λ�����������ʹ��ڽ������
unsigned char LobotTxBuf[128];						// ����������ݻ�����
unsigned short int batteryVolt, LobotRxBuf[16];		// ��ص�ѹ�ͽ������ݻ�����
unsigned char RX_len, RX_buf[len2], jsonData[200];	// ���ڽ��ճ��ȡ����ջ�������JSON���ݻ�����
extern UART_HandleTypeDef huart2;					// UART2���
extern UART_HandleTypeDef huart6;					// UART6���
extern TIM_HandleTypeDef htim3;						// ��ʱ��3���
extern TIM_HandleTypeDef htim4;						// ��ʱ��4���
extern ADC_HandleTypeDef hadc1;						// ADC1���
float Temperature, Temperature_c, Humidity = 0;		// �¶ȣ����������¶ȣ���Һ����ʪ��
unsigned int adcValue = 0;							// ADC�ɼ�ֵ
float Alcohol = 0;									// �ƾ�Ũ��
int lux, lux_cnt = 0;								// ����ǿ�Ⱥͼ�����
unsigned short int count3, count4, ice_cnt, buz_cnt = 0;	// ��ʱ�������ͷ���������

/**
 * @brief ��ʼ����������ʹ�����
 * @details ��ʼ��GY30���մ�������SHT30��ʪ�ȴ�������DS18B20�¶ȴ�������
 *          ����UART2�жϣ�������ʱ��3������ʼ�����λ�á�
 */
void All_Init(void)
{
    GY30_Init();               // ��ʼ��GY30���մ�����
    Init_SHT30();              // ��ʼ��SHT30��ʪ�ȴ�����
    DS18B20_Init();            // ��ʼ��DS18B20�¶ȴ�����
    
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); // ����UART2�����ж�
    HAL_UART_Receive_IT(&huart2, RX_buf, len2);  // ����UART2�����ж�
    HAL_TIM_Base_Start_IT(&htim3);               // ������ʱ��3�ж�

    moveServo(1, 0, 500);                        // ��ʼ�����1��0�ȣ�ʱ��500ms
    moveServo(2, 1000, 1000);                    // ��ʼ�����2���ض�λ�ã�ʱ��1000ms
}

/**
 * @brief �ض����׼������������ַ�ͨ��UART6����
 * @param ch �����͵��ַ�
 * @param f �ļ�ָ��
 * @return ���ط��͵��ַ�
 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 1); // ͨ��UART6���͵����ַ�
    return ch;
}

/**
 * @brief ��ʱ���жϻص�����
 * @details ����ʱ��3�Ͷ�ʱ��4���жϡ���ʱ��3���ڲɼ����������ݲ�ͨ��JSON��ʽ���ͣ�
 *          ��ʱ��4���ڿ��ƶ���˶����кͷ�������
 * @param htim ��ʱ�����
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        count3++;
        lux_cnt++;
        if (count3 >= 10000) {
            start = 0;
            // ��ʽ������������ΪJSON��ͨ��UART2����
            snprintf((char*)jsonData, sizeof(jsonData),
                     "{\"services\": [{\"service_id\": \"2025_Ecc\", \"properties\": {\"Humidity\": %.2f, \"Temperature\": %.2f, \"Temperature_C\": %.2f, \"Alcohol\": %.2f, \"Light\": %d}}]}",
                     Humidity, Temperature, Temperature_c, Alcohol, lux);
            HAL_UART_Transmit(&huart2, (const unsigned char*)jsonData, sizeof(jsonData), 50);
            count3 = 0;
        }
        if (lux_cnt == 0) {
            Single_Write_BH1750(CONT_H_RES_MODE); // ����BH1750���մ����������߷ֱ���ģʽ
        } else if (lux_cnt > 181) {
            lux = Multiple_read_BH1750(); // ��ȡ����ǿ��
            lux_cnt = 0;
        }
    } else if (htim->Instance == TIM4) {
        ice_cnt++;
        buz_cnt++;
        // ����˶����п���
        if (ice_cnt == 0)       moveServo(2, 810, 900);   // �϶��ת��45��
        else if (ice_cnt == 1000) moveServo(2, 620, 900); // �϶��ת��90��
        else if (ice_cnt == 3000) moveServo(1, 750, 1900);// �¶��ת��180��
        else if (ice_cnt == 5500) moveServo(2, 1000, 400);// �϶��ת��0��
        else if (ice_cnt == 6000) moveServo(2, 620, 800); // �϶��ת��90��
        else if (ice_cnt == 7000) moveServo(2, 1000, 400);// �϶��ת��0��
        else if (ice_cnt == 8000) moveServo(2, 620, 800); // �϶��ת��90��
        else if (ice_cnt == 9000) moveServo(1, 0, 600);   // �¶��ת��0��
        else if (ice_cnt == 10800) moveServo(2, 1000, 1000); // �϶��ת��0��
        else if (ice_cnt == 11000) {
            ice_cnt = 0;
            buz_cnt = 0;
            HAL_TIM_Base_Stop_IT(&htim4); // ֹͣ��ʱ��4
        }
        
        // ����������
        if (buz_cnt > 1 && buz_cnt < 666)
            HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief ��ȡADCֵ
 * @details ����ADCת������ѯ��ȡ���������ת��ֵ��
 * @return ADCת��ֵ
 */
uint32_t ADC_Read(void)
{
    uint32_t adcValue = 0;
    HAL_ADC_Start(&hadc1); // ����ADC
    if (HAL_ADC_PollForConversion(&hadc1, 200) == HAL_OK)
        adcValue = HAL_ADC_GetValue(&hadc1); // ��ȡADCֵ
    HAL_ADC_Stop(&hadc1); // ֹͣADC
    return adcValue;
}

/**
 * @brief ���Ƶ������ת��
 * @details ���ݶ��ID��Ŀ��λ�ú�ת��ʱ�����ɿ���ָ�ͨ�����ڷ��͡�
 * @param servoID ���ID (0 <= servoID <= 31)
 * @param position Ŀ��λ��
 * @param time ת��ʱ�� (time > 0)
 */
void moveServo(uint8_t servoID, uint16_t position, uint16_t time)
{
    if (servoID > 31 || !(time > 0)) {
        return;
    }
    
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;    // ���֡ͷ
    LobotTxBuf[2] = 8;                               // ���ݳ���
    LobotTxBuf[3] = CMD_SERVO_MOVE;                  // ����ƶ�ָ��
    LobotTxBuf[4] = 1;                               // Ҫ���ƵĶ������
    LobotTxBuf[5] = (uint8_t)(time);                 // ʱ��Ͱ�λ
    LobotTxBuf[6] = (uint8_t)(time >> 8);            // ʱ��߰�λ
    LobotTxBuf[7] = servoID;                         // ���ID
    LobotTxBuf[8] = (uint8_t)(position);             // Ŀ��λ�õͰ�λ
    LobotTxBuf[9] = (uint8_t)(position >> 8);        // Ŀ��λ�ø߰�λ
    
    uartWriteBuf(LobotTxBuf, 10);                    // ��������
}

/**
 * @brief ���ƶ�����ת����ͨ�����飩
 * @details ���ݶ���ṹ�����顢���������ת��ʱ�����ɿ���ָ����͡�
 * @param servos ����ṹ������
 * @param num ������� (0 < num <= 32)
 * @param time ת��ʱ�� (time > 0)
 */
void moveServosByArray(LobotServo servos[], uint8_t num, uint16_t time)
{
    uint8_t index = 7;
    uint8_t i = 0;

    if (num < 1 || num > 32 || !(time > 0)) {
        return;
    }
    
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      // ���֡ͷ
    LobotTxBuf[2] = num * 3 + 5;                       // ���ݳ���
    LobotTxBuf[3] = CMD_SERVO_MOVE;                    // ����ƶ�ָ��
    LobotTxBuf[4] = num;                               // Ҫ���ƵĶ������
    LobotTxBuf[5] = (uint8_t)(time);                   // ʱ��Ͱ�λ
    LobotTxBuf[6] = (uint8_t)(time >> 8);              // ʱ��߰�λ

    for (i = 0; i < num; i++) {
        LobotTxBuf[index++] = servos[i].id;            // �����ID
        LobotTxBuf[index++] = (uint8_t)(servos[i].position); // Ŀ��λ�õͰ�λ
        LobotTxBuf[index++] = (uint8_t)(servos[i].position >> 8); // Ŀ��λ�ø߰�λ
    }

    uartWriteBuf(LobotTxBuf, LobotTxBuf[2] + 2);       // ��������
}

/**
 * @brief ���ƶ�����ת�����ɱ������
 * @details ���ݶ��������ת��ʱ��Ϳɱ���������ID��Ŀ��λ�ý������У����ɿ���ָ����͡�
 * @param num ������� (0 < num <= 32)
 * @param time ת��ʱ�� (time > 0)
 * @param ... ���ID��Ŀ��λ�ý�������
 */
void moveServos(uint8_t num, uint16_t time, ...)
{
    uint8_t index = 7;
    uint8_t i = 0;
    uint16_t temp;
    va_list arg_ptr;

    va_start(arg_ptr, time);
    if (num < 1 || num > 32) {
        va_end(arg_ptr);
        return;
    }
    
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      // ���֡ͷ
    LobotTxBuf[2] = num * 3 + 5;                       // ���ݳ���
    LobotTxBuf[3] = CMD_SERVO_MOVE;                    // ����ƶ�ָ��
    LobotTxBuf[4] = num;                               // Ҫ���ƵĶ������
    LobotTxBuf[5] = (uint8_t)(time);                   // ʱ��Ͱ�λ
    LobotTxBuf[6] = (uint8_t)(time >> 8);              // ʱ��߰�λ

    for (i = 0; i < num; i++) {
        temp = va_arg(arg_ptr, int);                   // ��ȡ���ID
        LobotTxBuf[index++] = (uint8_t)temp;
        temp = va_arg(arg_ptr, int);                   // ��ȡĿ��λ��
        LobotTxBuf[index++] = (uint8_t)temp;           // Ŀ��λ�õͰ�λ
        LobotTxBuf[index++] = (uint8_t)(temp >> 8);    // Ŀ��λ�ø߰�λ
    }

    va_end(arg_ptr);
    uartWriteBuf(LobotTxBuf, LobotTxBuf[2] + 2);       // ��������
}

/**
 * @brief ����ָ��������
 * @details ��������ָ���������ָ����ͣ��������ѭ��ִ��ָ��������
 * @param numOfAction ���������
 * @param times ִ�д��� (times = 0 ʱ����ѭ��)
 */
void runActionGroup(uint8_t numOfAction, uint16_t times)
{
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;  // ���֡ͷ
    LobotTxBuf[2] = 5;                              // ���ݳ���
    LobotTxBuf[3] = CMD_ACTION_GROUP_RUN;           // ���ж���������
    LobotTxBuf[4] = numOfAction;                    // �������
    LobotTxBuf[5] = (uint8_t)(times);               // ִ�д����Ͱ�λ
    LobotTxBuf[6] = (uint8_t)(times >> 8);          // ִ�д����߰�λ

    uartWriteBuf(LobotTxBuf, 7);                    // ��������
}

/**
 * @brief ֹͣ����������
 * @details ����ֹͣ�������ָ����͡�
 */
void stopActionGroup(void)
{
    LobotTxBuf[0] = FRAME_HEADER;     // ���֡ͷ
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = 2;                // ���ݳ���
    LobotTxBuf[3] = CMD_ACTION_GROUP_STOP;   // ֹͣ����������

    uartWriteBuf(LobotTxBuf, 4);      // ��������
}

/**
 * @brief ����ָ��������������ٶ�
 * @details �������ö������ٶȵ�ָ����͡�
 * @param numOfAction ���������
 * @param speed Ŀ���ٶ�
 */
void setActionGroupSpeed(uint8_t numOfAction, uint16_t speed)
{
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;   // ���֡ͷ
    LobotTxBuf[2] = 5;                              // ���ݳ���
    LobotTxBuf[3] = CMD_ACTION_GROUP_SPEED;         // ���ö������ٶ�����
    LobotTxBuf[4] = numOfAction;                    // �������
    LobotTxBuf[5] = (uint8_t)(speed);               // �ٶȵͰ�λ
    LobotTxBuf[6] = (uint8_t)(speed >> 8);          // �ٶȸ߰�λ

    uartWriteBuf(LobotTxBuf, 7);                    // ��������
}

/**
 * @brief �������ж�����������ٶ�
 * @details ����setActionGroupSpeed�������������ж�������ٶȣ����0xFF����
 * @param speed Ŀ���ٶ�
 */
void setAllActionGroupSpeed(uint16_t speed)
{
    setActionGroupSpeed(0xFF, speed);  // ���Ϊ0xFFʱ������������ٶ�
}

/**
 * @brief ���ͻ�ȡ��ص�ѹ����
 * @details ���ɻ�ȡ��ص�ѹ��ָ����͡�
 */
void getBatteryVoltage(void)
{
    LobotTxBuf[0] = FRAME_HEADER;  // ���֡ͷ
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = 2;             // ���ݳ���
    LobotTxBuf[3] = CMD_GET_BATTERY_VOLTAGE;  // ��ȡ��ص�ѹ����

    uartWriteBuf(LobotTxBuf, 4);   // ��������
}

/**
 * @brief �����ڽ�������
 * @details ���ݽ��յ�����������ݣ�Ŀǰ֧�ֻ�ȡ��ص�ѹ�Ĵ���
 */
void receiveHandle(void)
{
    if (isUartRxCompleted) {
        isUartRxCompleted = false;
        switch (LobotRxBuf[3]) {
            case CMD_GET_BATTERY_VOLTAGE: // ��ȡ��ѹ
                batteryVolt = ((uint16_t)(LobotRxBuf[5]) << 8) | LobotRxBuf[4];
                break;
            default:
                break;
        }
    }
}

/**
 * @brief ͨ�����ڷ�������
 * @details ʹ��UART6����ָ�����ȵ����ݡ�
 * @param buf ���ݻ�����
 * @param len ���ݳ���
 */
void uartWriteBuf(uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart6, buf, len, 1000); // ͨ��UART6��������
}
