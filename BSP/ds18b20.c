#include "ds18b20.h"

// Ӳ������
#define DS18B20_PORT    GPIOB
#define DS18B20_PIN     GPIO_PIN_1

// ���Ų�����
#define DS18B20_OUT_LOW()   HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET)
#define DS18B20_OUT_HIGH()  HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET)
#define DS18B20_IN()        HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN)

// ˽�к�������
static void DS18B20_IO_Input(void);
static void DS18B20_IO_Output(void);
static uint8_t DS18B20_Reset(void);

/**
  * @brief  ��ȷ΢����ʱ��100MHzʱ���Ż���
  * @param  us: ΢���� (0~65535)
  */
void DS18B20_Delay(uint32_t us)
{
    uint32_t ticks = us * (100 / 8);  // 100MHzʱ�Ӽ���
    do {
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
    } while(--ticks);
}

/**
  * @brief  GPIO����Ϊ����ģʽ����������
  */
static void DS18B20_IO_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);
}

/**
  * @brief  GPIO����Ϊ���ģʽ�����죩
  */
static void DS18B20_IO_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);
}

/**
  * @brief  ���͸�λ���岢�������ź�
  * @retval 0: �ɹ� 1: ����Ӧ
  */
static uint8_t DS18B20_Reset(void)
{
    DS18B20_IO_Output();
    
    // ��������480us
    DS18B20_OUT_LOW();
    DS18B20_Delay(480);
    
    // �ͷ�����
    DS18B20_OUT_HIGH();
    DS18B20_IO_Input();
    DS18B20_Delay(70);  // �ȴ�15-60us
    
    // ����������
    if(DS18B20_IN() != GPIO_PIN_RESET) return 1;
    
    // �ȴ��������������60-240us��
    uint32_t timeout = 240;
    while(DS18B20_IN() == GPIO_PIN_RESET && timeout--) 
        DS18B20_Delay(1);
    
    return (timeout > 0) ? 0 : 1;
}

/**
  * @brief  д��һ���ֽ�
  * @param  data: Ҫд�������
  */
void DS18B20_WriteByte(uint8_t data)
{
    DS18B20_IO_Output();
    for(uint8_t i=0; i<8; i++){
        DS18B20_OUT_LOW();
        DS18B20_Delay(2);  // ����1us
        
        if(data & 0x01) DS18B20_OUT_HIGH();
        else DS18B20_OUT_LOW();
        
        DS18B20_Delay(60);  // ����60us
        DS18B20_OUT_HIGH();
        data >>= 1;
        DS18B20_Delay(2);
    }
}

/**
  * @brief  ��ȡһ���ֽ�
  * @retval ��ȡ��������
  */
uint8_t DS18B20_ReadByte(void)
{
    uint8_t data = 0;
    for(uint8_t i=0; i<8; i++){
        data >>= 1;
        DS18B20_IO_Output();
        DS18B20_OUT_LOW();
        DS18B20_Delay(2);  // ����1us
        
        DS18B20_IO_Input();
        DS18B20_Delay(12);  // �ȴ�15us����
        
        if(DS18B20_IN() == GPIO_PIN_SET) data |= 0x80;
        
        DS18B20_Delay(50);  // �������ʱ϶
    }
    return data;
}

/**
  * @brief  �����¶�ת��
  */
void DS18B20_StartConversion(void)
{
    DS18B20_Reset();
    DS18B20_WriteByte(0xCC);  // ����ROM
    DS18B20_WriteByte(0x44);  // ����ת��
}

/**
  * @brief  ��ȡ�¶�ֵ������ʽ�ȴ�ת����ɣ�
  * @retval �¶�ֵ������λС������λ���棩
  */
float DS18B20_ReadTemperature(void)
{
    // ����ת�����ȴ����
    DS18B20_StartConversion();
    DS18B20_Delay(750000);    // 750ms�ȴ�ת�����
    
    // ��ȡ�ݴ���
    DS18B20_Reset();
    DS18B20_WriteByte(0xCC);  // ����ROM
    DS18B20_WriteByte(0xBE);  // ���ݴ���
    
    // ��ȡ�¶�����
    uint8_t lsb = DS18B20_ReadByte();
    uint8_t msb = DS18B20_ReadByte();
    
    // ת��Ϊʵ���¶�
    int16_t temp = (msb << 8) | lsb;
    return temp * 0.0625f;
}

/**
  * @brief  Ӳ����ʼ��
  * @retval 0: �ɹ� 1: �豸δ��Ӧ
  */
uint8_t DS18B20_Init(void)
{
    // ȷ��GPIOʱ����ʹ��
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // ��ʼ��Ϊ�ߵ�ƽ
    DS18B20_IO_Output();
    DS18B20_OUT_HIGH();
    
    // ���͸�λ����
    return DS18B20_Reset();
}
