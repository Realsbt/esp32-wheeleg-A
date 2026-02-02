#include "adc.h"
#include <Adafruit_ADS1X15.h>
#include "motor.h"
Adafruit_ADS1115 ADS1115;  /* Use this for the 16-bit version */
float vot;

#define VOLTAGE_COMPENSATION  10.0f  //越小代表电池压降带来的电机倍率补偿越大

void Batteryvoltage_Task(void *pvParameters)
{
    int16_t adc0;
    float volts0;
    while(1)
    {
        adc0 = ADS1115.readADC_SingleEnded(0);
        vot = ADS1115.computeVolts(adc0)/0.191678f;
        if (vot >14 && vot <17) //正常的电压
        {
            motorOutRatio = (16.8f-vot)/VOLTAGE_COMPENSATION +1.0f;
        }
        vTaskDelay(200);
    }
}
void ADS1115_Init(void)
{
    //走I2C总线，如果MPU6050已经初始化了,则无需下面两行
    // Wire.begin(21, 22);//SDA SCL
    // Wire.setClock(400000);
    ADS1115.begin();  /* This initializes the ADC */
    xTaskCreate(Batteryvoltage_Task, "Batteryvoltage_Task", 4096, NULL, 5, NULL);
}