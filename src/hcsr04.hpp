/*
Author: Hepheir <hepheir@gmail.com>
Description: HC-SR04 ultrasonic sensor library for keil studio.
             초음파 센서를 사용하기 위한 클래스를 정의함.

             주의: HC-SR04 초음파 센서는 5V 전압을 사용하므로 STM32F4 보드의 5V 핀을 사용해야 함. 5VC 핀을 사용하도록.
*/
#ifndef __HCSR04_H__
#define __HCSR04_H__

#include "debug.h"

class HCSR04
{
public:
    HCSR04(GPIO_TypeDef *echo_port, uint16_t echo_pin, GPIO_TypeDef *trig_port, uint16_t trig_pin)
    {
        this->echo_port = echo_port;
        this->echo_pin = echo_pin;
        this->trig_port = trig_port;
        this->trig_pin = trig_pin;

        GPIO_InitTypeDef GPIO_InitStruct = {0};

        GPIO_InitStruct.Pin = echo_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(echo_port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = trig_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(trig_port, &GPIO_InitStruct);
    }

    float read_cm()
    {
        send_trig();
        if (wait_echo())
        {
            return read_echo() / 58.0f;
        }
        return 0;
    }

private:
    GPIO_TypeDef *trig_port;
    uint16_t trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t echo_pin;

    void send_trig()
    {
        Timer timer;
        HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_SET);
        timer.start();
        while (timer.read_us() < 10)
        {
            // DO NOTHING
        }
        HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
    }

    bool wait_echo()
    {
        Timer timer;
        float timeout_ms = 1000;
        timer.start();
        while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_RESET)
        {
            if (timer.read_ms() > timeout_ms)
            {
                debug("[WARN] HCSR04 ECHO waiting timeout\n");
                return false;
            }
        }
        timer.stop();
        return true;
    }

    float read_echo()
    {
        Timer timer;
        float timeout_ms = 2000;
        timer.start();
        while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_SET)
        {
            if (timer.read_ms() > timeout_ms)
            {
                debug("[WARN] HCSR04 ECHO reading timeout\n");
                return 0;
            }
        }
        timer.stop();
        return timer.read_us();
    }
};

#endif // __HCSR04_H__
