
#include "stm32f4xx_hal.h"
#include "main.h"
#include "sx1272-hal.h"

#ifndef INC_DOTMAT_H_
#define INC_DOTMAT_H_

#define SER_PIN  GPIO_PIN_8
#define RCK_PIN  GPIO_PIN_9
#define SRCK_PIN GPIO_PIN_10


uint8_t SCR_DEFAULT[8] = {
		0b00000000,
		0b11100111,
		0b01000010,
		0b00000000,
		0b00000000,
		0b00111100,
		0b00000000,
		0b00000000
};

uint8_t SCR_SEE_LEFT[8] = {
		0b00000000,
		0b01110111,
		0b00010001,
		0b00000000,
		0b00011000,
		0b00011000,
		0b00000000,
		0b00000000
};

uint8_t SCR_SEE_RIGHT[8] = {
		0b00000000,
		0b11101110,
		0b10001000,
		0b00000000,
		0b00110000,
		0b00110000,
		0b00000000,
		0b00000000
};

uint8_t SCR_COLD_SMILE[8] = {
		0b00000000,
		0b11101111,
		0b00100000,
		0b00100000,
		0b00100000,
		0b01000000,
		0b00111110,
		0b00000000
};

uint8_t SCR_CREEPER[8] = {
		0b00000000,
		0b01100110,
		0b01100110,
		0b00000000,
		0b00011000,
		0b00111100,
		0b00100100,
		0b00000000
};



uint8_t SCR_ALL_OFF[8] = {
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00
};

uint8_t SCR_ALL_ON[8] = {
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF
};

void shiftOut(GPIO_TypeDef* dataPort, uint16_t dataPin,
              GPIO_TypeDef* clockPort, uint16_t clockPin,
              uint8_t val);
void updateDisplay(uint8_t screen[]);
void allOFF(void);
void allON(void);



void shiftOut(GPIO_TypeDef* dataPort, uint16_t dataPin,
              GPIO_TypeDef* clockPort, uint16_t clockPin,
              uint8_t val) {
  for (int i = 0; i < 8; i++) {
    GPIO_PinState pinState;

    pinState = (val & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET; // LSB부터 전송

    HAL_GPIO_WritePin(dataPort, dataPin, pinState); // 데이터 전송
    HAL_GPIO_WritePin(clockPort, clockPin, GPIO_PIN_SET); // 클럭 상승 에지
    HAL_GPIO_WritePin(clockPort, clockPin, GPIO_PIN_RESET); // 클럭 하강 에지
  }
}



void updateDisplay(uint8_t *screen)
{
//	HAL_GPIO_WritePin(GPIOA, RCK_PIN, GPIO_PIN_RESET);
    for (uint8_t i = 0; i < 8; i++)
    {
    	HAL_GPIO_WritePin(GPIOA, RCK_PIN, GPIO_PIN_RESET);
        shiftOut(GPIOA, SER_PIN, GPIOA, SRCK_PIN, ~screen[i]);  // 열 데이터
        shiftOut(GPIOA, SER_PIN, GPIOA, SRCK_PIN, 1<<i);  // 행 선택 (LOW 활성)
        HAL_GPIO_WritePin(GPIOA, RCK_PIN, GPIO_PIN_SET);
        HAL_Delay(25);
    }
//    HAL_GPIO_WritePin(GPIOA, RCK_PIN, GPIO_PIN_SET);
}

void allOFF(void)
{
    updateDisplay(SCR_ALL_OFF);
}

void allON(void)
{
    updateDisplay(SCR_ALL_ON);
}

#endif /* INC_DOTMAT_H_ */