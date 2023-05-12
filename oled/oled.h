/*
* @Author: nhantt
* @Date:   2020-03-10
* @Last Modified by:   nhantt
* @Last Modified time: 2020-03-10
*/

#ifndef __OLED_H__
#define __OLED_H__

#include "stm32f1xx_hal.h"
#include "main.h"
#include "u8g2.h"

#define I2C_ADDRESS 0x3C

extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern u8g2_t u8g2;
extern u8x8_t u8x8;
extern uint32_t oledRefesh;

#define LCD_PORT GPIOB
#define LCD_RST_PIN GPIO_PIN_14
#define LCD_CS_PIN	GPIO_PIN_12
#define LCD_CLK_PORT() __HAL_RCC_GPIOB_CLK_ENABLE()

/* define gpio connect lcd display */
#define LCD_RST_1() HAL_GPIO_WritePin(LCD_PORT, LCD_RST_PIN, GPIO_PIN_SET)
#define LCD_RST_0() HAL_GPIO_WritePin(LCD_PORT, LCD_RST_PIN, GPIO_PIN_RESET)
#define LCD_CS_1() HAL_GPIO_WritePin(LCD_PORT, LCD_CS_PIN, GPIO_PIN_SET)
#define LCD_CS_0() HAL_GPIO_WritePin(LCD_PORT, LCD_CS_PIN, GPIO_PIN_RESET)

void OLED_Init(void);
void OLED_Refresh(void);
void Set_Backlight(uint8_t percent);
void Backlight_Off(void);
void Backlight_On();
#endif