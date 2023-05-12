/*
* @Author: nhantt
* @Date:   2020-02-02
* @Last Modified by:   nhantt
* @Last Modified time: 2020-02-02
*/

#include "oled.h"
#include "u8g2.h"
#include "u8x8.h"

extern I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

u8x8_t u8x8;
u8g2_t u8g2;

uint32_t oledRefesh;

static uint8_t u8g2_gpio_and_delay_cb(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr);
static uint8_t u8x8_byte_stm32hal_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
// static void MX_I2C1_Init(void);
static void MX_DMA_Init(void);
static uint8_t u8x8_gpio_and_delay_cm3(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
static uint8_t u8x8_byte_hw_i2c_cm3(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
void OLED_Init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
  MX_DMA_Init();
  // MX_I2C1_Init();

  //u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8g2_gpio_and_delay_cb);
  u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_stm32hal_hw_i2c, u8g2_gpio_and_delay_cb);
  //u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_stm32hal_hw_i2c, u8g2_gpio_and_delay_cb);
  //u8g2_Setup_sh1106_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_i2c, u8x8_stm32_gpio_and_delay);
  //u8g2_Setup_sh1106_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_i2c, u8x8_stm32_gpio_and_delay);
  
  //u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_hw_i2c_cm3, u8x8_gpio_and_delay_cm3);
  u8g2_SetI2CAddress(&u8g2, I2C_ADDRESS*2);
	u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0); // wake up display
  u8g2_ClearDisplay(&u8g2);
  u8g2_ClearBuffer(&u8g2);
  oledRefesh = HAL_GetTick();
}

/* Set percent backlight */
void Set_Backlight(uint8_t percent)
{
  if((percent < 0) || (percent > 100)) return;
  TIM1->CCR1 = (uint32_t)(percent*10);
}

void OLED_Refresh(void)
{
	if(HAL_GetTick() - oledRefesh > 100)
	{
		u8g2_SendBuffer(&u8g2);
    oledRefesh = HAL_GetTick();
	}
}

static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  printf("I2C Error\n");
}

static uint8_t u8x8_gpio_and_delay_cm3(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
  switch(msg) {
  case U8X8_MSG_GPIO_AND_DELAY_INIT:
    MX_I2C1_Init();  /* Init I2C communication */
    break;

  default:
    u8x8_SetGPIOResult(u8x8, 1);
    break;
  }

  return 1;
}

/* I2C hardware transfer based on u8x8_byte.c implementation */
static uint8_t u8x8_byte_hw_i2c_cm3(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
  static uint8_t buffer[32];   /* u8g2/u8x8 will never send more than 32 bytes */
  static uint8_t buf_idx;
  uint8_t *data;

  switch(msg) {
  case U8X8_MSG_BYTE_SEND:
    data = (uint8_t *)arg_ptr;
    while(arg_int > 0) {
      buffer[buf_idx++] = *data;
      data++;
      arg_int--;
    }
    break;
  case U8X8_MSG_BYTE_INIT:
    break;
  case U8X8_MSG_BYTE_SET_DC:
    break;
  case U8X8_MSG_BYTE_START_TRANSFER:
    buf_idx = 0;
    break;
  case U8X8_MSG_BYTE_END_TRANSFER:
    //i2c_transfer7(I2C1, 0x3C, buffer, buf_idx, NULL, 0);
    HAL_I2C_Master_Transmit(&hi2c1, 0x3C<<1, buffer, buf_idx, 1000);
    break;
  default:
    return 0;
  }
  return 1;
}

static uint8_t u8g2_gpio_and_delay_cb(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
      /* only support for software I2C*/
      break;
    case U8X8_MSG_DELAY_NANO:
      /* not required for SW I2C*/
      {
        volatile uint32_t i;
        for(i = 1; i <= arg_int*10; i++);
      }
      break;
    case U8X8_MSG_DELAY_10MICRO:
      break;
    case U8X8_MSG_DELAY_100NANO:
      break;
    case U8X8_MSG_DELAY_MILLI:
      HAL_Delay(arg_int);
      break;
    case U8X8_MSG_DELAY_I2C:
      break;
    case U8X8_MSG_GPIO_I2C_CLOCK:
      break;
   /*
    case U8X8_MSG_GPIO_MENU_SELECT:
      u8x8_SetGPIOResult(u8x8, Chip_GPIO_GetPinState(LPC_GPIO, KEY_SELECT_PORT, KEY_SELECT_PIN));
      break;
    case U8X8_MSG_GPIO_MENU_NEXT:
      u8x8_SetGPIOResult(u8x8, Chip_GPIO_GetPinState(LPC_GPIO, KEY_NEXT_PORT, KEY_NEXT_PIN));
      break;
    case U8X8_MSG_GPIO_MENU_PREV:
      u8x8_SetGPIOResult(u8x8, Chip_GPIO_GetPinState(LPC_GPIO, KEY_PREV_PORT, KEY_PREV_PIN));
      break;
    case U8X8_MSG_GPIO_MENU_HOME:
      u8x8_SetGPIOResult(u8x8, Chip_GPIO_GetPinState(LPC_GPIO, KEY_HOME_PORT, KEY_HOME_PIN));
      break;
*/
    default:
      u8x8_SetGPIOResult(u8x8, 1);
      break;
  }
  return 1;
}

static uint8_t u8x8_byte_stm32hal_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  static uint8_t buffer[32];    /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
  static uint8_t buf_idx;
  uint8_t *data;


  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
    {
        data = (uint8_t *)arg_ptr;
        while( arg_int > 0 )
        {
      buffer[buf_idx++] = *data;
      data++;
      arg_int--;
        }
    }
      break;
    case U8X8_MSG_BYTE_INIT:
      break;
    case U8X8_MSG_BYTE_SET_DC:
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
    {
      buf_idx = 0;
    }
    break;
    case U8X8_MSG_BYTE_END_TRANSFER:
    {
      uint8_t iaddress = I2C_ADDRESS;
      //HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)iaddress<<1, buffer, buf_idx);
      //HAL_I2C_Mem_Write_DMA(&hi2c1, (uint16_t)iaddress<<1, 0x40, 1, buffer, sizeof(buffer));
      HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)iaddress<<1, &buffer[0], buf_idx, 1000);
      //TODO Investigate why delay is needed here.
      //Seems like DMA feeding bytes too fast.
      volatile uint32_t i;
      for (i = 1; i <= 500; i++);
    }
      break;
    default:
      return 0;
  }
  return 1;
}