#ifndef __RS485_H_
#define __RS485_H_

#include "stm32g4xx_hal_def.h"
// RS485 方向控制：写/读
#define RS485_EN_W HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET)
#define RS485_EN_R HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET)

#endif
