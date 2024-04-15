#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the public defines of the application.
*/

// MADE FOR chip model: stm32c011f4p


/* Includes ------------------------------------------------------------------*/
#include "stm32c0xx_hal.h"      // included by default
#include <stdint.h>




/* Public function prototypes -----------------------------------------------*/
void SystemClock_Config();

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler();

// position fetchers
uint64_t motorA_current_position(void);
uint64_t motorB_current_position(void);

// position updaters
void motorA_update_position(uint64_t new_pos);
void motorB_update_position(uint64_t new_pos);








#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
