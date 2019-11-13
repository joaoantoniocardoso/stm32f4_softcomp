/*
 * leds_and_buttons.h
 *
 *  Created on: Jul 3, 2019
 *      Author: joaoantoniocardoso
 */

#ifndef LEDS_AND_BUTTONS_H_
#define LEDS_AND_BUTTONS_H_

#include <stm32f4xx_hal.h>

// Leds macros
// LED0 -> green    right
// LED1 -> orange   up
// LED2 -> red      left
// LED3 -> blue     down
#define LED0_on()     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET)
#define LED1_on()     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET)
#define LED2_on()     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET)
#define LED3_on()     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET)
#define LED0_off()    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET)
#define LED1_off()    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED2_off()    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET)
#define LED3_off()    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET)
#define LED0_tg()     HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) ? LED0_on() : LED0_off()
#define LED1_tg()     HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) ? LED1_on() : LED1_off()
#define LED2_tg()     HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) ? LED2_on() : LED2_off()
#define LED3_tg()     HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) ? LED3_on() : LED3_off()

// Buttons Macros
#define BT0_on()      (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
#define BT0_off()     (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)

#endif /* LEDS_AND_BUTTONS_H_ */
