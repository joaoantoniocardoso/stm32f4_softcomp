#ifndef DWT_H_
#define DWT_H_

#include <stm32f4xx_hal.h>

#define DWT_GetValue() (DWT->CYCCNT)
#define DWT_SetValue(x) (DWT->CYCCNT = x)
#define DWT_Reset(x) (DWT->CYCCNT = 0)
#define DWT_Disable() (DWT->CTRL &= DWT_CTRL_CYCCNTENA_Msk)

uint32_t DWT_Enable();

#endif /* DWT_H_ */
