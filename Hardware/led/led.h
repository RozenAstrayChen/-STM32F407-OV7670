#ifndef __LED_H
#define __LED_H
#include "stm32f4xx.h"


#define On      0
#define Off     1
#define LED0(x) (x?(GPIO_SetBits(GPIOF,GPIO_Pin_9)):(GPIO_ResetBits(GPIOF,GPIO_Pin_9)))
#define LED1(x) (x?(GPIO_SetBits(GPIOF,GPIO_Pin_10)):(GPIO_ResetBits(GPIOF,GPIO_Pin_10)))

void LED_GPIO_Conf(void);

#endif
