#include "myLibrary.h"

uint16_t distance = 0, triggerTime = 0, sensor = 0;
GPIO_TypeDef *triggerPorts[1] = {GPIOD};
uint16_t triggerPins[1] = {GPIO_PIN_11};
GPIO_TypeDef *echoPorts[1] = {GPIOD};
uint16_t echoPins[1] = {GPIO_PIN_14};

void SysTickEnable()
{
	__disable_irq();
	SysTick->CTRL |= (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);
	__enable_irq();
}

void SysTickDisable()
{
	__disable_irq();
	SysTick->CTRL &= ~(SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);
	__enable_irq();
}

uint16_t measureDistance(GPIO_TypeDef *triggerPort, uint16_t triggerPin, GPIO_TypeDef *echoPort, uint16_t echoPin)
{
	SysTickDisable();
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_GPIO_WritePin(triggerPort, triggerPin, GPIO_PIN_SET);
	triggerTime = 0;//reset the variable
	asm ("nop");//to avoid program freezing
	while(triggerTime < TriggerDuration);
	HAL_GPIO_WritePin(triggerPort, triggerPin, GPIO_PIN_RESET);
	while(!HAL_GPIO_ReadPin(echoPort, echoPin));
	distance = 0;//reset the variable
	while(HAL_GPIO_ReadPin(echoPort, echoPin));
	HAL_TIM_Base_Stop_IT(&htim2);
	SysTickEnable();
	return distance;
}
