#ifndef INC_MYLIBRARY_H_
#define INC_MYLIBRARY_H_

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

extern TIM_HandleTypeDef htim2;

#define TriggerDuration 2

uint16_t distance, triggerTime, sensor;
GPIO_TypeDef *triggerPorts[1];
uint16_t triggerPins[1];
GPIO_TypeDef *echoPorts[1];
uint16_t echoPins[1];

void SysTickEnable();
void SysTickDisable();
uint16_t measureDistance(GPIO_TypeDef *triggerPort, uint16_t triggerPin, GPIO_TypeDef *echoPort, uint16_t echoPin);

#endif /* INC_MYLIBRARY_H_ */
