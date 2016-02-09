/*
 * ReadSensors.h
 *
 *  Created on: 2015. okt. 8.
 *      Author: Csabi
 */

#ifndef INCLUDE_READSENSORS_H_
#define INCLUDE_READSENSORS_H_


#include "stm32f4xx_hal.h"
#include <string>
#include <stdlib.h>


int ReadFrontLeft();
int ReadFrontRight();
int ReadFrontMiddle();
void ReadSensors();
void ReadSensorsDummy();
void ADC1_read();
void ADC2_read();
void ControlTaskDelay(int us);
void EnableDrivers();
void DisableDrivers();
void SetLeds(uint16_t pattern);
void ShiftLeds(uint8_t amount);
void LATCHLeds();

void EnableMUX();
void DisableMUX();
void SetMUX(uint8_t);



#endif /* INCLUDE_READSENSORS_H_ */
