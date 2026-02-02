#ifndef CAN_H
#define CAN_H

#include "stdio.h"

void CAN_Init(void);
void CAN_SendFrame(uint32_t id, uint8_t *data);

#endif