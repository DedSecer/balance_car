#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"
#include "can.h"

extern void can_filter(CAN_HandleTypeDef* hcan);
extern void can_filter_init(void);

#endif
