#ifndef _BSP_CAN_H
#define _BSP_CAN_H


#include "main.h"



typedef CAN_HandleTypeDef hcan_t;

extern void CAN1_Config(void);
extern void CAN2_Config(void);
extern uint8_t canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len , uint16_t ExternID_Flag);



#endif

