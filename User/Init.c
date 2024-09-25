//
// Created by ³Âè¤ on 24-9-18.
//
#include "all.h"

void my_Init() {
    RetargetInit(&huart4);
    HAL_UART_Receive_IT(&huart4, (uint8_t *)RxBuffer, 1);
    CAN_Filter_Mask_Config_32bit(&hcan1,CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE,0x201,0);
    CAN_Init(&hcan1);
    PID_Init(&pid_speed,&pid_position);
//    HAL_TIM_Base_Start_IT(&htim3);
}