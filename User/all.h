//
// Created by 陈瑜 on 24-9-16.
//

#ifndef RC_WORK1_ALL_H
#define RC_WORK1_ALL_H

#include "retarget.h"

#include "PID_Adjust.h"

#include "string.h"

#include "main.h"

#include "PID_Adjust.h"

#include "PID.h"

#include "can.h"

#include "usart.h"


#include "my_can.h"

#include "FreeRTOS.h"

#include "task.h"

#include "semphr.h"

#include "Application.h"

#include <stdbool.h>

#include "dgm_driver.h"
extern float Target_Speed;
extern float Target_Speed_actual;
extern float MIN_Spe_Increment;
extern float Target_Position;
extern float Target_Current;
extern PID pid_position;
extern PID pid_speed;
extern PID pid_current;


extern uint8_t RxBuffer[1];
extern uint16_t RxLine;
extern uint8_t DataBuff[200];


extern DJI_motor gm3508_1;
extern float angle_last, angle_now, angle_total;
extern int16_t current;

extern TaskHandle_t g_uart_tx_task_handle;

extern void my_Init();
#define pi 3.1416


#endif //RC_WORK1_ALL_H
