//
// Created by 陈瑜 on 24-6-5.
//

#ifndef CAN_DEMO01_MY_CAN_H
#define CAN_DEMO01_MY_CAN_H

#include "main.h"
//滤波器编号
#define CAN_FILTER(x) ((x)<<3)
//接收队列
#define CAN_FIFO_0 (0<<2)
#define CAN_FIFO_1 (1<<2)
//标准帧或扩展帧
#define CAN_STDID (0<<1)
#define CAN_EXTID (1<<1)

//数据帧或遥控帧
#define CAN_DATA_TYPE (0<<0)
#define CAN_REMOTE_TYPE (1<<0)

//gm6020结构体
typedef struct DJI_motor{
    uint16_t can_id;//电机ID
    int16_t  set_voltage;//设定的电压值
    uint16_t rotor_angle;//机械角度
    int16_t  rotor_speed;//转速
    int16_t  torque_current;//扭矩电流
    uint8_t  temp;//温度
}DJI_motor;

void CAN_Init(CAN_HandleTypeDef *hcan);
uint8_t CAN_Transmit_DATA(CAN_HandleTypeDef *hcan, uint16_t Std_Id,uint32_t Ext_Id, uint8_t *data, uint8_t len);
void CAN_Filter_Mask_Config_32bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);
void CAN_Filter_List_Config_32bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID_1, uint32_t ID_2);
void CAN_Filter_Mask_Config_16bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID_set, uint32_t Mask_ID_set);
void CAN_Filter_List_Config_16bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID_1, uint32_t ID_2);
void GM6020_Voltage_Set(CAN_HandleTypeDef *hcan, int16_t volt,uint8_t motor_ID);
void GM3508_Current_Set(CAN_HandleTypeDef *hcan, int16_t Current,uint16_t id,uint8_t motor_ID);



#endif //CAN_DEMO01_MY_CAN_H

