//
// Created by 陈瑜 on 24-9-25.
//

#ifndef DGM_DRIVER_DGM_DRIVER_H
#define DGM_DRIVER_DGM_DRIVER_H
#include "main.h"

void DGM_Motor_Disable(uint8_t motor_id);
void DGM_Motor_Enable(uint8_t motor_id);
void DGM_Motor_Set_Torque(uint8_t motor_id,float *torque);
void DGM_Motor_Set_Speed(uint8_t motor_id,float *speed);
void DGM_Motor_Set_Position(uint8_t motor_id,float *position);
void DGM_Motor_Sync(void);
void DGM_Motor_Set_home(uint8_t motor_id);
void DGM_Motor_ERR_Reset(uint8_t motor_id);
void DGM_Motor_Get_StatusWord(uint8_t motor_id);
void DGM_Motor_Get_Torque(uint8_t motor_id);
void DGM_Motor_Get_Speed(uint8_t motor_id);
void DGM_Motor_Get_Position(uint8_t motor_id);
void DGM_Motor_Get_Current(uint8_t motor_id);
void DGM_Motor_Get_VBus(uint8_t motor_id);
void DGM_Motor_Get_IBus(uint8_t motor_id);
void DGM_Motor_Get_Power(uint8_t motor_id);
void DGM_Motor_Heartbeat(uint8_t motor_id);
#endif //DGM_DRIVER_DGM_DRIVER_H
