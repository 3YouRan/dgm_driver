//
// Created by 陈瑜 on 24-9-25.
//


#include "all.h"
/*
 * @brief DGM_Motor_Disable
 *
 * @param motor_id 电机ID
 *
 * @return None
 */
void DGM_Motor_Disable(uint8_t motor_id){

    CAN_Transmit_DATA(&hcan1,motor_id<<6|0x000,NULL,0);

}
/*
 * @brief DGM_Motor_Enable
 *
 * @param motor_id 电机ID
 *
 * @return None
 */
void DGM_Motor_Enable(uint8_t motor_id){

    CAN_Transmit_DATA(&hcan1,motor_id<<6|0x001,NULL,0);
}
/*
 * @brief DGM_Motor_Set_Torque
 *
 * @param motor_id 电机ID
 * @param torque 力矩 Nm
 *
 * @return None
 */
void DGM_Motor_Set_Torque(uint8_t motor_id,float *torque){

    CAN_Transmit_DATA(&hcan1,motor_id<<6|0x002,(uint8_t*)torque,4);

}
/*
 * @brief DGM_Motor_Set_Speed
 *
 * @param motor_id 电机ID
 * @param speed 速度 r/s
 *
 * @return None
 */
void DGM_Motor_Set_Speed(uint8_t motor_id,float *speed){

    CAN_Transmit_DATA(&hcan1,motor_id<<6|0x003,(uint8_t*)speed,4);

}
/*
 * @brief DGM_Motor_Set_Position
 *
 * @param motor_id 电机ID
 * @param position 位置 r
 *
 * @return None
 */
void DGM_Motor_Set_Position(uint8_t motor_id,float *position) {
    CAN_Transmit_DATA(&hcan1,motor_id<<6|0x004,(uint8_t*)position,4);
}
/*
 * @brief DGM_Motor_Sync 运动控制指令同步信号 node_id设置为0x1F表示广播
 *
 * @param None
 *
 * @return None
 */
void DGM_Motor_Sync(void){
    CAN_Transmit_DATA(&hcan1,0x1F<<6|0x005,NULL,0);
}
/*
 * @brief DGM_Motor_Set_home 设定电机的零点位置
 *
 * @param None
 *
 * @return None
 */
void DGM_Motor_Set_home(uint8_t motor_id){
    CAN_Transmit_DATA(&hcan1,motor_id<<6|0x00C,NULL,0);
}
/*
 * @brief DGM_Motor_ERR_Reset 复位电机的错误状态
 *
 * @param motor_id 电机ID
 *
 * @return None
 */
void DGM_Motor_ERR_Reset(uint8_t motor_id){
    CAN_Transmit_DATA(&hcan1,motor_id<<6|0x00D,NULL,0);
}
/*
 * @brief DGM_Motor_Get_StatusWord 获取电机的状态字
 *
 * @param motor_id 电机ID
 *
 * @return None
 */
void DGM_Motor_Get_StatusWord(uint8_t motor_id){
    CAN_Transmit_DATA(&hcan1,motor_id<<6|0x00E,NULL,0);
}
/*
 * @brief DGM_Motor_Get_Torque 获取电机的力矩
 *
 * @param motor_id 电机ID
 *
 * @return None
 */
void DGM_Motor_Get_Torque(uint8_t motor_id){
    CAN_Transmit_DATA(&hcan1,motor_id<<6|0x010,NULL,0);
}
/*
 * @brief DGM_Motor_Get_Speed 获取电机的速度
 *
 * @param motor_id 电机ID
 *
 * @return None
 */
void DGM_Motor_Get_Speed(uint8_t motor_id){
    CAN_Transmit_DATA(&hcan1,motor_id<<6|0x011,NULL,0);
}
/*
 * @brief DGM_Motor_Get_Position 获取电机的位置
 *
 * @param motor_id 电机ID
 *
 * @return None
 */
void DGM_Motor_Get_Position(uint8_t motor_id){
    CAN_Transmit_DATA(&hcan1,motor_id<<6|0x012,NULL,0);
}
/*
 * @brief DGM_Motor_Get_Current 获取电机的电流
 *
 * @param motor_id 电机ID
 *
 * @return None
 */
void DGM_Motor_Get_Current(uint8_t motor_id){
    CAN_Transmit_DATA(&hcan1,motor_id<<6|0x013,NULL,0);
}
/*
 * @brief DGM_Motor_Get_VBus 获取电机的母线电压
 *
 * @param motor_id 电机ID
 *
 * @return None
 */
void DGM_Motor_Get_VBus(uint8_t motor_id){
    CAN_Transmit_DATA(&hcan1,motor_id<<6|0x014,NULL,0);
}
/*
 * @brief DGM_Motor_Get_IBus 获取电机的母线电流
 *
 * @param motor_id 电机ID
 *
 * @return None
 */
void DGM_Motor_Get_IBus(uint8_t motor_id){
    CAN_Transmit_DATA(&hcan1,motor_id<<6|0x015,NULL,0);
}
/*
 * @brief DGM_Motor_Get_Power 获取电机的功率
 *
 * @param motor_id 电机ID
 *
 * @return None
 */
void DGM_Motor_Get_Power(uint8_t motor_id){
    CAN_Transmit_DATA(&hcan1,motor_id<<6|0x016,NULL,0);
}
/*
 * @brief DGM_Motor_Heartbeat 心跳命令
 *
 * @param motor_id 电机ID
 *
 * @return None
 */
void DGM_Motor_Heartbeat(uint8_t motor_id){
    CAN_Transmit_DATA(&hcan1,motor_id<<6|63,NULL,0);
}
