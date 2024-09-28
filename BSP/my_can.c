//
// Created by 陈瑜 on 24-6-5.
//

#include "all.h"
//#include "all.h"

DJI_motor gm3508_1;

/*
 * @brief CAN初始化函数
 * @param hcan CAN句柄
 *
 * @return None
 * */
void CAN_Init(CAN_HandleTypeDef *hcan){
    HAL_CAN_Start(hcan);//开启CAN
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);//使能接收FIFO0中断
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);//使能接收FIFO1中断
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}
/*
 * @brief CAN发送数据函数
 *
 * @param hcan CAN句柄
 * @param Std_Id 标准帧ID (Std_Id=0表示不使用标准帧ID)
 * @param Ext_Id 扩展帧ID (Ext_Id=0表示不使用扩展帧ID)
 * @param *data 数据指针
 * @param len 数据长度
 *
 * @return 发送成功返回0，发送失败返回1
 * */
uint8_t CAN_Transmit_DATA(CAN_HandleTypeDef *hcan, uint16_t Std_Id, uint32_t Ext_Id,uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef TX_Header;
    uint32_t used_mailbox=CAN_TX_MAILBOX0;

    //检测关键传参
    assert_param(hcan!= NULL);
    if(Std_Id){
        TX_Header.StdId = Std_Id;//标准帧ID
        TX_Header.ExtId = 0;//扩展帧ID，不使用
    } else if(Ext_Id){
        TX_Header.StdId = 0;//标准帧ID，不使用
        TX_Header.ExtId = Ext_Id;//扩展帧ID
    }
    TX_Header.IDE=CAN_ID_STD;//标准帧，不使用扩展帧
    TX_Header.RTR=CAN_RTR_DATA;//标准帧，不使用
    TX_Header.DLC=len;//数据长度

    return (HAL_CAN_AddTxMessage(hcan, &TX_Header, data, &used_mailbox));
}

/*
 * @brief GM6020_电压设置函数
 *
 * @param hcan CAN句柄
 * @param volt 电压值
 * @param motor_ID 电机ID
 *
 * @return None
 * */
void GM6020_Voltage_Set(CAN_HandleTypeDef *hcan, int16_t volt,uint8_t motor_ID){
    uint8_t tx_data[8];
    if (motor_ID == 1) {
        tx_data[0] = (volt>>8)&0xff;
        tx_data[1] = (volt)&0xff;
    }

    CAN_Transmit_DATA(hcan,0x202,0,tx_data,8);

}
/*
 * @brief GM3508_电压设置函数
 *
 * @param hcan CAN句柄
 * @param volt 电流值
 * @param motor_ID 电机ID
 *
 * @return None
 * */
void GM3508_Current_Set(CAN_HandleTypeDef *hcan, int16_t Current,uint16_t id,uint8_t motor_ID){
    uint8_t tx_data[8];
    if (motor_ID == 1||motor_ID==5) {
        tx_data[0] = (Current>>8)&0xff;
        tx_data[1] = (Current)&0xff;
    }
    if (motor_ID == 2||motor_ID==6) {
        tx_data[2] = (Current>>8)&0xff;
        tx_data[3] = (Current)&0xff;
    }
    if (motor_ID == 3||motor_ID==7) {
        tx_data[4] = (Current>>8)&0xff;
        tx_data[5] = (Current)&0xff;
    }
    if (motor_ID == 4||motor_ID==8) {
        tx_data[6] = (Current>>8)&0xff;
        tx_data[7] = (Current)&0xff;
    }
    CAN_Transmit_DATA(hcan,id,0,tx_data,8);

}
/*
 * @brief 配置CAN的滤波器(32位掩码模式)
 *
 * @param hcan CAN句柄
 * @param Object_Para 滤波器编号|FIFOx|ID类型|帧类型 (使用例子:CAN_FILTER(0)|CAN_FIFO_1|CAN_STDID|CAN_DATA_TYPE)
 * @param ID 接收的帧ID(32bit)
 * @param Mask_ID 屏蔽ID(0x7FF,0x1FFFFFFF)
 *
 * @return None
 * */
void CAN_Filter_Mask_Config_32bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID){

    CAN_FilterTypeDef CAN_Filter_Init_Structure;

    //检测关键传参
    assert_param(hcan!= NULL);
    if ((Object_Para & 0x02)){
        //扩展帧
        //扩展格式ID的高16bit
        CAN_Filter_Init_Structure.FilterMaskIdHigh = ID<<3>>16;//该成员变量只有高16bit发挥作用
        //扩展格式ID的低16bit
        CAN_Filter_Init_Structure.FilterMaskIdLow = ID<<3|((Object_Para&0x03)<<1);//该成员变量只有低16bit发挥作用
        //掩码高16bit
        CAN_Filter_Init_Structure.FilterIdHigh = Mask_ID<<3>>16;
        //掩码低16bit
        CAN_Filter_Init_Structure.FilterIdLow = Mask_ID<<3|((Object_Para&0x03)<<1);
    }
        //0000 0010 0000 0101  <<3  0001 0000 0010 1000
    else{
        //标准帧
        //ID的高16bit
        CAN_Filter_Init_Structure.FilterIdHigh = ID<<5;
        //ID的低16bit
        CAN_Filter_Init_Structure.FilterIdLow = ((Object_Para&0x03)<<1);
        //掩码高16bit
        CAN_Filter_Init_Structure.FilterMaskIdHigh = Mask_ID<<5;
        //掩码低16bit
        CAN_Filter_Init_Structure.FilterMaskIdLow = ((Object_Para&0x03)<<1);
    }
    //滤波器编号,0~27,28个滤波器,can1(0~13),can2(14~27)
    CAN_Filter_Init_Structure.FilterBank=Object_Para>>3;
    //滤波器绑定到FIFOx,只能绑定一个
    CAN_Filter_Init_Structure.FilterFIFOAssignment=(Object_Para>>2)&0x01;
    //使能滤波器
    CAN_Filter_Init_Structure.FilterActivation = ENABLE;
    //滤波器模式，设置ID掩码模式
    CAN_Filter_Init_Structure.FilterMode = CAN_FILTERMODE_IDMASK;
    //32位滤波器
    CAN_Filter_Init_Structure.FilterScale = CAN_FILTERSCALE_32BIT;
    //从机模式选择开始单元
    CAN_Filter_Init_Structure.SlaveStartFilterBank=14;

    HAL_CAN_ConfigFilter(hcan, &CAN_Filter_Init_Structure);
}

/*
 * @brief 配置CAN的滤波器(32位列表模式)
 *
 * @param hcan CAN句柄
 * @param Object_Para 滤波器编号|FIFOx|ID类型|帧类型 (使用例子:CAN_FILTER(0)|CAN_FIFO_1|CAN_STDID|CAN_DATA_TYPE)
 * @param ID_1 接收的第一个帧ID(32bit)
 * @param ID_2 接收的第二个帧ID(32bit)
 *
 * @return None
 * */
void CAN_Filter_List_Config_32bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID_1, uint32_t ID_2){

    CAN_FilterTypeDef CAN_Filter_Init_Structure;

    //检测关键传参
    assert_param(hcan!= NULL);
    if ((Object_Para & 0x01)){
        //扩展帧
        //掩码后ID的高16bit
        CAN_Filter_Init_Structure.FilterMaskIdHigh = ID_1<<3>>16;
        //掩码后ID的低16bit
        CAN_Filter_Init_Structure.FilterMaskIdLow = ID_1<<3|((Object_Para&0x03)<<2);
        //ID掩码值高16bit
        CAN_Filter_Init_Structure.FilterIdHigh = ID_2<<3>>16;
        //ID掩码值低16bit
        CAN_Filter_Init_Structure.FilterIdLow = ID_2<<3|((Object_Para&0x03)<<2);
    }
        //0000 0010 0000 0101  <<3  0001 0000 0010 1000
    else{
        //标准帧
        //ID的高16bit
        CAN_Filter_Init_Structure.FilterIdHigh = ID_1<<5;
        //ID的低16bit
        CAN_Filter_Init_Structure.FilterIdLow = ((Object_Para&0x03)<<1);
        //掩码高16bit
        CAN_Filter_Init_Structure.FilterMaskIdHigh = ID_2<<5;
        //掩码低16bit
        CAN_Filter_Init_Structure.FilterMaskIdLow = ((Object_Para&0x03)<<1);
    }
    //滤波器编号,0~27,28个滤波器,can1(0~13),can2(14~27)
    CAN_Filter_Init_Structure.FilterBank=Object_Para>>3;
    //滤波器绑定到FIFOx,只能绑定一个
    CAN_Filter_Init_Structure.FilterFIFOAssignment=(Object_Para>>2)&0x01;
    //使能滤波器
    CAN_Filter_Init_Structure.FilterActivation = ENABLE;
    //滤波器模式，设置ID掩码模式
    CAN_Filter_Init_Structure.FilterMode = CAN_FILTERMODE_IDLIST;
    //32位滤波器
    CAN_Filter_Init_Structure.FilterScale = CAN_FILTERSCALE_32BIT;
    //从机模式选择开始单元
    CAN_Filter_Init_Structure.SlaveStartFilterBank=14;

    HAL_CAN_ConfigFilter(hcan, &CAN_Filter_Init_Structure);
}
/*
 * @brief 配置CAN的滤波器(16位掩码模式)
 *
 * @param hcan CAN句柄
 * @param Object_Para 滤波器编号|FIFOx|ID类型|帧类型 (使用例子:CAN_FILTER(0)|CAN_FIFO_1|CAN_STDID|CAN_DATA_TYPE) ID类型只可输入CAN_STDID
 * @param ID 接收的帧ID组(高16bit一个，低16bit一个，如:(0x205<<16)|(0x201))
 * @param Mask_ID_set 掩码组(高16bit一个，低16bit一个，如:(0x7FF<<16)|(0x444))
 *
 * @return None
 * */
void CAN_Filter_Mask_Config_16bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID_set, uint32_t Mask_ID_set){

    CAN_FilterTypeDef CAN_Filter_Init_Structure;

    //交换CAN_STDID|CAN_DATA_TYPE的顺序
    unsigned int bit1 = (Object_Para >> 0) & 1;  // 低端第1位
    unsigned int bit2 = (Object_Para >> 1) & 1;  // 低端第2位

    // 清除这两位
    Object_Para &= ~((1 << 0) | (1 << 1));

    // 交换这两位并将它们放回去
    Object_Para |= (bit1 << 1) | (bit2 << 0);

    //检测关键传参
    assert_param(hcan!= NULL);
    assert_param(!(Object_Para & 0x02)!= 0);

    if ((Object_Para & 0x01)){
        //扩展帧
        //掩码的高16bit
        CAN_Filter_Init_Structure.FilterMaskIdHigh = ID_set>>16<<5|(Object_Para&0x03)<<3;
        //掩码的低16bit
        CAN_Filter_Init_Structure.FilterMaskIdLow = ID_set<<5|(Object_Para&0x03)<<3;
        //ID高16bit即
        CAN_Filter_Init_Structure.FilterIdHigh = Mask_ID_set>>16<<5|(Object_Para&0x03)<<3;
        //ID低16bit
        CAN_Filter_Init_Structure.FilterIdLow = Mask_ID_set<<5|((Object_Para&0x03)<<3);
    }
    //滤波器编号,0~27,28个滤波器,can1(0~13),can2(14~27)
    CAN_Filter_Init_Structure.FilterBank=Object_Para>>3;
    //滤波器绑定到FIFOx,只能绑定一个
    CAN_Filter_Init_Structure.FilterFIFOAssignment=(Object_Para>>2)&0x01;
    //使能滤波器
    CAN_Filter_Init_Structure.FilterActivation = ENABLE;
    //滤波器模式，设置ID掩码模式
    CAN_Filter_Init_Structure.FilterMode = CAN_FILTERMODE_IDMASK;
    //32位滤波器
    CAN_Filter_Init_Structure.FilterScale = CAN_FILTERSCALE_16BIT;
    //从机模式选择开始单元
    CAN_Filter_Init_Structure.SlaveStartFilterBank=14;

    HAL_CAN_ConfigFilter(hcan, &CAN_Filter_Init_Structure);
}
/*
 * @brief 配置CAN的滤波器(16位列表模式)
 *
 * @param hcan CAN句柄
 * @param Object_Para 滤波器编号|FIFOx|ID类型|帧类型 (使用例子:CAN_FILTER(0)|CAN_FIFO_1|CAN_STDID|CAN_DATA_TYPE) ID类型只可输入CAN_STDID
 * @param ID_set1 接收的第一组帧ID(高16bit填一个,低16bit填一个，如:(0x205<<16)|(0x201))
 * @param ID_set2 接收的第二组帧ID(高16bit填一个,低16bit填一个，如:(0x205<<16)|(0x201))
 *
 * @return None
 * */
void CAN_Filter_List_Config_16bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID_set1, uint32_t ID_set2){

    CAN_FilterTypeDef CAN_Filter_Init_Structure;

    //交换CAN_STDID|CAN_DATA_TYPE的顺序
    unsigned int bit1 = (Object_Para >> 0) & 1;  // 低端第1位
    unsigned int bit2 = (Object_Para >> 1) & 1;  // 低端第2位

    // 清除这两位
    Object_Para &= ~((1 << 0) | (1 << 1));

    // 交换这两位并将它们放回去
    Object_Para |= (bit1 << 1) | (bit2 << 0);

    //检测关键传参
    assert_param(hcan!= NULL);
    assert_param(!(Object_Para & 0x02)!= 0);

    if (!(Object_Para & 0x02)){
        //掩码的高16bit
        CAN_Filter_Init_Structure.FilterMaskIdHigh = ID_set1>>16<<5|(Object_Para&0x03)<<3;
        //掩码的低16bit
        CAN_Filter_Init_Structure.FilterMaskIdLow = ID_set1<<5|(Object_Para&0x03)<<3;
        //ID高16bit即
        CAN_Filter_Init_Structure.FilterIdHigh = ID_set2>>16<<5|(Object_Para&0x03)<<3;
        //ID低16bit
        CAN_Filter_Init_Structure.FilterIdLow = ID_set2<<5|((Object_Para&0x03)<<3);
    }
    //滤波器编号,0~27,28个滤波器,can1(0~13),can2(14~27)
    CAN_Filter_Init_Structure.FilterBank=Object_Para>>3;
    //滤波器绑定到FIFOx,只能绑定一个
    CAN_Filter_Init_Structure.FilterFIFOAssignment=(Object_Para>>2)&0x01;
    //使能滤波器
    CAN_Filter_Init_Structure.FilterActivation = ENABLE;
    //滤波器模式，设置ID掩码模式
    CAN_Filter_Init_Structure.FilterMode = CAN_FILTERMODE_IDLIST;
    //32位滤波器
    CAN_Filter_Init_Structure.FilterScale = CAN_FILTERSCALE_16BIT;
    //从机模式选择开始单元
    CAN_Filter_Init_Structure.SlaveStartFilterBank=14;

    HAL_CAN_ConfigFilter(hcan, &CAN_Filter_Init_Structure);
}
