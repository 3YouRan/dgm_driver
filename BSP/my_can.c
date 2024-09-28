//
// Created by ��� on 24-6-5.
//

#include "all.h"
//#include "all.h"

DJI_motor gm3508_1;

/*
 * @brief CAN��ʼ������
 * @param hcan CAN���
 *
 * @return None
 * */
void CAN_Init(CAN_HandleTypeDef *hcan){
    HAL_CAN_Start(hcan);//����CAN
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);//ʹ�ܽ���FIFO0�ж�
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);//ʹ�ܽ���FIFO1�ж�
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}
/*
 * @brief CAN�������ݺ���
 *
 * @param hcan CAN���
 * @param Std_Id ��׼֡ID (Std_Id=0��ʾ��ʹ�ñ�׼֡ID)
 * @param Ext_Id ��չ֡ID (Ext_Id=0��ʾ��ʹ����չ֡ID)
 * @param *data ����ָ��
 * @param len ���ݳ���
 *
 * @return ���ͳɹ�����0������ʧ�ܷ���1
 * */
uint8_t CAN_Transmit_DATA(CAN_HandleTypeDef *hcan, uint16_t Std_Id, uint32_t Ext_Id,uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef TX_Header;
    uint32_t used_mailbox=CAN_TX_MAILBOX0;

    //���ؼ�����
    assert_param(hcan!= NULL);
    if(Std_Id){
        TX_Header.StdId = Std_Id;//��׼֡ID
        TX_Header.ExtId = 0;//��չ֡ID����ʹ��
    } else if(Ext_Id){
        TX_Header.StdId = 0;//��׼֡ID����ʹ��
        TX_Header.ExtId = Ext_Id;//��չ֡ID
    }
    TX_Header.IDE=CAN_ID_STD;//��׼֡����ʹ����չ֡
    TX_Header.RTR=CAN_RTR_DATA;//��׼֡����ʹ��
    TX_Header.DLC=len;//���ݳ���

    return (HAL_CAN_AddTxMessage(hcan, &TX_Header, data, &used_mailbox));
}

/*
 * @brief GM6020_��ѹ���ú���
 *
 * @param hcan CAN���
 * @param volt ��ѹֵ
 * @param motor_ID ���ID
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
 * @brief GM3508_��ѹ���ú���
 *
 * @param hcan CAN���
 * @param volt ����ֵ
 * @param motor_ID ���ID
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
 * @brief ����CAN���˲���(32λ����ģʽ)
 *
 * @param hcan CAN���
 * @param Object_Para �˲������|FIFOx|ID����|֡���� (ʹ������:CAN_FILTER(0)|CAN_FIFO_1|CAN_STDID|CAN_DATA_TYPE)
 * @param ID ���յ�֡ID(32bit)
 * @param Mask_ID ����ID(0x7FF,0x1FFFFFFF)
 *
 * @return None
 * */
void CAN_Filter_Mask_Config_32bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID){

    CAN_FilterTypeDef CAN_Filter_Init_Structure;

    //���ؼ�����
    assert_param(hcan!= NULL);
    if ((Object_Para & 0x02)){
        //��չ֡
        //��չ��ʽID�ĸ�16bit
        CAN_Filter_Init_Structure.FilterMaskIdHigh = ID<<3>>16;//�ó�Ա����ֻ�и�16bit��������
        //��չ��ʽID�ĵ�16bit
        CAN_Filter_Init_Structure.FilterMaskIdLow = ID<<3|((Object_Para&0x03)<<1);//�ó�Ա����ֻ�е�16bit��������
        //�����16bit
        CAN_Filter_Init_Structure.FilterIdHigh = Mask_ID<<3>>16;
        //�����16bit
        CAN_Filter_Init_Structure.FilterIdLow = Mask_ID<<3|((Object_Para&0x03)<<1);
    }
        //0000 0010 0000 0101  <<3  0001 0000 0010 1000
    else{
        //��׼֡
        //ID�ĸ�16bit
        CAN_Filter_Init_Structure.FilterIdHigh = ID<<5;
        //ID�ĵ�16bit
        CAN_Filter_Init_Structure.FilterIdLow = ((Object_Para&0x03)<<1);
        //�����16bit
        CAN_Filter_Init_Structure.FilterMaskIdHigh = Mask_ID<<5;
        //�����16bit
        CAN_Filter_Init_Structure.FilterMaskIdLow = ((Object_Para&0x03)<<1);
    }
    //�˲������,0~27,28���˲���,can1(0~13),can2(14~27)
    CAN_Filter_Init_Structure.FilterBank=Object_Para>>3;
    //�˲����󶨵�FIFOx,ֻ�ܰ�һ��
    CAN_Filter_Init_Structure.FilterFIFOAssignment=(Object_Para>>2)&0x01;
    //ʹ���˲���
    CAN_Filter_Init_Structure.FilterActivation = ENABLE;
    //�˲���ģʽ������ID����ģʽ
    CAN_Filter_Init_Structure.FilterMode = CAN_FILTERMODE_IDMASK;
    //32λ�˲���
    CAN_Filter_Init_Structure.FilterScale = CAN_FILTERSCALE_32BIT;
    //�ӻ�ģʽѡ��ʼ��Ԫ
    CAN_Filter_Init_Structure.SlaveStartFilterBank=14;

    HAL_CAN_ConfigFilter(hcan, &CAN_Filter_Init_Structure);
}

/*
 * @brief ����CAN���˲���(32λ�б�ģʽ)
 *
 * @param hcan CAN���
 * @param Object_Para �˲������|FIFOx|ID����|֡���� (ʹ������:CAN_FILTER(0)|CAN_FIFO_1|CAN_STDID|CAN_DATA_TYPE)
 * @param ID_1 ���յĵ�һ��֡ID(32bit)
 * @param ID_2 ���յĵڶ���֡ID(32bit)
 *
 * @return None
 * */
void CAN_Filter_List_Config_32bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID_1, uint32_t ID_2){

    CAN_FilterTypeDef CAN_Filter_Init_Structure;

    //���ؼ�����
    assert_param(hcan!= NULL);
    if ((Object_Para & 0x01)){
        //��չ֡
        //�����ID�ĸ�16bit
        CAN_Filter_Init_Structure.FilterMaskIdHigh = ID_1<<3>>16;
        //�����ID�ĵ�16bit
        CAN_Filter_Init_Structure.FilterMaskIdLow = ID_1<<3|((Object_Para&0x03)<<2);
        //ID����ֵ��16bit
        CAN_Filter_Init_Structure.FilterIdHigh = ID_2<<3>>16;
        //ID����ֵ��16bit
        CAN_Filter_Init_Structure.FilterIdLow = ID_2<<3|((Object_Para&0x03)<<2);
    }
        //0000 0010 0000 0101  <<3  0001 0000 0010 1000
    else{
        //��׼֡
        //ID�ĸ�16bit
        CAN_Filter_Init_Structure.FilterIdHigh = ID_1<<5;
        //ID�ĵ�16bit
        CAN_Filter_Init_Structure.FilterIdLow = ((Object_Para&0x03)<<1);
        //�����16bit
        CAN_Filter_Init_Structure.FilterMaskIdHigh = ID_2<<5;
        //�����16bit
        CAN_Filter_Init_Structure.FilterMaskIdLow = ((Object_Para&0x03)<<1);
    }
    //�˲������,0~27,28���˲���,can1(0~13),can2(14~27)
    CAN_Filter_Init_Structure.FilterBank=Object_Para>>3;
    //�˲����󶨵�FIFOx,ֻ�ܰ�һ��
    CAN_Filter_Init_Structure.FilterFIFOAssignment=(Object_Para>>2)&0x01;
    //ʹ���˲���
    CAN_Filter_Init_Structure.FilterActivation = ENABLE;
    //�˲���ģʽ������ID����ģʽ
    CAN_Filter_Init_Structure.FilterMode = CAN_FILTERMODE_IDLIST;
    //32λ�˲���
    CAN_Filter_Init_Structure.FilterScale = CAN_FILTERSCALE_32BIT;
    //�ӻ�ģʽѡ��ʼ��Ԫ
    CAN_Filter_Init_Structure.SlaveStartFilterBank=14;

    HAL_CAN_ConfigFilter(hcan, &CAN_Filter_Init_Structure);
}
/*
 * @brief ����CAN���˲���(16λ����ģʽ)
 *
 * @param hcan CAN���
 * @param Object_Para �˲������|FIFOx|ID����|֡���� (ʹ������:CAN_FILTER(0)|CAN_FIFO_1|CAN_STDID|CAN_DATA_TYPE) ID����ֻ������CAN_STDID
 * @param ID ���յ�֡ID��(��16bitһ������16bitһ������:(0x205<<16)|(0x201))
 * @param Mask_ID_set ������(��16bitһ������16bitһ������:(0x7FF<<16)|(0x444))
 *
 * @return None
 * */
void CAN_Filter_Mask_Config_16bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID_set, uint32_t Mask_ID_set){

    CAN_FilterTypeDef CAN_Filter_Init_Structure;

    //����CAN_STDID|CAN_DATA_TYPE��˳��
    unsigned int bit1 = (Object_Para >> 0) & 1;  // �Ͷ˵�1λ
    unsigned int bit2 = (Object_Para >> 1) & 1;  // �Ͷ˵�2λ

    // �������λ
    Object_Para &= ~((1 << 0) | (1 << 1));

    // ��������λ�������ǷŻ�ȥ
    Object_Para |= (bit1 << 1) | (bit2 << 0);

    //���ؼ�����
    assert_param(hcan!= NULL);
    assert_param(!(Object_Para & 0x02)!= 0);

    if ((Object_Para & 0x01)){
        //��չ֡
        //����ĸ�16bit
        CAN_Filter_Init_Structure.FilterMaskIdHigh = ID_set>>16<<5|(Object_Para&0x03)<<3;
        //����ĵ�16bit
        CAN_Filter_Init_Structure.FilterMaskIdLow = ID_set<<5|(Object_Para&0x03)<<3;
        //ID��16bit��
        CAN_Filter_Init_Structure.FilterIdHigh = Mask_ID_set>>16<<5|(Object_Para&0x03)<<3;
        //ID��16bit
        CAN_Filter_Init_Structure.FilterIdLow = Mask_ID_set<<5|((Object_Para&0x03)<<3);
    }
    //�˲������,0~27,28���˲���,can1(0~13),can2(14~27)
    CAN_Filter_Init_Structure.FilterBank=Object_Para>>3;
    //�˲����󶨵�FIFOx,ֻ�ܰ�һ��
    CAN_Filter_Init_Structure.FilterFIFOAssignment=(Object_Para>>2)&0x01;
    //ʹ���˲���
    CAN_Filter_Init_Structure.FilterActivation = ENABLE;
    //�˲���ģʽ������ID����ģʽ
    CAN_Filter_Init_Structure.FilterMode = CAN_FILTERMODE_IDMASK;
    //32λ�˲���
    CAN_Filter_Init_Structure.FilterScale = CAN_FILTERSCALE_16BIT;
    //�ӻ�ģʽѡ��ʼ��Ԫ
    CAN_Filter_Init_Structure.SlaveStartFilterBank=14;

    HAL_CAN_ConfigFilter(hcan, &CAN_Filter_Init_Structure);
}
/*
 * @brief ����CAN���˲���(16λ�б�ģʽ)
 *
 * @param hcan CAN���
 * @param Object_Para �˲������|FIFOx|ID����|֡���� (ʹ������:CAN_FILTER(0)|CAN_FIFO_1|CAN_STDID|CAN_DATA_TYPE) ID����ֻ������CAN_STDID
 * @param ID_set1 ���յĵ�һ��֡ID(��16bit��һ��,��16bit��һ������:(0x205<<16)|(0x201))
 * @param ID_set2 ���յĵڶ���֡ID(��16bit��һ��,��16bit��һ������:(0x205<<16)|(0x201))
 *
 * @return None
 * */
void CAN_Filter_List_Config_16bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID_set1, uint32_t ID_set2){

    CAN_FilterTypeDef CAN_Filter_Init_Structure;

    //����CAN_STDID|CAN_DATA_TYPE��˳��
    unsigned int bit1 = (Object_Para >> 0) & 1;  // �Ͷ˵�1λ
    unsigned int bit2 = (Object_Para >> 1) & 1;  // �Ͷ˵�2λ

    // �������λ
    Object_Para &= ~((1 << 0) | (1 << 1));

    // ��������λ�������ǷŻ�ȥ
    Object_Para |= (bit1 << 1) | (bit2 << 0);

    //���ؼ�����
    assert_param(hcan!= NULL);
    assert_param(!(Object_Para & 0x02)!= 0);

    if (!(Object_Para & 0x02)){
        //����ĸ�16bit
        CAN_Filter_Init_Structure.FilterMaskIdHigh = ID_set1>>16<<5|(Object_Para&0x03)<<3;
        //����ĵ�16bit
        CAN_Filter_Init_Structure.FilterMaskIdLow = ID_set1<<5|(Object_Para&0x03)<<3;
        //ID��16bit��
        CAN_Filter_Init_Structure.FilterIdHigh = ID_set2>>16<<5|(Object_Para&0x03)<<3;
        //ID��16bit
        CAN_Filter_Init_Structure.FilterIdLow = ID_set2<<5|((Object_Para&0x03)<<3);
    }
    //�˲������,0~27,28���˲���,can1(0~13),can2(14~27)
    CAN_Filter_Init_Structure.FilterBank=Object_Para>>3;
    //�˲����󶨵�FIFOx,ֻ�ܰ�һ��
    CAN_Filter_Init_Structure.FilterFIFOAssignment=(Object_Para>>2)&0x01;
    //ʹ���˲���
    CAN_Filter_Init_Structure.FilterActivation = ENABLE;
    //�˲���ģʽ������ID����ģʽ
    CAN_Filter_Init_Structure.FilterMode = CAN_FILTERMODE_IDLIST;
    //32λ�˲���
    CAN_Filter_Init_Structure.FilterScale = CAN_FILTERSCALE_16BIT;
    //�ӻ�ģʽѡ��ʼ��Ԫ
    CAN_Filter_Init_Structure.SlaveStartFilterBank=14;

    HAL_CAN_ConfigFilter(hcan, &CAN_Filter_Init_Structure);
}
