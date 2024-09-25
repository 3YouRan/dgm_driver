//
// Created by ��� on 24-9-18.
//
#include "all.h"

/*
 * @brief CAN�������ݻص�����
 *
 * @param hcan CAN���
 *
 * @return None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){

    uint8_t CMD_index;
    CAN_RxHeaderTypeDef rx_header;
    uint8_t             rx_data[8];
    if(hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data
        printf("CAN_BUS_get_id:%X\r\n",rx_header.StdId);
        CMD_index=rx_header.StdId&0x3F;
        switch(CMD_index) {
            case 0: {//ͣ��ָ��ر���
                if (!(int32_t) ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]))
                    printf("DGM_Motor_CAN:Disabled\r\n");
                break;
            }//1 00111 00000
            case 1: {//ʹ��ָ��ر���
                if (!(int32_t) ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]))
                    printf("DGM_Motor_CAN:Enabled\r\n");
                break;
            }
            case 12: {//�������ָ��ر���
                if (!(int32_t) ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]))
                    printf("DGM_Motor_CAN:Set home successfully!\r\n");
                break;
            }
            case 14: {//���״̬��ָ��ر���
                printf("DGM_Motor_CAN:"
                       "���ʹ��:%d\r\n"
                       "Ŀ��ﵽ��%d\r\n"
                       "�������ƣ�%d\r\n", rx_data[0] & 0x01, (rx_data[0] >> 1) & 0x01, (rx_data[0] >> 1) & 0x01
                );
                printf("DGM_Motor_CAN:ERROR Word:%x\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 15: {//��õ��������Ϣ���ر���
                printf("DGM_Motor_CAN:ERROR Word:%x\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 16: {//�������ָ��ر���
                printf("DGM_Motor_CAN:torque:%.4f\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 17: {//����ٶ�ָ��ر���
                printf("DGM_Motor_CAN:Speed:%.4f\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 18: {//���λ��ָ��ر���
                printf("DGM_Motor_CAN:Position:%.4f\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 19: {//��õ���ָ��ر���
                printf("DGM_Motor_CAN:Current:%.4f\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 20: {//���ĸ�ߵ�ѹָ��ر���
                printf("DGM_Motor_CAN:VBus:%.4f\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 21: {//���ĸ�ߵ���ָ��ر���
                printf("DGM_Motor_CAN:IBus:%.4f\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 22: {//��ù���ָ��ر���
                printf("DGM_Motor_CAN:Power:%.4f\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 63: {//��������
                printf("DGM_Motor_CAN:Error:Heatbeat \r\n");
                break;
            }
        }

    }


}
/*
 * @brief UART�������ݻص�����
 *
 * @param huart UART���
 * @return None
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


    if(huart==&huart4)
    {
        RxLine++;                      //每接收到�?个数据，进入回调数据长度�?1
        DataBuff[RxLine-1]=RxBuffer[0];  //把每次接收到的数据保存到缓存数组
        if(RxBuffer[0]=='!')           //接收结束标志�?
        {
            printf("RXLen=%d\r\n",RxLine);
            for(int i=0;i<RxLine;i++)
                printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);
//            xEventGroupSetBitsFromISR(g_xEventGroup_Uart_Rx,(1<<0),NULL);

            USART_PID_Adjust(1);//数据解析和参数赋值函�?
//
            memset(DataBuff,0,sizeof(DataBuff));  //清空缓存数组
            RxLine=0;  //清空接收长度
//            xSemaphoreGiveFromISR(g_SemaphoreHandle_For_Pid, NULL);//ʹ���ź�������UART_Rx����

        }
        RxBuffer[0]=0;
        HAL_UART_Receive_IT(&huart4, (uint8_t *)RxBuffer, 1);//每接收一个数据，就打�?�?次串口中断接�?
    }
}


/**
  * @brief ������ģʽ�µ� Period elapsed �ص�
  * @note �� TIM4 �жϷ���ʱ����
  * HAL_TIM_IRQHandler���� ����ֱ�ӵ��� HAL_IncTick���� �Ե���
  * ����Ӧ�ó���ʱ����ȫ�ֱ��� ��uwTick����
  * @param htim �� TIM ���
  * @retval ��
  */
//uint8_t time1=0;
//
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    /* USER CODE BEGIN Callback 0 */
//
//    /* USER CODE END Callback 0 */
//    if (htim->Instance == TIM4) {
//        HAL_IncTick();
//    }
//    /* USER CODE BEGIN Callback 1 */
//    if(htim->Instance==TIM3){
//        time1++;
//        angle_last=angle_now;
//        angle_now = transfer(gm3508_1.rotor_angle, 0, 8191, -pi/19, pi/19);
//        if(angle_now-angle_last>(pi/19)){
//            angle_total+=(angle_now-angle_last-2*pi/19);
//        }else if(angle_now-angle_last<(-pi/19))
//        {
//            angle_total+=(angle_now-angle_last+2*pi/19);
//        }else{
//            angle_total+=(angle_now-angle_last);
//        }
//
//        if(time1==10){
//            time1=0;
//            if((Target_Speed-Target_Speed_actual)>MIN_Spe_Increment){
//               Target_Speed_actual+=MIN_Spe_Increment;
//           } else if((Target_Speed-Target_Speed_actual)<-MIN_Spe_Increment){
//               Target_Speed_actual-=MIN_Spe_Increment;
//           }
////            xSemaphoreGiveFromISR(g_SemaphoreHandle_For_Pid, NULL);//ʹ���ź�������PID����
//        }
//    }
//    /* USER CODE END Callback 1 */
//}