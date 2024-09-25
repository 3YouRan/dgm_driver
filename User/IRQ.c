//
// Created by 陈瑜 on 24-9-18.
//
#include "all.h"

/*
 * @brief CAN接收数据回调函数
 *
 * @param hcan CAN句柄
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
            case 0: {//停机指令返回报文
                if (!(int32_t) ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]))
                    printf("DGM_Motor_CAN:Disabled\r\n");
                break;
            }//1 00111 00000
            case 1: {//使能指令返回报文
                if (!(int32_t) ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]))
                    printf("DGM_Motor_CAN:Enabled\r\n");
                break;
            }
            case 12: {//设置零点指令返回报文
                if (!(int32_t) ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]))
                    printf("DGM_Motor_CAN:Set home successfully!\r\n");
                break;
            }
            case 14: {//获得状态字指令返回报文
                printf("DGM_Motor_CAN:"
                       "电机使能:%d\r\n"
                       "目标达到：%d\r\n"
                       "电流限制：%d\r\n", rx_data[0] & 0x01, (rx_data[0] >> 1) & 0x01, (rx_data[0] >> 1) & 0x01
                );
                printf("DGM_Motor_CAN:ERROR Word:%x\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 15: {//获得电机错误信息返回报文
                printf("DGM_Motor_CAN:ERROR Word:%x\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 16: {//获得力矩指令返回报文
                printf("DGM_Motor_CAN:torque:%.4f\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 17: {//获得速度指令返回报文
                printf("DGM_Motor_CAN:Speed:%.4f\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 18: {//获得位置指令返回报文
                printf("DGM_Motor_CAN:Position:%.4f\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 19: {//获得电流指令返回报文
                printf("DGM_Motor_CAN:Current:%.4f\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 20: {//获得母线电压指令返回报文
                printf("DGM_Motor_CAN:VBus:%.4f\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 21: {//获得母线电流指令返回报文
                printf("DGM_Motor_CAN:IBus:%.4f\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 22: {//获得功率指令返回报文
                printf("DGM_Motor_CAN:Power:%.4f\r\n",
                       ((rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]));
                break;
            }
            case 63: {//心跳报文
                printf("DGM_Motor_CAN:Error:Heatbeat \r\n");
                break;
            }
        }

    }


}
/*
 * @brief UART接收数据回调函数
 *
 * @param huart UART句柄
 * @return None
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


    if(huart==&huart4)
    {
        RxLine++;                      //姣忔帴鏀跺埌涓?涓暟鎹紝杩涘叆鍥炶皟鏁版嵁闀垮害鍔?1
        DataBuff[RxLine-1]=RxBuffer[0];  //鎶婃瘡娆℃帴鏀跺埌鐨勬暟鎹繚瀛樺埌缂撳瓨鏁扮粍
        if(RxBuffer[0]=='!')           //鎺ユ敹缁撴潫鏍囧織浣?
        {
            printf("RXLen=%d\r\n",RxLine);
            for(int i=0;i<RxLine;i++)
                printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);
//            xEventGroupSetBitsFromISR(g_xEventGroup_Uart_Rx,(1<<0),NULL);

            USART_PID_Adjust(1);//鏁版嵁瑙ｆ瀽鍜屽弬鏁拌祴鍊煎嚱鏁?
//
            memset(DataBuff,0,sizeof(DataBuff));  //娓呯┖缂撳瓨鏁扮粍
            RxLine=0;  //娓呯┖鎺ユ敹闀垮害
//            xSemaphoreGiveFromISR(g_SemaphoreHandle_For_Pid, NULL);//使用信号量唤醒UART_Rx任务

        }
        RxBuffer[0]=0;
        HAL_UART_Receive_IT(&huart4, (uint8_t *)RxBuffer, 1);//姣忔帴鏀朵竴涓暟鎹紝灏辨墦寮?涓?娆′覆鍙ｄ腑鏂帴鏀?
    }
}


/**
  * @brief 非阻塞模式下的 Period elapsed 回调
  * @note 当 TIM4 中断发生时，在
  * HAL_TIM_IRQHandler（） 的它直接调用 HAL_IncTick（） 以递增
  * 用作应用程序时基的全局变量 “uwTick”。
  * @param htim ： TIM 句柄
  * @retval 无
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
////            xSemaphoreGiveFromISR(g_SemaphoreHandle_For_Pid, NULL);//使用信号量唤醒PID任务
//        }
//    }
//    /* USER CODE END Callback 1 */
//}