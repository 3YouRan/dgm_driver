//
// Created by 陈瑜 on 24-9-25.
//
#include "all.h"
void can_tx_task(void *arg){
    float speed = 0;
    float Position = 5;
    vTaskDelay(1000);
    DGM_Motor_Enable(1);
    vTaskDelay(500);

    while(1) {

        for (int i = 0; i < 30; ++i) {
            vTaskDelay(100);
            speed += 0.1;
            DGM_Motor_Set_Speed(1, &speed);
            vTaskDelay(100);
        }
        for (int i = 0; i < 30; ++i) {
            vTaskDelay(100);
            speed -= 0.1;
            DGM_Motor_Set_Speed(1, &speed);
            vTaskDelay(100);
        }//00001 00 1111
        //00111 00 1111
    }
}