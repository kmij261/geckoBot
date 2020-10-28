/**
  ***********************************UTF-8**************************************
  * @file    robotTask.c
  * @author  Xiong
  * @version V0.1
  * @date    26-Oct-2020
  * @brief   此文件用于定义机器人运行中使用的任务函数
  ******************************************************************************  
  */ 
#ifndef _ROBOT_TASK_H
#define _ROBOT_TASK_H

#include "stdbool.h"
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "delay.h"
#include "usart.h"
#include "led.h"
#include "gyro.h"
#include "beep.h"
#include "nrf24l01p.h"
#include "dynamixel.h"
#include "controller.h"
#include "timeCalib.h"
#include "dataComm.h"
#include "trajectoryPlanning.h"

int RT_StartTask( bool inOfflineMode );


#endif
