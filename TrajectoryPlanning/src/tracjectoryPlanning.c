/**
  ***********************************UTF-8**************************************
  * @file    trajectoryPlanning.c
  * @author  Xiong
  * @version V1.0
  * @date    10-Aug-2020
  * @brief   此文件用于定义机器人步态轨迹生成函数
  ******************************************************************************  
  */ 

#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "trajectoryPlanning.h"
#include "controller.h"
#include "usart.h"
#include "delay.h"
#include "dynamixel.h"

static const double StepHeightLimit[ 2 ] = { 5.0, 50.0 };
static const double StepLenLimit[ 2 ] = { 0.0, 60.0 };
static const uint16_t PeriodLimit[ 2 ] = { 50, 500 };

	
static uint16_t s_period = 300;
static uint16_t s_periodSeg;
static double s_stepHeight = 40.0;
static double s_stepLen = 70.0;
static uint16_t s_currentSeq = 0;
static int speed;

static Robot_t s_tRobot;


/* ---------------------------------------------------------------------------*/

/****
	* @brief	初始化模块
    * @param  	无
	* @retval 	0:初始化成功，非0：初始化失败
    */

int TP_Init( void )
{
	uint8_t i = 0;
	s_tRobot.pTipPos = calloc( ctrlLEG_COUNT, sizeof( Point3d ) );
	if( s_tRobot.pTipPos == NULL )
		return 1;
	
	s_tRobot.pLegAngle = calloc( ctrlLEG_COUNT, sizeof( LegAngle_t ) );
	if( s_tRobot.pLegAngle == NULL )
		return 1;
	
	s_periodSeg = s_period / ctrlLEG_COUNT;

	return 0;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	读取当前舵机位置，运动到起始位置
    * @param  	无
	* @retval 	运动后当前序列号
    */
void TP_MoveToInitPos( void )
{
	uint8_t i, seq;
	uint32_t pos[ctrlSERVO_NUM];
	
	while( 1 )
	{
		memset( g_ucaServoRxBuffer, 0, MAX_BUF_SIZE );
		DXL_GetRegState( dxlREG_Goal_Position, 4 );
		delay_ms( 10 );
		
		if( xServoMsg.bDataReady )
			break;
	}
	
	if ( DXL_GetPresentParam( &xServoMsg ) <= 0 )
		return ;
	
	for( seq=0; seq<s_period; seq++ )
	{
		for( i=0; i<ctrlSERVO_NUM; i++ )
		{
			pos[i] = (dServoStatusBuf[dxlCOL_Pos][i]*(s_period-seq) + seq*2048) / s_period;
		}
	}
	
}

/* ---------------------------------------------------------------------------*/

/****
	* @brief	根据当前序列号在周期中的位置，控制机器人运动到指定位置
    * @param  	无
	* @retval 	运动后当前序列号
    */
int TP_TripodGait( void )
{
	uint8_t legInSwingPhase;
	uint8_t i;
	
	
	legInSwingPhase = s_currentSeq / ( s_period / ctrlLEG_COUNT );
	for( i=0; i<ctrlLEG_COUNT; i++ )
	{
		if( i == legInSwingPhase )
		{
			s_tRobot.pTipPos[i].x = 0.0;
			s_tRobot.pTipPos[i].y =  -0.25 * i * s_stepLen
				+ 0.75 * s_stepLen*(s_currentSeq - i*s_periodSeg )/(s_periodSeg-1);
			s_tRobot.pTipPos[i].z = 
				s_stepHeight * sin((s_currentSeq - i*s_periodSeg )*PI/(s_periodSeg-1));
		}
		
		else
		{
			s_tRobot.pTipPos[i].x = 0;
			s_tRobot.pTipPos[i].y -= s_stepLen / s_period;
			s_tRobot.pTipPos[i].z = 0;
		}
	}

	CTRL_SetTipsPos( s_tRobot.pTipPos );
	if( s_currentSeq < s_period-1 )
		s_currentSeq++;
	else
		s_currentSeq = 0;
	
	return s_currentSeq;
}







