/**
  ***********************************UTF-8**************************************
  * @file    robotTask.c
  * @author  Xiong
  * @version V0.1
  * @date    26-Oct-2020
  * @brief   此文件用于定义机器人运行中使用的任务函数
  ******************************************************************************  
  */ 
  
  
#include "robotTask.h"



static const uint32_t ServoWaitCount = 12000;
static bool statusGot = false;

 
static TaskHandle_t t_sendHandler;		//
static TaskHandle_t t_recvHandler;		// 
static TaskHandle_t t_ledHandler;		//
static TaskHandle_t t_servoHandler;		//
static TaskHandle_t t_PowerSupplyHandler;


static const uint16_t interval =  100;
	
static int dur;
	

static void* tSendTask(void)
{
	uint8_t systemUpdateSeq = 0;
	uint32_t waitCount = 0;
	uint16_t servoErrCount = 0;
	int ret, count, seq;
	uint32_t systick1, systick2, totalTime, systickLoad;
	
	while(1)
	{
		LED4 = !LED4;
		systick1 = SysTick->VAL;
		ret = DC_UpdateSendBuf( systemUpdateSeq );
		if(!ret)
		{
			NRF_SetMode( MODE_TX );
			DC_Send();
			NRF_SetMode( MODE_RX );
			if(systemUpdateSeq == eMsgSystemInfo)
				systemUpdateSeq = seq;
			else
				systemUpdateSeq = ( systemUpdateSeq + 1 ) % 5;
		}

		systick2 = SysTick->VAL;
		totalTime += ( ( systick1 - systick2 + systickLoad ) % systickLoad ) / 21;
		
		count++;
		if( count >= 10 )
		{
			totalTime /= 10;
			g_errStat[ 0 ] = errCount & 0xFF;
			g_errStat[ 1 ] = errCount >> 8;
			g_errStat[ 2 ] = servoErrCount & 0xFF;
			g_errStat[ 3 ] = servoErrCount >> 8;
			g_errStat[ 4 ] = totalTime & 0xFF;
			g_errStat[ 5 ] = totalTime >> 8;
			g_errStat[ 6 ] = totalTime >> 16;
			g_errStat[ 7 ] = totalTime >> 24;
			
			errCount = 0;
			servoErrCount = 0;
			totalTime = 0;
			count = 0;
			seq = systemUpdateSeq;
			systemUpdateSeq = 5;
		}
		
		vTaskDelay( 50 );
	}

}


static void* tRecvTask(void)
{
	uint8_t status;
	int ret;
	while( 1 )
	{
		if( nrfIrqTriggered )
		{
			nrfIrqTriggered = false;
			
			status = NRF_Read_Reg( nrfREG_STATUS );
			printf("status: %02X\r\n", status);
			NRF_Write_Reg(nrfREG_STATUS, status);
			if( status & nrfRX_OK )
			{
				NRF_Write_Reg( nrfREG_STATUS, status );
				ret = NRF_Read_Reg( nrfCMD_R_RX_PLD_WID );
				if(ret)
					ret = DC_Recv( ret );
			}
		}
		
		vTaskDelay(0);
	}
	
}

static void* tServoCtrlTask(void)
{
	
	while(1)
	{
		
		TP_TripodGait();
		vTaskDelay( interval );
		
	}
	
}

static void* tLedTask(void)
{
	while(1)
	{
		LED0 = ~LED0;
		vTaskDelay( 500 );
	}
}

static void* tSystemStatusTask( void )
{
	static int stackLeftBuf[4];
	while( 1 )
	{
		
//		stackLeftBuf[ 0 ] = ( int )uxTaskGetStackHighWaterMark( t_BodyStatusHandler );
//		stackLeftBuf[ 1 ] = ( int )uxTaskGetStackHighWaterMark( t_ServoCtrlHandler );
//		stackLeftBuf[ 2 ] = ( int )uxTaskGetStackHighWaterMark( t_LedHandler );
//		stackLeftBuf[ 3 ] = ( int )uxTaskGetStackHighWaterMark( t_SystemStatusHandler );
		
		printf( "stack left: read:%d\t\tctrl: %d\t\tled: %d\t\tmemQuery: %d\r\n", 
			stackLeftBuf[ 0 ], stackLeftBuf[ 1 ], stackLeftBuf[ 2 ], stackLeftBuf[ 3 ] );
		
		vTaskDelay( 1000 );
	}
}



