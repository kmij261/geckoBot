
#include "stm32f4xx_conf.h"


#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					 
#endif


#include "usart.h"	
#include "dynamixel.h"
#include "led.h"	
#include "dataComm.h"
#include "delay.h"
#include "timeCalib.h"

static volatile uint32_t waitCount = 0;;
static const uint32_t WaitTime = 60000;


static uint16_t bytes_recved = 0;
static bool packHeadFound = false;


/*----------------------------------------------------------------*/
#pragma import(__use_no_semihosting)             
//            
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
// 
void _sys_exit(int x) 
{ 
	x = x; 
} 
//
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0);//
	USART2->DR = (u8) ch;      
	return ch;
}
//
/* ---------------------------------------------------------------------------*/

/****
	* @brief	初始化USART1
    * @param  	ulBaudrate：波特率
    * @retval 	None
    */
void USART2_Init(uint32_t ulBaudrate){
	
   
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);

	GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_2 | GPIO_Pin_3; 
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType 		= GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_UP; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 

	USART_InitStruct.USART_BaudRate 			= ulBaudrate;//
	USART_InitStruct.USART_WordLength 			= USART_WordLength_8b;//
	USART_InitStruct.USART_StopBits 			= USART_StopBits_1;//
	USART_InitStruct.USART_Parity 				= USART_Parity_No;//
	USART_InitStruct.USART_HardwareFlowControl 	= USART_HardwareFlowControl_None;//
	USART_InitStruct.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;	//
	USART_Init(USART2, &USART_InitStruct); 						//

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);							//
	NVIC_InitStructure.NVIC_IRQChannel 						= USART2_IRQn;	//
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 7;				//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0;				//
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;		//
	NVIC_Init(&NVIC_InitStructure);											//

	
	USART_Cmd(USART2, ENABLE);  											//
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);	
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	初始化USART2
    * @param  	ulBaudrate：波特率
    * @retval 	None
    */
void USART1_Init(u32 ulBaudrate)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART1_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); 

	GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF;		
	GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType 		= GPIO_OType_PP; 	
	GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_UP;	 	
	GPIO_Init(GPIOA,&GPIO_InitStructure); 					
					

	USART1_InitStruct.USART_BaudRate 	= ulBaudrate;			
	USART1_InitStruct.USART_WordLength 	= USART_WordLength_8b;	
	USART1_InitStruct.USART_StopBits 	= USART_StopBits_1;		
	USART1_InitStruct.USART_Parity 		= USART_Parity_No;		
	USART1_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;			
	USART1_InitStruct.USART_Mode 		= USART_Mode_Rx | USART_Mode_Tx;	
	USART_Init(USART1, &USART1_InitStruct); 						
	
	NVIC_InitStructure.NVIC_IRQChannel 						= USART1_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 4;				
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0;				
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;		
	NVIC_Init(&NVIC_InitStructure);				

	USART_HalfDuplexCmd(USART1, ENABLE);
	
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);		
	USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);		
//	
	
	USART_Cmd(USART1, ENABLE);  							
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);		
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	
	

}
/* ---------------------------------------------------------------------------*/

void USART1_SetDirIn( void )
{
	USART1->CR1 &= ~(0x11<<2);
	USART1->CR1 |= 1<<2;
}
/* ---------------------------------------------------------------------------*/

void USART1_SetDirOut( void )
{
	USART1->CR1 &= ~(0x11<<2);
	USART1->CR1 |= 1<<3;	
}
/* ---------------------------------------------------------------------------*/

/**
* @brief 	初始化串口1 Tx DMA，高优先级，开启FIFO设定阈值为满，16节拍一次突发
  * @param  None
  * @retval None
  */
void DMA_ServoTxInit( uint32_t ulPeriphAddr, uint32_t ulMemAddr, uint16_t usBufferSize )
{
	DMA_InitTypeDef DMA_InitStruct;						
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);					//使能DMA时钟

	DMA_DeInit(dmaServoTxStream);   										//重置DMA
	
	DMA_InitStruct.DMA_Channel				= DMA_Channel_4;				//DMA通道4
	DMA_InitStruct.DMA_PeripheralBaseAddr 	= ulPeriphAddr;  				//外设基地址
	DMA_InitStruct.DMA_Memory0BaseAddr 		= ulMemAddr;  					//内存基地址
	DMA_InitStruct.DMA_DIR 					= DMA_DIR_MemoryToPeripheral;  	//数据方向：内存到外设
	DMA_InitStruct.DMA_BufferSize 			= usBufferSize;  				//数组大小
	DMA_InitStruct.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;  	//外设地址自增：关
	DMA_InitStruct.DMA_MemoryInc 			= DMA_MemoryInc_Enable;  		//内存地址自增：开
	DMA_InitStruct.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;  //外设数据大小：字节
	DMA_InitStruct.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte; 		//内存字节大小：字节
	DMA_InitStruct.DMA_Mode 				= DMA_Mode_Normal;  			//DMA模式（普通或循环发送）：普通
	DMA_InitStruct.DMA_Priority 			= DMA_Priority_High; 			//DMA优先级：高
	
	DMA_InitStruct.DMA_FIFOMode 			= DMA_FIFOMode_Enable;			//FIFO模式：开启
	DMA_InitStruct.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;		//FIFO阈值：满（4字）
	DMA_InitStruct.DMA_MemoryBurst			= DMA_MemoryBurst_INC16;		//16节拍一次突发
	DMA_InitStruct.DMA_PeripheralBurst		= DMA_PeripheralBurst_Single;	//外设突发节拍：1
	DMA_Init(dmaServoTxStream, &DMA_InitStruct);  
	  	
	NVIC_InitStructure.NVIC_IRQChannel 						= dmaServoTxIRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(dmaServoTxStream, DMA_IT_TC, ENABLE);			//使能dma传输完成中断
	
	DMA_Cmd(dmaServoTxStream, DISABLE);							//初始化完成，先关闭发送DMA
}
/* ---------------------------------------------------------------------------*/

/**
  * @brief 	初始化串口2接收DMA
  * @param  None
  * @retval None
  */
void DMA_ServoRxInit( uint32_t ulPeriphAddr, uint32_t ulMemAddr, uint16_t usBufferSize )
{
	DMA_InitTypeDef DMA_InitStruct;
	DMA_Stream_TypeDef* DMA_Stream = dmaServoRxStream;							
	NVIC_InitTypeDef NVIC_InitStructure;							//最大传送数据量	
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);					//使能DMA时钟

	DMA_DeInit(DMA_Stream);   												//重置DMA
	
	DMA_InitStruct.DMA_Channel				= DMA_Channel_4;				//DMA通道4
	DMA_InitStruct.DMA_PeripheralBaseAddr 	= ulPeriphAddr;  				//外设基地址
	DMA_InitStruct.DMA_Memory0BaseAddr 		= ulMemAddr;  					//内存基地址
	DMA_InitStruct.DMA_DIR 					= DMA_DIR_PeripheralToMemory ;  //数据方向：外设到内存
	DMA_InitStruct.DMA_BufferSize 			= usBufferSize;  				//数组大小
	DMA_InitStruct.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;  	//外设地址自增：关
	DMA_InitStruct.DMA_MemoryInc 			= DMA_MemoryInc_Enable;  		//内存地址自增：开
	DMA_InitStruct.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;  //外设数据大小：字节
	DMA_InitStruct.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte; 		//内存字节大小：字节
	DMA_InitStruct.DMA_Mode 				= DMA_Mode_Normal;  			//DMA模式（普通或循环发送）：普通
	DMA_InitStruct.DMA_Priority 			= DMA_Priority_Medium; 			//DMA优先级：中等
	
	DMA_InitStruct.DMA_FIFOMode 			= DMA_FIFOMode_Disable;			//FIFO模式：
	DMA_InitStruct.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;		//FIFO阈值：满（4字）
	DMA_InitStruct.DMA_MemoryBurst			= DMA_MemoryBurst_Single;		//一个节拍一次突发
	DMA_InitStruct.DMA_PeripheralBurst		= DMA_PeripheralBurst_Single;	//
	
	DMA_Init(DMA_Stream, &DMA_InitStruct);  
	
	NVIC_InitStructure.NVIC_IRQChannel 						= dmaServoRxIRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig( dmaServoRxStream, DMA_IT_TC, ENABLE);			//使能传输完成中断
	
	DMA_Cmd( dmaServoRxStream, DISABLE);							//先关闭串口接收DMA
}
/* ---------------------------------------------------------------------------*/

/**
  * @brief 	使能单次DMA发送
  * @param  usLen：发送的数据长度
  * @retval None
  */
void DMA_SendData( DMA_Stream_TypeDef *DMAy_Streamx, uint16_t usLen )
{	

	packHeadFound = false;
	USART_ITConfig( USART1, USART_IT_IDLE, DISABLE );
	USART1_SetDirOut();
	DMA_Cmd(DMAy_Streamx, DISABLE );  //
 	DMA_SetCurrDataCounter(DMAy_Streamx, usLen);//
 	DMA_Cmd(DMAy_Streamx, ENABLE);  //
	
}
/* ---------------------------------------------------------------------------*/

/**
  * @brief 	使能单次DMA发送
  * @param  usLen：发送的数据长度
  * @retval None
  */
void DMA_RecvData( DMA_Stream_TypeDef *DMAy_Streamx, uint16_t usLen )
{
	LED2 = !LED2;
	USART1_SetDirIn();	
	DMA_Cmd(DMAy_Streamx, DISABLE );  //
 	DMA_SetCurrDataCounter(DMAy_Streamx, usLen);//
 	DMA_Cmd(DMAy_Streamx, ENABLE);  //

}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	复位dma
    * @param  	None
	* @retval 	无
    */
void DMA_Clear( DMA_Stream_TypeDef *DMAy_Streamx )
{
	DMA_Cmd( DMAy_Streamx, DISABLE );
	DMA_SetCurrDataCounter( DMAy_Streamx, 0 );
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	串口2 ISR
    * @param  	None
    * @retval 	None
    */
void USART2_IRQHandler(void) 
{	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	} 
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	串口1 ISR
    * @param  	None
    * @retval 	None
    */ 
void USART1_IRQHandler(void) 
{	
	int bytesLeft;
	int packHeadIndex;
	//串口2空闲中断标志置位
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		bytesLeft = dmaServoRxStream->NDTR;
		xServoMsg.ucByteRecved = xServoMsg.byteToRecv + DMA_BYTE_ADD_LEN - bytesLeft;
//		if( xServoMsg.ucByteRecved >= DMA_BYTE_ADD_LEN && !packHeadFound )
//		{
//			packHeadIndex = findPackHead( g_ucaServoRxBuffer, xServoMsg.ucByteRecved );
//			if( packHeadIndex >= 0 )
//			{
//				xServoMsg.ucByteRecved -= packHeadIndex;
//				packHeadFound = true;
//			}
//		}
	
//		printf("%d\r\n", bytesLeft );
		LED3 = !LED3;
		//消息结构体中的数据就绪状态为true
		if( xServoMsg.ucByteRecved >= xServoMsg.byteToRecv )
		{
			xServoMsg.bDataReady = true;
			USART_ITConfig( USART1, USART_IT_IDLE, DISABLE );
			DMA_Cmd( dmaServoRxStream, DISABLE );
			DMA_SetCurrDataCounter( dmaServoRxStream, 0 );
			DMA_ClearFlag( dmaServoRxStream, dmaServoRxTCIF );
			
		}
		//这里是读一次接收寄存器以清除IDLE标志（详见手册）
		bytesLeft = USART_ReceiveData(USART1);
		bytesLeft = bytesLeft;
		
	}
}
/* ---------------------------------------------------------------------------*/

/**
  * @brief 	串口1发送DMA中断服务函数
  * @param  None
  * @retval None
  */
void DMA2_Stream7_IRQHandler(void)
{
	
	if(DMA_GetFlagStatus(dmaServoTxStream, dmaServoTxTCIF) != RESET) 
	{
		
		//清除标志位
		DMA_ClearFlag(dmaServoTxStream, dmaServoTxTCIF);
		//关闭DMA
		DMA_Cmd(dmaServoTxStream, DISABLE);
		//发送完毕，使能串口接收中断和空闲中断
		if( xServoMsg.fReadEnable )
		{
			USART_ITConfig( USART1, USART_IT_IDLE, ENABLE );
			DMA_RecvData( dmaServoRxStream, xServoMsg.byteToRecv + DMA_BYTE_ADD_LEN );
//			DMA_RecvData( dmaServoRxStream, 50 + DMA_BYTE_ADD_LEN, ENABLE );
		}
		
		LED1 = !LED1;
		
	}
}
/* ---------------------------------------------------------------------------*/

/**
  * @brief 	串口1接收DMA中断服务函数
  * @param  None
  * @retval None
  */
void DMA2_Stream2_IRQHandler(void)
{
	if(DMA_GetFlagStatus( dmaServoRxStream, dmaServoRxTCIF ) != RESET) 
	{
		LED4 = !LED4;
		//清除标志位
		DMA_ClearFlag( dmaServoRxStream, dmaServoRxTCIF );
		//关闭DMA
		DMA_Cmd( dmaServoRxStream, DISABLE );
	}
}