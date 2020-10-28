#ifndef __USART_H
#define __USART_H

#include "sys.h" 
#include "stdio.h"

#define DMA_BYTE_ADD_LEN			150

/*串口宏引脚定义*/
#define usartServoIOCtrlGPIO		GPIOA
#define usartServoIOCtrlRCC			RCC_AHB1Periph_GPIOA
#define usartServoIOCtrlPin			GPIO_Pin_11	

/*DMA宏定义*/
#define dmaServoTxPeriphRCC			RCC_AHB1Periph_DMA2
#define dmaServoRxPeriphRCC			RCC_AHB1Periph_DMA2
#define dmaServoTxStream			DMA2_Stream7
#define dmaServoRxStream			DMA2_Stream2
#define dmaServoTxChn				DMA_Channel_4
#define dmaServoRxChn				DMA_Channel_4
#define dmaServoTxIRQn				DMA2_Stream7_IRQn
#define dmaServoRxIRQn				DMA2_Stream2_IRQn
#define dmaServoTxTCIF				DMA_IT_TCIF7
#define dmaServoRxTCIF				DMA_IT_TCIF2
#define dmaServoTxPeriphAddr		( uint32_t )&USART1->DR
#define dmaServoRxPeriphAddr		( uint32_t )&USART1->DR

void USART1_Init(uint32_t ulBaudrate);
void USART2_Init(uint32_t ulBaudrate);
void USART1_SetDirIn( void );
void USART1_SetDirOut( void );

void DMA_ServoTxInit( uint32_t ulPeriphAddr, uint32_t ulMemAddr, uint16_t usBufSize );
void DMA_ServoRxInit( uint32_t ulPeriphAddr, uint32_t ulMemAddr, uint16_t usBufSize );
void DMA_SendData( DMA_Stream_TypeDef *DMAy_Streamx, uint16_t usLen );
void DMA_RecvData( DMA_Stream_TypeDef *DMAy_Streamx, uint16_t usLen );
void DMA_Clear( DMA_Stream_TypeDef *DMAy_Streamx );
void DMA_PrintTime( void );




#endif


