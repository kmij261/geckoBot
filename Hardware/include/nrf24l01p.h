/**
  ***********************************UTF-8**************************************
  * @file    nrf24l01p.h
  * @author  Xiong
  * @version V1.0
  * @date    02-July-2020
  * @brief   此文件用于实现NRF24L01+的收发，模块型号为EBYTE-ML01DP5
  ******************************************************************************  
  */ 
#ifndef _NRF24L01P_H
#define _NRF24L01P_H

#include "spi.h"

/* nrf24l01+ SPI宏定义	------------------------------------------------------*/
#define nrfSPI						( ( SPI_TypeDef * ) SPI1 )

/* nrf24l01+ 类型宏定义	------------------------------------------------------*/
typedef unsigned char NrfStatusType;
#define NrfStatus_t 				NrfStatusType

/* nrf24l01+模块时钟和GPIO、EXTI宏定义	--------------------------------------*/
#define nrfRCC						( ( uint32_t ) RCC_AHB1Periph_GPIOA|\
												   RCC_AHB1Periph_GPIOB|\
												   RCC_AHB1Periph_GPIOC )
#define nrfIO_CSN_GPIO				( ( GPIO_TypeDef* ) GPIOA )
#define nrfIO_CE_GPIO				( ( GPIO_TypeDef* ) GPIOC )
#define nrfIO_IRQ_GPIO				( ( GPIO_TypeDef* ) GPIOC )
#define nrfIO_CSN_Pin				( ( uint16_t ) GPIO_Pin_4 )
#define nrfIO_CE_Pin				( ( uint16_t ) GPIO_Pin_4 )
#define nrfIO_IRQ_Pin				( ( uint16_t ) GPIO_Pin_5 )
#define nrfEXTI_Line				( ( uint32_t ) EXTI_Line0 )

/* nrf24l01+ 命令宏定义 ------------------------------------------------------*/
#define nrfCMD_R_REG				( ( uint8_t ) 0x00 )
#define nrfCMD_W_REG				( ( uint8_t ) 0x20 )
#define nrfCMD_R_RX_PAYLOAD			( ( uint8_t ) 0x61 )
#define nrfCMD_W_TX_PAYLOAD			( ( uint8_t ) 0xA0 )
#define nrfCMD_FLUSH_TX				( ( uint8_t ) 0xE1 )
#define nrfCMD_FLUSH_RX				( ( uint8_t ) 0xE2 )
#define nrfCMD_REUSE_TX_PL			( ( uint8_t ) 0xE3 )
#define nrfCMD_R_RX_PL_WID			( ( uint8_t ) 0x60 )
#define nrfCMD_W_ACK_PAYLOAD		( ( uint8_t ) 0xA8 )
#define nrfCMD_W_TX_PL_NACK			( ( uint8_t ) 0xB0 )
#define nrfCMD_NOP					( ( uint8_t ) 0xFF )

/* nrf24l01+ 寄存器地址宏定义 ------------------------------------------------*/
#define nrfREG_CONFIG				( ( uint8_t ) 0x00 )
#define nrfREG_ENAA					( ( uint8_t ) 0x01 )
#define nrfREG_RXADDR				( ( uint8_t ) 0x02 )
#define nrfREG_SETUP_AW				( ( uint8_t ) 0x03 )
#define nrfREG_SETUP_RETR			( ( uint8_t ) 0x04 )
#define nrfREG_RF_CH				( ( uint8_t ) 0x05 )
#define nrfREG_RF_SETUP				( ( uint8_t ) 0x06 )
#define nrfREG_STATUS				( ( uint8_t ) 0x07 )
#define nrfREG_OBSERVE_TX			( ( uint8_t ) 0x08 )
#define nrfREG_RPD					( ( uint8_t ) 0x09 )
#define nrfREG_RX_ADDR_P0			( ( uint8_t ) 0x0A )
#define nrfREG_RX_ADDR_P1			( ( uint8_t ) 0x0B )
#define nrfREG_RX_ADDR_P2			( ( uint8_t ) 0x0C )
#define nrfREG_RX_ADDR_P3			( ( uint8_t ) 0x0D )
#define nrfREG_RX_ADDR_P4			( ( uint8_t ) 0x0E )
#define nrfREG_RX_ADDR_P5			( ( uint8_t ) 0x0F )
#define nrfREG_TX_ADDR				( ( uint8_t ) 0x10 )
#define nrfREG_RX_PW_P0				( ( uint8_t ) 0x11 )
#define nrfREG_RX_PW_P1				( ( uint8_t ) 0x12 )
#define nrfREG_RX_PW_P2				( ( uint8_t ) 0x13 )
#define nrfREG_RX_PW_P3				( ( uint8_t ) 0x14 )
#define nrfREG_RX_PW_P4				( ( uint8_t ) 0x15 )
#define nrfREG_RX_PW_P5				( ( uint8_t ) 0x16 )
#define nrfREG_FIFO_STATUS			( ( uint8_t ) 0x17 )
#define nrfREG_DYNPD				( ( uint8_t ) 0x1C )
#define nrfREG_FEATURE				( ( uint8_t ) 0x1D )


/* nrf24l01+ 类型定义 --------------------------------------------------------*/


/* RX_DR引起中断使能 */
typedef enum
{
	NRF_Mask_RX_DR_Enable 		= 0x00,	/*!<*/
	NRF_Mask_RX_DR_Dsiable		= 0x40,
}NRF_Mask_RX_DR_t;

/* TX_DS引起中断使能 */
typedef enum
{
	NRF_Mask_TX_DS_Enable		= 0x00,
	NRF_Mask_TX_DS_Disable		= 0x20,
}NRF_Mask_TX_DS_t;

/* MAX_RT引起中断使能 */
typedef enum
{
	NRF_Mask_MAX_RT_Enable		= 0x00,
	NRF_Mask_MAX_RT_Disable		= 0x10,
}NRF_Mask_MAX_RT_t;

/* CRC使能 */
typedef enum
{
	NRF_CRC_Disable				= 0x00,
	NRF_CRC_Enable				= 0x08,
}NRF_EN_CRC_t;

/* CRC编码字节数 */
typedef enum 
{
	NRF_CRC_Coding_1_Byte		= 0x00,
	NRF_CRC_Coding_2_Bytes		= 0x04,
}NRF_CRC_Coding_Bytes_t;

/* 电源使能 */
typedef enum
{
	NRF_Power_Down				= 0x00,
	NRF_Power_Up				= 0x02,
}NRF_PWR_Manage_t;

/* PRIM_RX */
typedef enum
{
	NRF_PRIM_PTX				= 0x00,
	NRF_PRIM_PRX				= 0x01,
}NRF_PRIM_RX_t;

/* 数据管道枚举定义 */
typedef enum
{
	NRF_Data_Pipe_0				= 0x00,
	NRF_Data_Pipe_1				= 0x01,
	NRF_Data_Pipe_2				= 0x02,
	NRF_Data_Pipe_3				= 0x04,
	NRF_Data_Pipe_4				= 0x08,
	NRF_Data_Pipe_5				= 0x10,
}NRF_Data_Pipe_t;

/* 地址宽度定义 */
typedef enum
{
	NRF_ADDR_Width_3_Bytes		= 0x01,
	NRF_ADDR_Width_4_Bytes		= 0x02,
	NRF_ADDR_Width_5_Bytes		= 0x03,
}NRF_ADDR_Width_t;

/* 自动重发延时定义 */
typedef enum 
{
	NRF_ARD_250us				= 0x00,
	
}NRF_Auto_Retransmit_Delay_t;

/* 自动重发次数定义 */
typedef enum 
{
	NRF_ARC_Diable				= 0x00,
	NRF_ARC_1					= 0x01,
	
}NRF_Auto_Retransmit_Count_t;
	
/*nrf24l01+ 自动重发结构体类型*/
typedef enum
{
	NRF_RX_Mode					= 0x00,
	NRF_TX_Mode					= 0x01,
}NRF_Mode_t;
/*nrf24l01+ 初始化结构体类型*/
typedef struct NrfInitStruct
{
	NRF_Mask_RX_DR_t 			NRF_Mask_RX_DR;
	NRF_Mask_TX_DS_t 			NRF_Mask_TX_DS;
	NRF_Mask_MAX_RT_t 			NRF_Mask_MAX_RT;
	NRF_EN_CRC_t 				NRF_EN_CRC;
	NRF_CRC_Coding_Bytes_t 		NRF_CRC_Coding_Bytes;
	NRF_PWR_Manage_t 			NRF_PWR_Manage;
	NRF_PRIM_RX_t			NRF_PRIM_RX;
	
}NRF_InitTypeDef;

/*nrf24l01+ 自动重发结构体类型*/
typedef struct NrfAutoReTxStruct
{
	NRF_Auto_Retransmit_Delay_t Nrf_ARD_us;
	NRF_Auto_Retransmit_Count_t Nrf_ARC;
}NrfAutoReTx_t;

/* nrf24l01+ 全局变量定义 ----------------------------------------------*/
/*设定nrf24l01+模块的发送目标地址*/
extern const uint8_t NRF_TX_Addr[5];
/*设定nrf24l01+模块的本机接收地址*/
extern const uint8_t NRF_RX_Addr[5];
/*设定nrf24l01+模块的收/发频道的中心频率为2484MHz*/
extern const uint8_t RF_Channel;

/* nrf24l01+ 寄存器读写函数定义 ----------------------------------------------*/
NrfStatus_t Nrf_RegSingleWrite( uint8_t ucRegAddr, uint8_t ucSrc, 
									SpiErr_t* pErr );
NrfStatus_t Nrf_RegSingleRead( uint8_t ucRegAddr, uint8_t* pucDst, 
									SpiErr_t* pErr );
NrfStatus_t Nrf_RegMultiWrite( uint8_t ucRegAddr, uint8_t* ucSrcBuf, 
									uint8_t ucLen, SpiErr_t* pErr );
NrfStatus_t Nrf_RegMultiRead( uint8_t ucRegAddr, uint8_t* ucDstBuf, 
									uint8_t ucLen, SpiErr_t* pErr );

/* nrf24l01+ 操作函数定义 ----------------------------------------------------*/
NrfStatus_t Nrf_Init( void );
NrfStatus_t Nrf_TxData( uint8_t* ucSrcBuf, uint8_t ucLen );
NrfStatus_t Nrf_RxData( uint8_t* usDstBuf, uint8_t ucLen );

/* nrf24l01+ 参数设定函数定义 ------------------------------------------------*/
NrfStatus_t Nrf_SetAutoAckState( NRF_Data_Pipe_t xPipe, FunctionalState fState );
NrfStatus_t Nrf_SetRxAddrState( NRF_Data_Pipe_t xPipe, FunctionalState fState );
NrfStatus_t Nrf_SetPowerState( FunctionalState fState );
NrfStatus_t Nrf_SetMode( NRF_Mode_t xTxRxMode );
NrfStatus_t Nrf_SetADDR_Width( NRF_ADDR_Width_t xWidth);
NrfStatus_t Nrf_SetAutoRetr( uint8_t RETR_Status );
NrfStatus_t Nrf_SetRF( uint8_t RF_Channel, uint8_t RF_param );
NrfStatus_t Nrf_SetRxADDR( const uint8_t* rxAddrBuf, uint8_t addrWidth);
NrfStatus_t Nrf_SetTxADDR( const uint8_t* txAddrBuf, uint8_t addrWidth);

/* nrf24l01+ 中断函数定义 ------------------------------------------------*/
uint8_t Nrf_IT_Triggered(void);


#ifdef IN_DEBUG_MODE
void Nrf_SetCSN_Low( void );
void Nrf_SetCSN_High( void );
void Nrf_SetCE_Low( void );
void Nrf_SetCE_High( void );
#endif

#endif
