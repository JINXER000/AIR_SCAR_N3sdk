#ifndef __USART1_H__
#define __USART1_H__
#include "sys.h"
#define PITCH_MAX 19.0f
#define YAW_MAX 720.0f//720.0				//cyq:��̨�Ƕȵķ�Χ
/*
*********************************************************************************************************
*                                               MACROS
*********************************************************************************************************
*/

#define  BSP_USART1_DMA_RX_BUF_LEN               30u                   
 
#define BSP_USART1_RX_BUF_SIZE_IN_FRAMES         (BSP_USART1_RX_BUF_SIZE / RC_FRAME_LENGTH)
#define  RC_FRAME_LENGTH                            18u
#define     BSP_USART1_RX_BUF_SIZE            128u
#define     BSP_USART3_RX_BUF_SIZE            1024u
#define     BSP_USART3_TX_BUF_SIZE            512u

/*
*********************************************************************************************************
*                                             FUNCTION PROTOTYPES
*********************************************************************************************************
*/
#ifdef __cplusplus
extern "C"{
#endif //__cplusplus
void USART1_IRQHandler(void);
#ifdef __cplusplus
}
#endif //__cplusplus
static void USART1_FIFO_Init(void);
void *USART1_GetRxBuf(void);
void USART1_Configuration(uint32_t baud_rate);
void RemoteDataPrcess(uint8_t *pData);

#endif
