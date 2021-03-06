/*! @file BspUsart.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Usart helper functions and ISR for board STM32F4Discovery
 *
 *  @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *	U6: PC 	U1:REMOTE		U3: XMC		U2:N3		U4:  phone   U5 TX(PC12)(3) U5 RX(PD2)(1) :BASE OF
 *  XMC : 32  // XMC2  //RADAR
 */

#include "stm32f4xx.h"
#include "BspUsart.h"
#include "timer.h"
#include "Driver_vision.h"
#include "kdbase.h"
#include "ano_of.h"
extern int Rx_Handle_Flag;

using namespace DJI::OSDK;

extern Vehicle  vehicle;
extern Vehicle* v;
extern Control  control;

extern bool           isFrame;
bool                  isACKProcessed    = false;
bool                  ackReceivedByUser = false;
extern RecvContainer  receivedFramie;
extern RecvContainer* rFrame;

// extern CoreAPI defaultAPI;
// extern CoreAPI *coreApi;
// extern Flight flight;
// extern FlightData flightData;

extern uint8_t come_data;
extern uint8_t Rx_length;
extern int     Rx_adr;
extern int     Rx_Handle_Flag;
extern uint8_t Rx_buff[];

void
USART2_Gpio_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3 ;//| GPIO_Pin_9 | GPIO_Pin_10;
	
	
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // tx
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // rx
	
	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART3); // tx
  //GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART3); // rx
}

void
USART3_Gpio_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11 ;//| GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // tx
  //GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // rx
	
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); // tx
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); // rx
}

/*
 * USART2 is used for receiving commands from PC and
 * printing debug information to PC
 */
void
USART2_Config(void)
{
  USART2_Gpio_Config();

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate   = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits   = USART_StopBits_1;
  USART_InitStructure.USART_Parity     = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  USART_Cmd(USART2, ENABLE);

  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) != SET)
    ;
}

/*
 * USART3 is used for communicating with the DJI flight controller
 * The Baud rate needs to match the Baud rate used by the flight controller
 */


void
USART3_Config(void)
{
  USART3_Gpio_Config();

  USART_InitTypeDef USART_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);  //change
  USART_InitStructure.USART_BaudRate   = 230400;     //230400
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits   = USART_StopBits_1;
  USART_InitStructure.USART_Parity     = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART3, &USART_InitStructure);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  USART_Cmd(USART3, ENABLE);
  while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) != SET)
    ;
}

// Author: CYZ
//U4 is for base
void Uart4_Init ( u32 br_num )
{
    USART_InitTypeDef USART_InitStructure;
    //USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_UART4, ENABLE ); //??USART2??
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOA, ENABLE );

    //???????
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );


    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource1, GPIO_AF_UART4 );

    //??PC12??UART5 Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );
    //??PD2??UART5 Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );

    //??UART5
    //??????
    USART_InitStructure.USART_BaudRate = br_num;       //????????????
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8???
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //??????1????
    USART_InitStructure.USART_Parity = USART_Parity_No;    //??????
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //???????
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //???????
    USART_Init ( UART4, &USART_InitStructure );

    //??UART5????
    USART_ITConfig ( UART4, USART_IT_RXNE, ENABLE );
    //??USART5
    USART_Cmd ( UART4, ENABLE );
}
u8 Tx4Buffer[256];
u8 Tx4Counter = 0;
u8 count4 = 0;

//void UART4_IRQHandler ( void )
//{
//    u8 com_data;

//    //????
//    if ( USART_GetITStatus ( UART4, USART_IT_RXNE ) )
//    {
//        USART_ClearITPendingBit ( UART4, USART_IT_RXNE ); //??????

//        com_data = UART4->DR;

////        AnoOF_GetOneByte ( com_data );
//    }

//    //??(????)??
//    if ( USART_GetITStatus ( UART4, USART_IT_TXE ) )
//    {

//        UART4->DR = Tx4Buffer[Tx4Counter++]; //?DR??????

//        if ( Tx4Counter == count4 )
//        {
//            UART4->CR1 &= ~USART_CR1_TXEIE;		//??TXE(????)??
//        }


//        //USART_ClearITPendingBit(USART2,USART_IT_TXE);
//    }

//}
void Uart4_Send(unsigned char *DataToSend ,u8 data_num)
{
	u8 i;
	for(i=0;i<data_num;i++)
	{

		Tx4Buffer[count4++] = *(DataToSend+i);
	}

	if(!(UART4->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(UART4, USART_IT_TXE, ENABLE); //??????
	}

}

//u6 is for minipc 

void USART6_IT_Config(void)
{
     NVIC_InitTypeDef NVIC_InitStructure;
   
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//?????�??????
     //USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
    //Usart6 NVIC ????
     NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//?�????????????????�?��2
     NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//?�??6?????�??
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//??????????3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;       //�???????2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQ?�??????
    NVIC_Init(&NVIC_InitStructure); //?�?????�????????????NVIC?????�??
 
}

void USART6_Config(u32 bound){
     //GPIO??
     GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
     
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //????GPIOC?�??
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//????USART6?�??
  
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOC6??????USART6
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOC7??????USART6
     
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_6; //GPIOC6??GPIOC7
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//????????
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //????50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //???�????????
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //????
    GPIO_Init(GPIOC,&GPIO_InitStructure); //??????PC6??PC7
 
	
		USART_InitStructure.USART_BaudRate = bound;       //????????????
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8???
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //??????1????
	USART_InitStructure.USART_Parity = USART_Parity_No;    //??????
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //???????
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //???????
     USART_Init(USART6, &USART_InitStructure); //???????�??6
     
		 USART6_IT_Config();
//		 USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);

     USART_Cmd(USART6, ENABLE);  //?????�??6
     
    USART_ClearFlag(USART6, USART_FLAG_TC);
}

u8 Tx6Buffer[256];
u8 Tx6Counter = 0;
u8 count6 = 0;

void Usart6_Send(unsigned char *DataToSend ,u8 data_num)
{
	u8 i;
	for(i=0;i<data_num;i++)
	{

		Tx6Buffer[count6++] = *(DataToSend+i);
	}

	if(!(USART6->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(USART6, USART_IT_TXE, ENABLE); //??????
	}

}


#ifdef __cplusplus
extern "C" {
#endif //__cplusplus
    int comdata6;

void USART6_IRQHandler(void)
{

//	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
		if(USART_GetITStatus(UART5,USART_FLAG_ORE)==RESET)

	{
			USART_ClearITPendingBit(USART6,USART_IT_RXNE); 
			PCdataprocess(USART6->DR);
			KDBase_process(USART6->DR);
		comdata6=USART6->DR;
    }
		
    if ( USART_GetITStatus ( USART6, USART_IT_TXE ) )
    {

        USART6->DR = Tx6Buffer[Tx6Counter++]; //?DR??

        if ( Tx6Counter == count6 )
        {
            USART6->CR1 &= ~USART_CR1_TXEIE;		//??TXE(????)??
        }

        //USART_ClearITPendingBit(USART2,USART_IT_TXE);
    }

 
}
#ifdef __cplusplus
}
#endif //__cplusplus

void Uart5_Init(u32 br_num)
{
	USART_InitTypeDef USART_InitStructure;
	//USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	
	//配置PC12作为UART5　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD2作为UART5　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置UART5
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(UART5, &USART_InitStructure);
	


	//使能UART5接收中断
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	//使能USART5
	USART_Cmd(UART5, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}

}
u8 Tx5Buffer[256];
u8 Tx5Counter=0;
u8 count5=0; 

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

void UART5_IRQHandler(void)
{
	u8 com_data;

  //接收中断
	//if( USART_GetITStatus(UART5,USART_IT_RXNE) )
	if(USART_GetITStatus(UART5,USART_FLAG_ORE)==RESET)
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);//清除中断标志

		com_data = UART5->DR;
		
		AnoOF_GetOneByte(com_data);
	}

	//发送（进入移位）中断
	if( USART_GetITStatus(UART5,USART_IT_TXE ) )
	{
				
		UART5->DR = Tx5Buffer[Tx5Counter++]; //写DR清除中断标志
          
		if(Tx5Counter == count5)
		{
			UART5->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}


		//USART_ClearITPendingBit(USART2,USART_IT_TXE);
	}

}

#ifdef __cplusplus
}
#endif //__cplusplus


void Uart5_Send(unsigned char *DataToSend ,u8 data_num)
{
	u8 i;
	for(i=0;i<data_num;i++)
	{
		Tx5Buffer[count5++] = *(DataToSend+i);
	}

	if(!(UART5->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(UART5, USART_IT_TXE, ENABLE); //打开发送中断
	}

}
void
USARTxNVIC_Config()
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  NVIC_InitTypeDef NVIC_InitStructure_USART3;
  NVIC_InitStructure_USART3.NVIC_IRQChannelPreemptionPriority = 0x04;
  NVIC_InitStructure_USART3.NVIC_IRQChannelSubPriority        = 0x03;
  NVIC_InitStructure_USART3.NVIC_IRQChannel                   = USART3_IRQn;
  NVIC_InitStructure_USART3.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure_USART3);

  NVIC_InitTypeDef NVIC_InitStructure_USART2;
  NVIC_InitStructure_USART2.NVIC_IRQChannelPreemptionPriority = 0x03;
  NVIC_InitStructure_USART2.NVIC_IRQChannelSubPriority        = 0x02;
  NVIC_InitStructure_USART2.NVIC_IRQChannel                   = USART2_IRQn;
  NVIC_InitStructure_USART2.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure_USART2);
}

void
UsartConfig()
{
  USART2_Config();
  USART3_Config();
	
  USARTxNVIC_Config();
}
/*
DJI::OSDK::ACK::ErrorCode
waitForACK()
{
  ACK::ErrorCode ack;
  ack.data = ACK_NO_RESPONSE_ERROR;
  memset(&(ack.data), 0, sizeof(ack.data));
  uint32_t next500MilTick;
	uint8_t cmd[] = {rFrame->recvInfo.cmd_set, rFrame->recvInfo.cmd_id};

  //	next500MilTick = v->protocolLayer->getDriver()->getTimeStamp() + 500;

  //	while(rFrame->dispatchInfo.isCallback != true &&
  //		v->protocolLayer->getDriver()->getTimeStamp() < next500MilTick)
  while (true)
  {
    if (isACKProcessed == true)
    {
      if (rFrame->recvInfo.cmd_set == DJI::OSDK::CMD_SET_ACTIVATION &&
          rFrame->recvInfo.cmd_id == DJI::OSDK::CMD_ID_ACTIVATE)
      {
        ack.data = rFrame->recvData.ack;
        ack.info = rFrame->recvInfo;

        return ack;
      }
      else if (rFrame->recvInfo.cmd_set == DJI::OSDK::CMD_SET_SUBSCRIBE &&
               rFrame->recvInfo.cmd_id ==
                 DJI::OSDK::CMD_ID_SUBSCRIBE_VERSION_MATCH)
      {
        ack.data = rFrame->recvData.ack;
        ack.info = rFrame->recvInfo;

        return ack;
      }
      else if (rFrame->recvInfo.cmd_set == DJI::OSDK::CMD_SET_SUBSCRIBE &&
               rFrame->recvInfo.cmd_id ==
                 DJI::OSDK::CMD_ID_SUBSCRIBE_ADD_PACKAGE)
      {
        ack.data = rFrame->recvData.ack;
        ack.info = rFrame->recvInfo;

        return ack;
      }
      else if (rFrame->recvInfo.cmd_set == DJI::OSDK::CMD_SET_CONTROL &&
               rFrame->recvInfo.cmd_id == DJI::OSDK::CMD_ID_TASK)
      {
        ack.data = rFrame->recvData.ack;
        ack.info = rFrame->recvInfo;

        return ack;
      }
    }
  }

  // return ack;
}
*/
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus



void UART4_IRQHandler ( void )
{
  if (USART_GetFlagStatus(UART4, USART_FLAG_RXNE) == SET)
  {
    isACKProcessed = false;
    isFrame = v->protocolLayer->byteHandler(USART_ReceiveData(UART4));
    if (isFrame == true)
    {
			rFrame = v->protocolLayer->getReceivedFrame();
			
      //! Trigger default or user defined callback
      v->processReceivedData(rFrame);

      //! Reset
      isFrame        = false;
      isACKProcessed = true;
    }
  }
}


/*void UART4_IRQHandler ( void )
{
    u8 com_data;

    //????
    if ( USART_GetITStatus ( UART4, USART_IT_RXNE ) )
    {
        USART_ClearITPendingBit ( UART4, USART_IT_RXNE ); //??????

        com_data = UART4->DR;

//        AnoOF_GetOneByte ( com_data );
    }

    //??(????)??
    if ( USART_GetITStatus ( UART4, USART_IT_TXE ) )
    {

        UART4->DR = Tx4Buffer[Tx4Counter++]; //?DR??????

        if ( Tx4Counter == count4 )
        {
            UART4->CR1 &= ~USART_CR1_TXEIE;		//??TXE(????)??
        }


        //USART_ClearITPendingBit(USART2,USART_IT_TXE);
    }

}*/
#ifdef __cplusplus
}
#endif //__cplusplus
