/*! @file Receive.cpp
 *
 *  @version 3.3
 *  @date Jun 2017
 *
 *  @brief
 *  This function parses Rx buffer and execute commands sent from computer.
 *
 *  @Copyright (c) 2016-2017 DJI
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
 *
 */

#include "stm32f4xx.h"
#include "Receive.h"
#include "main.h"
uint8_t mission_flag=0;
volatile  uint8_t baro_height = 0; //unit:dm
volatile  int8_t baro_delta_h = 0; // unit:dm               //////////////////////////////////////////
volatile  uint8_t  flag_baro_hold = 0;                      ///////////////////////////////////////////
extern Vehicle* v;

using namespace DJI::OSDK;

//cmd for api test
uint32_t runOnce = 0;
float x_offset,y_offset,z_offset,yaw_desired;    //for position control
float set_vx,set_vy,set_vz,set_yawrate;             //for velocity control  
uint8_t set_duration;


//communication with XMC
static uint8_t xmc_data_flag = 0x00;  
static uint8_t xmc_data[50];
static uint8_t xmc_data_state = 0;
static uint8_t xmc_data_len = 0;  // data length without functional bytes
static uint8_t xmc_data_cnt = 0;  // data number counter
Target_List_t s_target_list;                        //////radar data
/*
 * @brief Helper function to assemble two bytes into a float number
 */
static float32_t
hex2Float(uint8_t HighByte, uint8_t LowByte)
{
  float32_t high = (float32_t)(HighByte & 0x7f);
  float32_t low  = (float32_t)LowByte;
  if (HighByte & 0x80) // MSB is 1 means a negative number
  {
    return -(high * 256.0f + low) / 100.0f;
  }
  else
  {
    return (high * 256.0f + low) / 100.0f;
  }
}

TerminalCommand myTerminal;


void XMC_Data_Receive_Prepare(uint8_t data)
{
	
		if(xmc_data_state==0&&data==0xFA)
	{
		xmc_data_state=1;
		xmc_data[0]=data;
	}
	else if(xmc_data_state==1&&data==0xFB)
	{
		xmc_data_state=2;
		xmc_data[1]=data;
	}
	else if(xmc_data_state==2&&data<0XF1)     //功能字
	{
		xmc_data_state=3;
		xmc_data[2]=data;
	}
	else if(xmc_data_state==3&&data<50)
	{
		xmc_data_state = 4;
		xmc_data[3]=data;
		xmc_data_len = data;
		xmc_data_cnt = 0;
	}
	else if(xmc_data_state==4&&xmc_data_len>0)
	{
		xmc_data_len--;
		xmc_data[4+xmc_data_cnt++]=data;
		if(xmc_data_len==0)
			xmc_data_state = 5;
	}
	else if(xmc_data_state==5)
	{
		xmc_data_state = 0;
		xmc_data[4+xmc_data_cnt]=data;
		
		
		XMC_Data_Handler(); //考虑放在中断服务中！！！
		
		
	}
	else
	{
		xmc_data_state = 0;
	}
		
}

void
XMC_Data_Handler(void)
{
	uint16_t checksum=0;
	
	for(uint8_t i=0;i<(4+xmc_data_cnt);i++)
	checksum+=xmc_data[i];
	
	checksum = checksum%256;
	if(checksum != xmc_data[xmc_data_cnt+4])
	{	
		printf("check failed\n");
		
		return;
	}
	xmc_data_flag = xmc_data[2];   // 0x20 0x21 0x22 0x23
	if(xmc_data_flag == 0)
		 flag_baro_hold = 0;
	if(xmc_data[2])
	{
      baro_delta_h =(int8_t) xmc_data[4];	
			s_target_list.target_num = xmc_data[5];
		  for(uint8_t i=0;i<s_target_list.target_num;i++)
		   {
			    s_target_list.distance[i] = ((uint16_t)xmc_data[6+6*i]<<8)|((uint16_t)xmc_data[7+6*i]);
				  s_target_list.level[i] =  ((uint32_t)xmc_data[8+6*i]<<24)|((uint32_t)xmc_data[9+6*i]<<16)|((uint32_t)xmc_data[10+6*i]<<8)|((uint32_t)xmc_data[11+6*i]);
			 }
		}
}
void
TerminalCommand::terminalCommandHandler(Vehicle* vehicle)
{
  if (cmdReadyFlag == 1)
  {
    cmdReadyFlag = 0;
  }
  else
  { // No full command has been received yet.
    return;
  }

  if ((cmdIn[0] != 0xFA) || (cmdIn[1] != 0xFB))
  { // Command header doesn't match
    return;
  }

  switch (cmdIn[2])
  {
    case 0x00:
      vehicle->getDroneVersion();
      break;
    case 0x01:
      userActivate();
      break;
		case 0x02:
      printf("\n\nStarting executing position control:\r\n");
      delay_nms(10);
      // Run position control sample
      float zPosition = 0;
			x_offset = (float)cmdIn[3]/10.0;
		  y_offset = (float)cmdIn[4]/10.0;
		  z_offset = (float)cmdIn[5]/10.0;
		  yaw_desired = (float)cmdIn[6];
	    moveByPositionOffset(x_offset, y_offset, z_offset, yaw_desired);
		
		  //   should add ack
		  //
		  break;
		
		case 0x03:
			set_vx = (float)cmdIn[3]/10.0;
		  set_vy = (float)cmdIn[4]/10.0;
		  set_vz = (float)cmdIn[5]/10.0;
		  set_yawrate = (float)cmdIn[7];
		  uint8_t cnt = cmdIn[6]*50;  //50Hz  for test
		  while(cnt--){
					v->control->velocityAndYawRateCtrl(set_vx,set_vy,set_vz,set_yawrate);
					delay_nms(20);       //50Hz   for test
					}
			//   should add ack
		  //
			break;
					
		case 0x04:
			subscribeToData();
		  break;
    case 0x05:
      switch(cmdIn[3])
        {
        case 0x01:
          //flight->task(Flight::TASK_GOHOME);
          break;
        case 0x02:
          monitoredTakeOff();
          break;
        case 0x03:
          monitoredLanding();
          break;
        }
        break;
		case 0x06:
			 v->obtainCtrlAuthority();
		   break;
		case 0x07:
			if(cmdIn[3])
			{
		    printf("Starting attitude hold by baro_delta_h.\n");
				if(xmc_data_flag)
					flag_baro_hold = 1;
				else printf("Starting failed.Press Button2");
				
			}
			else
				flag_baro_hold = 0;
				
      delay_nms(10);	
      break;     
    /*  case 0x02:
        api->setControl(cmdIn[3]);
        break;

      case 0x03:
        flight->setArm(cmdIn[3]);
        break;

      case 0x04:
        if (cmdIn[3] == 0x01)
        {
          flightData.flag = cmdIn[4];
          flightData.x = hex2Float(cmdIn[5], cmdIn[6]);
          flightData.y = hex2Float(cmdIn[7], cmdIn[8]);
          flightData.z = hex2Float(cmdIn[9], cmdIn[10]);
          flightData.yaw = hex2Float(cmdIn[11], cmdIn[12]);
          flight->setFlight(&flightData);
          TIM_Cmd(TIM2, ENABLE);
          printf("roll_or_x =%f\n", hex2Float(cmdIn[5], cmdIn[6]));
          printf("pitch_or_y =%f\n", hex2Float(cmdIn[7], cmdIn[8]));
          printf("thr_z =%f\n", hex2Float(cmdIn[9], cmdIn[10]));
          printf("yaw =%f\n\n", hex2Float(cmdIn[11], cmdIn[12]));
        }
        else if (cmdIn[3] == 0x02)
        {//for display ur hex2int
          printf("roll_or_x =%f\n", hex2Float(cmdIn[5], cmdIn[6]));
          printf("pitch_or_y =%f\n", hex2Float(cmdIn[7], cmdIn[8]));
          printf("thr_z =%f\n", hex2Float(cmdIn[9], cmdIn[10]));
          printf("yaw =%f\n\n", hex2Float(cmdIn[11], cmdIn[12]));
        }
        break;

      case 0x05:
        switch(cmdIn[3])
        {
        case 0x01:
          flight->task(Flight::TASK_GOHOME);
          break;
        case 0x02:
          flight->task(Flight::TASK_TAKEOFF);
          break;
        case 0x03:
          flight->task(Flight::TASK_LANDING);
          break;
        }
        break;

      case 0x06:
        if (cmdIn[3] == 0x00)              //0x06 0x00 to stop VRC
        {
          VRCResetData();
          TIM_Cmd(TIM1, DISABLE);
        }
        else if (cmdIn[3] == 0x02)            //0x06 0x01 to turn VRC to F gear
        {
          VRC_TakeControl();
          TIM_Cmd(TIM1, ENABLE);
        }
        else if (cmdIn[3] == 0x01)          //0x06 0x02 to reset data
        {
          VRCResetData();
          TIM_Cmd(TIM1, ENABLE);
        };
        break;

      case 0x07:
        if(cmdIn[3] == 0x00)
        {
          tryHotpoint(cmdIn[4], cmdIn[5], cmdIn[6]);
        }
        else
        {
          stopHotpoint();
        }
        break;
      case 0x08:
        printf("TimeStamp is %d\r\n", api->getBroadcastData().timeStamp.time);
        printf("Battery capacity remains %d percent\r\n",
            api->getBroadcastData().battery);
        break;

      case 0x09:
        if (cmdIn[3] == 0x01)
        {
          startLocalNavExample();
        }
        else if(cmdIn[3] == 0x00)
        {
          stopLocalNavExample();
        }
        break;
    */
    default:
      break;
  }
}

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus
void
USART2_IRQHandler(void)
{
  if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET)
  {
    uint8_t oneByte = USART_ReceiveData(USART2);
//		if(runOnce == 0)
//		runOnce = oneByte;
    if (myTerminal.rxIndex == 0)
    {
      if (oneByte == 0xFA){
        myTerminal.cmdIn[myTerminal.rxIndex] = oneByte;
        myTerminal.rxIndex                   = 1;
      }else{;}
    }
    else
    {
      if (oneByte == 0xFE){ // receive a 0xFE would lead to a command-execution
        myTerminal.cmdIn[myTerminal.rxIndex] = oneByte;
        myTerminal.rxLength                  = myTerminal.rxIndex + 1;
        myTerminal.rxIndex                   = 0;
        myTerminal.cmdReadyFlag              = 1;
				myTerminal.terminalCommandHandler(v);
			}else{
        myTerminal.cmdIn[myTerminal.rxIndex] = oneByte;
        myTerminal.rxIndex++;
      }
    }
  }
	//myTerminal.terminalCommandHandler();
}

void USART3_IRQHandler(void)
{
  if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
  {
    uint8_t oneByte = USART_ReceiveData(USART3);
		XMC_Data_Receive_Prepare(oneByte);
	}
}


#ifdef __cplusplus
}
#endif //__cplusplus
