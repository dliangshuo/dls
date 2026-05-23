#include "stm32f10x.h"
#include "stdio.h"
#include "string.h"
#include "oled.h"
#include "wifi.h"
#include "delay.h"
#include "usart.h"
volatile u16 recvCnt = 0;
volatile u8 wifiRecvBuf[WIFI_RECV_BUF_SIZE];
u8 wifiReady = 0;
volatile u8 wifiRecvOver = 0;
static u8 downlinkFrame[WIFI_RECV_BUF_SIZE];
static volatile u16 downlinkLen = 0;
static volatile u8 downlinkReady = 0;
int status = 0;

#define WIFI_WAIT_STEP_MS       60U
#define WIFI_WAIT_CYCLES_CMDAT  50U
#define WIFI_WAIT_CYCLES_RAW    8U


u8 UserName[]="q7pj8jzucv54vpm4";
u8 PassWord[]="sLWqI5ereL";
u8 Cloud_addr[] = "bj-2-mqtt.iot-api.com"; 
u8 WIFI_UserName[] = "DLS 2686";
u8 WIFI_PassWord[] = "56E=3g92";
u8 MQTT_SEND_RealtimeData[2048]={0};


void UART_Init()
{
	
	 GPIO_InitTypeDef GPIO_InitStructure;
	 USART_InitTypeDef USART_InitStructure; 
	  // PA2 PA3
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); // 使能 GPIOA 时钟 
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);// 使能 USART2 时钟

  
   GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;// TX 复用推挽输出 PA2 
	 GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; 
	 GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; // 复用推挽输出 
	 GPIO_Init(GPIOA,&GPIO_InitStructure); // 初始化 GPIO
	 GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;// RX 浮空输入 PA3 
	 GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING; // 浮空输入 
	 GPIO_Init(GPIOA,&GPIO_InitStructure); // 初始化 GPIO 


	 
   USART_InitStructure.USART_BaudRate =115200;// 波特率 115200 
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;// 字长 8 位数据 
   USART_InitStructure.USART_StopBits = USART_StopBits_1;// 1 位停止位 
   USART_InitStructure.USART_Parity = USART_Parity_No;// 无校验位 
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;// 无硬件流控制 
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式 
   USART_Init(USART2, &USART_InitStructure); // 初始化 USART2
	 
	 USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);// 开启接收中断 
	 USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);// 开启空闲中断 
	 NVIC_EnableIRQ(USART2_IRQn);

	 USART_Cmd(USART2, ENABLE); // 使能 USART2 
}


_Bool WIFI_Init()
{
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); // 使能 GPIOA 时钟 
   GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4 | GPIO_Pin_5;// PA4 PA5 
	 GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; 
	 GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP; // 推挽输出 
	 GPIO_Init(GPIOA,&GPIO_InitStructure); // 初始化 GPIO
   WIFI_ENABLE;
	 WIFI_UNRESET;
	 USART1_SendStr("wifi init...\r\n",sizeof("wifi init...\r\n"));
	 if(strstr((const char *)wifiRecvBuf,"CONNECTED"))
	 {
	   status = 3;
	   return 1;
	 }		
	 return testAT();
}

// STM32 发送数据到 ESP8266，长度为 len 字节
void USART2_Send(u8* data,int len)
{
	int i;
	for(i = 0;i < len;i++)
	{
    USART_SendData(USART2,(u8)data[i]); 
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	}

}

//int fputc(int ch,FILE *p) // 重定向 printf 函数到 USART2 
//{ 
//    USART_SendData(USART2,(u8)ch); 
//	  while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
//    return ch;
//} 


void USART2_IRQHandler(void)
{
    u8 rx;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        rx = USART_ReceiveData(USART2);
        if (recvCnt < (WIFI_RECV_BUF_SIZE - 1U))
        {
            wifiRecvBuf[recvCnt++] = rx;
        }
    }

    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        volatile u8 temp;
        temp = USART_ReceiveData(USART2);
        (void)temp;

        wifiRecvBuf[recvCnt] = '\0';
        if (recvCnt > 0U)
        {
            u16 frame_len = recvCnt;
            if (frame_len >= WIFI_RECV_BUF_SIZE)
                frame_len = WIFI_RECV_BUF_SIZE - 1U;
            memcpy((void *)downlinkFrame, (const void *)wifiRecvBuf, frame_len);
            downlinkFrame[frame_len] = '\0';
            downlinkLen = frame_len;
            downlinkReady = 1;
        }		
        wifiRecvOver = 1;
				  recvCnt = 0;
    }

    USART_ClearITPendingBit(USART2, USART_IT_IDLE);
}

_Bool Wifi_FetchDownlinkFrame(u8 *out, u16 out_size, u16 *out_len)
{
    u16 len;

    if (!out || out_size == 0 || !out_len)
        return 0;

    if (!downlinkReady)
        return 0;

    __disable_irq();
    len = downlinkLen;
    if (len >= out_size)
        len = out_size - 1U;
    memcpy(out, (const void *)downlinkFrame, len);
    out[len] = '\0';
    downlinkReady = 0;
    downlinkLen = 0;
    __enable_irq();

    *out_len = len;
    return 1;
}

//usart.c
 
/*
 * 函数名：cmdAT
 * 功能：  向 WF-ESP8266 发送 AT 指令
 * 参数：  cmdData：发送的命令字符串
 *         expReturn1、expReturn2：期望的返回字符串，如果为 NULL 则不检查返回值
 * 返回值：1：成功
 *         0：失败
 * 说明：  调用此函数
 */

_Bool cmdAT(char *cmdData,char *expReturn1,char *expReturn2,int len)
{
  
	_Bool res = 0;
	uint8_t count=0;
	// 发送 AT 命令到 WiFi 模块
	USART2_Send(cmdData,len);
	if (len < 2 || cmdData[len - 2] != '\r' || cmdData[len - 1] != '\n')
	{
		USART2_Send("\r\n",2);
	}
//	printf("cmdDataTmp:%s\r\nlenth:%d",cmdDataTmp,sizeof((char *)cmdDataTmp)/sizeof(char));
	if(expReturn1==NULL && expReturn2==NULL)
		return 1;
	DELAY_Nms(WIFI_WAIT_STEP_MS);
	while(count<WIFI_WAIT_CYCLES_CMDAT)
	{
		DELAY_Nms(WIFI_WAIT_STEP_MS);
		count++;
		// 检查 AT 命令响应
		if(expReturn2!=NULL)
		   res = (((_Bool)strstr((const char *)wifiRecvBuf,expReturn1))||((_Bool)strstr((const char *)wifiRecvBuf,expReturn2)));
		else
		   res = ((_Bool)strstr((const char *)wifiRecvBuf,expReturn1));
		
		if(res)  break;
	}
	// 清除缓冲区
	 memset((void *)wifiRecvBuf,0,WIFI_RECV_BUF_SIZE);
	 recvCnt=0;
	 return res;
}

_Bool Wifi_SendRaw(const u8 *data, int len, const char *expReturn1, const char *expReturn2)
{
    _Bool res = 0;
    uint8_t count = 0;

    if (data == NULL || len <= 0)
        return 0;
    USART2_Send((u8 *)data, len);

    if (expReturn1 == NULL && expReturn2 == NULL)
        return 1;

    DELAY_Nms(WIFI_WAIT_STEP_MS);
    while (count < WIFI_WAIT_CYCLES_RAW)
    {
        DELAY_Nms(WIFI_WAIT_STEP_MS);
        count++;
        if (expReturn2 != NULL)
            res = (((_Bool)strstr((const char *)wifiRecvBuf, expReturn1)) || ((_Bool)strstr((const char *)wifiRecvBuf, expReturn2)));
        else
            res = ((_Bool)strstr((const char *)wifiRecvBuf, expReturn1));

        if (res)
            break;
    }

    return res;
}
 
_Bool testAT( void )
{
	char count=0;
 
//	ESP8266_RST_H;	
	DELAY_Nms ( 100 );
	while ( count < 5 )
	{
		if( cmdAT ( "AT", "OK",NULL,strlen("AT")) )
    {
			USART1_SendStr("wifi test ok",sizeof("wifi test ok"));
      return 1;
    }
		WIFI_RESET;
		DELAY_Nms(500);
		WIFI_UNRESET;
		DELAY_Nms(500);
		++ count;
	}
	USART1_SendStr("wifi test failed!",sizeof("wifi test failed!"));
  return 0;
}
 
_Bool ESP8266_DHCP_CUR (void)
{
	return cmdAT( "AT+CWDHCP_CUR=1,1", "OK",NULL,strlen("AT+CWDHCP_CUR=1,1"));
}
 
/*
 * 函数名：ESP8266_Net_Mode_Choose
 * 功能：  设置 WF-ESP8266 的工作模式
 * 参数：  enumMode：工作模式
 * 返回值：1：成功
 *         0：失败
 * 说明：  调用此函数
 */
_Bool ESP8266_Net_Mode_Choose ( ENUM_Net_ModeTypeDef enumMode )
{
	switch ( enumMode )
	{
		case STA:
			return cmdAT ( "AT+CWMODE=1", "OK",NULL,strlen("AT+CWMODE=1")); 
		
	  case AP:
		  return cmdAT ( "AT+CWMODE=2", "OK",NULL,strlen("AT+CWMODE=2")); 
		
		case STA_AP:
		  return cmdAT ( "AT+CWMODE=3", "OK",NULL,strlen("AT+CWMODE=3")); 
		
	  default:
		  return 0;
  }
	
}
 
/*
 * 函数名：ESP8266_JoinAP
 * 功能：  WF-ESP8266 连接到 WiFi
 * 参数：  pSSID：WiFi 名称
 *         pPassWord：WiFi 密码
 * 返回值：1：成功
 *         0：失败
 * 说明：  调用此函数
 */
_Bool ESP8266_JoinAP ( char * pSSID, char * pPassWord )
{
	char cCmd [120];
 
	sprintf ( cCmd, "AT+CWJAP=\"%s\",\"%s\"", pSSID, pPassWord );
	
	return cmdAT ( cCmd, "OK","CONNECTED",strlen(cCmd));
	
}
 
/*
 * 函数名：ESP8266_Enable_MultipleId
 * 功能：  WF-ESP8266 启用多连接
 * 参数：  enumEnUnvarnishTx：多连接模式
 * 返回值：1：成功
 *         0：失败
 * 说明：  调用此函数
 */
_Bool ESP8266_Enable_MultipleId ( FunctionalState enumEnUnvarnishTx )
{
	char cStr [20];
	
	sprintf ( cStr, "AT+CIPMUX=%d", ( enumEnUnvarnishTx ? 1 : 0 ) );
	
	return cmdAT ( cStr, "OK",NULL,strlen(cStr));
	
}
 
/*

 */
_Bool ESP8266_Link_Server ( ENUM_NetPro_TypeDef enumE, char * ip, char * ComNum, ENUM_ID_NO_TypeDef id)
{
	char cStr [100] = { 0 }, cCmd [120];
 
  switch (  enumE )
  {
		case enumTCP:
		  sprintf ( cCmd, "AT+CIPSTART=\"%s\",\"%s\",%s", "TCP", ip, ComNum );
		  break;
		
		case enumUDP:
		  sprintf ( cCmd, "AT+CIPSTART=\"%s\",\"%s\",%s", "UDP", ip, ComNum );
		  break;
		
		default:
			break;
  }
 
	return cmdAT ( cCmd, "OK","ALREADY CONNECTED",strlen(cCmd));
	
}
 
/*
 * ????????ESP8266_UnvarnishSend
 * ????  ??????WF-ESP8266?????????????
 * ????  ????
 * ????  : 1?????????
 *         0?????????
 * ????  ??????????
 */
_Bool ESP8266_UnvarnishSend ( void )
{
	
 
}



_Bool TCP_Init()
{
    u8 retry_count = 0;
    const u8 max_retries = 50;  // 最多重试 50 次，避免无限循环

    status = 0;
    wifiReady = 0;
    int connect_len = 0;
    int subscribe_len = 0;

		while(retry_count < max_retries)
		{
			switch(status)
			{
				case 0: //???wifi???????
					USART1_SendStr("wifi init...0\r\n",sizeof("wifi init...0\r\n"));
					if(cmdAT("ATE0","OK",NULL,strlen("ATE0")))   
					status++;break;
				case 1: //????wifi????????????????s
					USART1_SendStr("wifi init...1\r\n",sizeof("wifi init...1\r\n"));
					if(ESP8266_Net_Mode_Choose(STA))             
					status++;break;//s?????STA??
				case 2: //????wifi???????????????
					USART1_SendStr("wifi init...2\r\n",sizeof("wifi init...2\r\n"));
				 if(ESP8266_JoinAP(WIFI_UserName,WIFI_PassWord))  //????????
				 status++; break;
				
				case 3://????wifi???
					USART1_SendStr("wifi init...3\r\n",sizeof("wifi init...3\r\n"));
                    if (cmdAT("AT+RST", "OK", "ready", strlen("AT+RST")))
                    {
                        DELAY_Nms(3000);  // 等待 ESP8266 重启完成
                        status++;
                    }
                    break;
				case 4://?????????????
					if(cmdAT ( "AT+CIPMUX=0", "OK", NULL ,strlen("AT+CIPMUX=0"))) //???????? 
						status++;
					break;
		case 5://??????????????????
			USART1_SendStr("wifi init...5\r\n",sizeof("wifi init...5\r\n"));
			{
				char a[64] = {0};
				sprintf(a,"AT+CIPSTART=\"TCP\",\"%s\",1883",Cloud_addr);
				if(cmdAT ( a, "OK", NULL ,strlen(a))) //??????????IP
					status++;
			}
			break;
		case 6://准备MQTT CONNECT数据
			USART1_SendStr("wifi init...6\r\n",sizeof("wifi init...6\r\n"));
			connect_len = ConnectMqtt("k0eldqgj","q7pj8jzucv54vpm4","sLWqI5ereL");
			{
				char cmd_wifi[32] = {0};
				sprintf(cmd_wifi,"AT+CIPSEND=%d",connect_len);
				if(cmdAT ( cmd_wifi, "OK", ">",strlen(cmd_wifi) ))
				{
					status++;
					wifiReady = 1;
				}
			}
			break;
		case 7://发送MQTT CONNECT数据
			USART1_SendStr("wifi init...7\r\n",sizeof("wifi init...7\r\n"));
			if(Wifi_SendRaw(MQTT_SEND_RealtimeData, connect_len, "SEND OK", NULL))
			{
				USART1_SendStr("MQTT CONNECT sent!\r\n",sizeof("MQTT CONNECT sent!\r\n"));
				status++;
			}
			break;

			case 8://准备MQTT SUBSCRIBE数据(attributes/push)
    USART1_SendStr("wifi init...10\r\n",sizeof("wifi init...10\r\n"));
    subscribe_len = MqttSubscribeTopic("attributes/push",0,1);
    {
        char cmd_Subscribe[25]={0};
        sprintf(cmd_Subscribe,"AT+CIPSEND=%d",subscribe_len);
        if(cmdAT ( cmd_Subscribe, "OK", ">",strlen(cmd_Subscribe) ))
            status++;
    }
    break;
       case 9://发送MQTT SUBSCRIBE数据
    USART1_SendStr("wifi init...11\r\n",sizeof("wifi init...11\r\n"));
    if(Wifi_SendRaw(MQTT_SEND_RealtimeData, subscribe_len, "SEND OK", NULL))
    {
        USART1_SendStr("MQTT SUBSCRIBE attributes/push sent!\r\n",sizeof("MQTT SUBSCRIBE attributes/push sent!\r\n"));
        status++;
        return 1; // 初始化完成
    }
    break;
			}
			DELAY_Nms(100);
            retry_count++;
	  }
      USART1_SendStr("WiFi init timeout!\r\n", sizeof("WiFi init timeout!\r\n"));
		return 0;
}


/******************************************
??????????wifi???????
??????
  sendbuf????????????
  recvbuf???????????????
???????
  ????????????????
*****************************************/
int Tcp_RW(char *sendbuf,char *recvbuf)
{
	int cnt = 0;
  if(!wifiReady)  return -1;
	if(sendbuf !=NULL)
		 printf("%s",sendbuf);
	if(recvbuf !=NULL)
	{
	   if(wifiRecvOver)
		 {
		    memcpy(recvbuf,(const void *)wifiRecvBuf,recvCnt);
			  cnt = recvCnt;
			  recvCnt = 0;
			  wifiRecvOver = 0;
			  memset((void *)wifiRecvBuf,0,WIFI_RECV_BUF_SIZE);
			 return cnt;
		 }     
	}
return 0;	
}

//????????????????
int ConnectMqtt(char *ClientID,char *Username,char *Password)
{
    int ClientIDLen = strlen(ClientID);
    int UsernameLen = strlen(Username);
    int PasswordLen = strlen(Password);
    int DataLen = 0;
    int Index = 0;
    int i = 0;
    DataLen = 16+ClientIDLen+UsernameLen+PasswordLen;
	//??????
    MQTT_SEND_RealtimeData[Index++] = 0x10;                //MQTT Message Type CONNECT
    if(DataLen<=127)
      MQTT_SEND_RealtimeData[Index++] = DataLen;
    else		
    {		
      MQTT_SEND_RealtimeData[Index++] = (DataLen%128)+0x80;//0xC3
      MQTT_SEND_RealtimeData[Index++] = DataLen/128;//78
    }
	//????
    MQTT_SEND_RealtimeData[Index++] = 0;        // Protocol Name Length MSB    
    MQTT_SEND_RealtimeData[Index++] = 4;        // Protocol Name Length LSB    
    MQTT_SEND_RealtimeData[Index++] = 'M';        // ASCII Code for M    
    MQTT_SEND_RealtimeData[Index++] = 'Q';        // ASCII Code for Q    
    MQTT_SEND_RealtimeData[Index++] = 'T';        // ASCII Code for T    
    MQTT_SEND_RealtimeData[Index++] = 'T';        // ASCII Code for T    
    MQTT_SEND_RealtimeData[Index++] = 4;        // MQTT Protocol version = 4    
    MQTT_SEND_RealtimeData[Index++] = 0xc2;        // conn flags 
    MQTT_SEND_RealtimeData[Index++] = 02;        // Keep-alive Time Length MSB    
    MQTT_SEND_RealtimeData[Index++] = 0x58;        // Keep-alive Time Length LSB  300S??????  
	//???????
    MQTT_SEND_RealtimeData[Index++] = (0xff00&ClientIDLen)>>8;// Client ID length MSB    
    MQTT_SEND_RealtimeData[Index++] = 0xff&ClientIDLen;    // Client ID length LSB  

    for(i = 0; i < ClientIDLen; i++)
    {
        MQTT_SEND_RealtimeData[Index + i] = ClientID[i];          
    }
    Index = Index + ClientIDLen;
    
    if(UsernameLen > 0)
    {   
        MQTT_SEND_RealtimeData[Index++] = (0xff00&UsernameLen)>>8;//username length MSB    
        MQTT_SEND_RealtimeData[Index++] = 0xff&UsernameLen;    //username length LSB    
        for(i = 0; i < UsernameLen ; i++)
        {
            MQTT_SEND_RealtimeData[Index + i] = Username[i];    
        }
        Index = Index + UsernameLen;
    }
    
    if(PasswordLen > 0)
    {    
        MQTT_SEND_RealtimeData[Index++] = (0xff00&PasswordLen)>>8;//password length MSB    
        MQTT_SEND_RealtimeData[Index++] = 0xff&PasswordLen;    //password length LSB    
        for(i = 0; i < PasswordLen ; i++)
        {
            MQTT_SEND_RealtimeData[Index + i] = Password[i];    
        }
        Index = Index + PasswordLen; 
    }    
    return Index;
}
//??????????
int MqttSubscribeTopic(char *topic,u8 qos,u8 whether)
{    
    int topiclen = strlen(topic);
    int i=0,index = 0;
  
    if(whether)//1????? 0????????
        MQTT_SEND_RealtimeData[index++] = 0x82;                        //0x82 //?????????? SUBSCRIBE ????
    else
        MQTT_SEND_RealtimeData[index++] = 0xA2;                        //0xA2 ???????
    MQTT_SEND_RealtimeData[index++] = topiclen + 4;                //?????(????????????)
    MQTT_SEND_RealtimeData[index++] = 0;                          //????????,????
    MQTT_SEND_RealtimeData[index++] = 0x01;                    //????????,????
    MQTT_SEND_RealtimeData[index++] = (0xff00&topiclen)>>8;    //??????(???????,???????)
    MQTT_SEND_RealtimeData[index++] = 0xff&topiclen;              //?????? 
    
    for (i = 0;i < topiclen; i++)
    {
        MQTT_SEND_RealtimeData[index + i] = topic[i];
    }
    index = index + topiclen;
    
    if(whether)//?????????????QOS??? ???????????
    {
        MQTT_SEND_RealtimeData[index] = qos;//QoS????
        index++;
		MQTT_SEND_RealtimeData[1]+=1;
    }

    return index;
}
//??????????????
int MqttPublishData(char * topic, char * message,int DataLen)
{  
    int topic_length = strlen(topic);    
    int message_length = DataLen; 
    int i,index=0;    
    u16 sum=0;
    
    MQTT_SEND_RealtimeData[index++] = 0x30;    // MQTT Message Type PUBLISH  

    sum = 2 + topic_length + message_length;   // ????? 
    if(sum<=127)
      MQTT_SEND_RealtimeData[index++] = sum;
    else		
    {		
      MQTT_SEND_RealtimeData[index++] = (sum%128)+0x80;//0xC3
      MQTT_SEND_RealtimeData[index++] = sum/128;//78
    }
    MQTT_SEND_RealtimeData[index++] = (0xff00&topic_length)>>8;//??????
    MQTT_SEND_RealtimeData[index++] = 0xff&topic_length;
    for(i = 0; i < topic_length; i++)
    {
        MQTT_SEND_RealtimeData[index + i] = topic[i];//????????
    }
    index += topic_length;
    for(i = 0; i < message_length; i++)       
    {
        MQTT_SEND_RealtimeData[index + i] = message[i];//????????
    }
    index += message_length;
    return index;
}
