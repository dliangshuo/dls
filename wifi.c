#include "stm32f10x.h"
#include "stdio.h"
#include "string.h"
#include "oled.h"
#include "wifi.h"
#include "delay.h"
#include "usart.h"
u8 recvCnt = 0;
u8 wifiRecvBuf[100];
u8 wifiReady = 0;
u8 wifiRecvOver = 0;
int status = 0;


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
	  //PA2  PA3
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //使能 GPIOA 时钟 
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能 USART2 时钟

  
   GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//TX 串口输出 PA2 
	 GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; 
	 GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; //复用推挽输出 
	 GPIO_Init(GPIOA,&GPIO_InitStructure); // 初始化串口输入 IO
	 GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;//RX 串口输入 PA3 
	 GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING; //浮空输入 
	 GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化 GPIO 


	 
   USART_InitStructure.USART_BaudRate =115200;//波特率设置 
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为 8 位数据格式 
   USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位 
   USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位 
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制 
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式 
   USART_Init(USART2, &USART_InitStructure); //初始化串口 1
	 
	 USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启接收中断 
	 USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//开启接收中断 
	 NVIC_EnableIRQ(USART2_IRQn);

	 USART_Cmd(USART2, ENABLE); //使能串口 1 
}


_Bool WIFI_Init()
{
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //使能 GPIOA 时钟 
   GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4 | GPIO_Pin_5;//PA4 PA5 
	 GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; 
	 GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP; //复用推挽输出 
	 GPIO_Init(GPIOA,&GPIO_InitStructure); // 初始化串口输入 IO
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

//stm32将长度为len的数据通过串口2发送出去
void USART2_Send(u8* data,int len)
{
	int i;
	for(i = 0;i < len;i++)
	{
    USART_SendData(USART2,(u8)data[i]); 
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	}

}

//int fputc(int ch,FILE *p) //函数默认的，在使用 printf 函数时自动调用 
//{ 
//    USART_SendData(USART2,(u8)ch); 
//	  while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
//    return ch;
//} 


void USART2_IRQHandler(void) //串口 1 中断服务程序 
{ 
		if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //判断 USARTx 的中断类型 USART_IT 是否产生中
		{ 
		   wifiRecvBuf[recvCnt++] =USART_ReceiveData(USART2);//串口接收读取到的数据//(USART1->DR);    
		} 
		USART_ClearFlag(USART2,USART_FLAG_RXNE); //最后通常会调用一个清除中断标志位的函数， 
		
		if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) //判断 USARTx 的中断类型 USART_IT 是否产生中
		{ 
		   wifiRecvBuf[recvCnt++] =USART_ReceiveData(USART2);//串口接收读取到的数据//(USART1->DR);  
       wifiRecvOver = 1;
//			USART1_SendStr(wifiRecvBuf,recvCnt);
			
			
		} 
		USART_ClearFlag(USART2,USART_FLAG_IDLE); //最后通常会调用一个清除中断标志位的函数， 
}


//usart.c
 
/*
 * 函数名：cmdAT
 * 描述  ：对WF-ESP8266模块发送AT指令
 * 输入  ：cmdData，待发送的指令
 *         expReturn1，expReturn2，期待的响应，为NULL表不需响应，两者为或逻辑关系
 * 返回  : 1，指令发送成功
 *         0，指令发送失败
 * 调用  ：被外部调用
 */

_Bool cmdAT(char *cmdData,char *expReturn1,char *expReturn2,int len)
{
  
	_Bool res = 0;
	//显示发送的指令内容，验证指令是否错误
	USART1_SendStr(cmdData,len);
	USART1_SendStr("\r\n",2);
	//向wifi模块发送AT指令
	USART2_Send(cmdData,len);
	USART2_Send("\r\n",2);
//	printf("cmdDataTmp:%s\r\nlenth:%d",cmdDataTmp,sizeof((char *)cmdDataTmp)/sizeof(char));
	if(expReturn1==NULL && expReturn2==NULL)
		return 1;
	DELAY_Nms(100);
	uint8_t count=0;
	 wifiRecvOver = 0;
	while(count<50)
	{
		DELAY_Nms(300);
		count++;
		//判断AT指令执行是否成功
		if(expReturn2!=NULL)
		   res = (((_Bool)strstr((const char *)wifiRecvBuf,expReturn1))||((_Bool)strstr((const char *)wifiRecvBuf,expReturn2)));
		else
		   res = ((_Bool)strstr((const char *)wifiRecvBuf,expReturn1));
		
		if(res)  break;
	}
	/* 若缓冲区中有+IPD下行数据（云平台下发），保留不清理 */
	if (strstr((const char *)wifiRecvBuf, "+IPD,") != NULL)
	{
		wifiRecvOver = 1;
		return res;
	}
	//清空串口2接收
	 memset(wifiRecvBuf,0,sizeof(wifiRecvBuf));
	 recvCnt=0;
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
 * 描述  ：选择WF-ESP8266模块的工作模式
 * 输入  ：enumMode，工作模式
 * 返回  : 1，选择成功
 *         0，选择失败
 * 调用  ：被外部调用
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
 * 描述  ：WF-ESP8266模块连接外部WiFi
 * 输入  ：pSSID，WiFi名称字符串
 *       ：pPassWord，WiFi密码字符串
 * 返回  : 1，连接成功
 *         0，连接失败
 * 调用  ：被外部调用
 */
_Bool ESP8266_JoinAP ( char * pSSID, char * pPassWord )
{
	char cCmd [120];
 
	sprintf ( cCmd, "AT+CWJAP=\"%s\",\"%s\"", pSSID, pPassWord );
	
	return cmdAT ( cCmd, "OK","CONNECTED",strlen(cCmd));
	
}
 
/*
 * 函数名：ESP8266_Enable_MultipleId
 * 描述  ：WF-ESP8266模块启动多连接
 * 输入  ：enumEnUnvarnishTx，配置是否多连接
 * 返回  : 1，配置成功
 *         0，配置失败
 * 调用  ：被外部调用
 */
_Bool ESP8266_Enable_MultipleId ( FunctionalState enumEnUnvarnishTx )
{
	char cStr [20];
	
	sprintf ( cStr, "AT+CIPMUX=%d", ( enumEnUnvarnishTx ? 1 : 0 ) );
	
	return cmdAT ( cStr, "OK",NULL,strlen(cStr));
	
}
 
/*
 * 函数名：ESP8266_Link_Server
 * 描述  ：WF-ESP8266模块连接外部服务器
 * 输入  ：enumE，网络协议
 *       ：ip，服务器IP字符串
 *       ：ComNum，服务器端口字符串
 *       ：id，模块连接服务器的ID
 * 返回  : 1，连接成功
 *         0，连接失败
 * 调用  ：被外部调用
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
 * 函数名：ESP8266_UnvarnishSend
 * 描述  ：配置WF-ESP8266模块进入透传发送
 * 输入  ：无
 * 返回  : 1，配置成功
 *         0，配置失败
 * 调用  ：被外部调用
 */
_Bool ESP8266_UnvarnishSend ( void )
{
	
 
}



_Bool TCP_Init()
{
    
		while(1)
		{
			switch(status)
			{
				case 0: //关闭wifi模块的回显
					USART1_SendStr("wifi init...0\r\n",sizeof("wifi init...0\r\n"));
					if(cmdAT("ATE0","OK",NULL,strlen("ATE0")))   
					status++;break;
				case 1: //设置wifi模块的模式为连接热点模式s
					USART1_SendStr("wifi init...1\r\n",sizeof("wifi init...1\r\n"));
					if(ESP8266_Net_Mode_Choose(STA))             
					status++;break;//s设置为STA模式
				case 2: //设置wifi模块要连接的热点账号
					USART1_SendStr("wifi init...2\r\n",sizeof("wifi init...2\r\n"));
				 if(ESP8266_JoinAP(WIFI_UserName,WIFI_PassWord))  //连接的热点
				 status++; break;
				
				case 3://重启wifi模块
					USART1_SendStr("wifi init...3\r\n",sizeof("wifi init...3\r\n"));
					cmdAT ( "AT+RST", "OK", NULL ,strlen("AT+RST"));
					status++;break;
				case 4: //设置wifi模块为TCP客户端，即单连接模式
					USART1_SendStr("wifi init...4\r\n",sizeof("wifi init...4\r\n"));			
				  if(cmdAT ( "AT+CIPMUX=0", "OK", NULL ,strlen("AT+CIPMUX=0"))) //单连接模式 
						status++;
					break;
			
				case 5://设置要连接的服务器地址
					USART1_SendStr("wifi init...5\r\n",sizeof("wifi init...5\r\n"));	
				u8 a[48]={0};
				sprintf(a,"AT+CIPSTART=\"TCP\",\"%s\",1883",Cloud_addr);
					if(cmdAT ( a, "OK", NULL ,strlen("AT+CIPSTART=\"TCP\",\"sh-3-mqtt.iot-api.com\",1883"))) //设置服务器IP
						status++;
					break;
				
				case 6://设置向服务器发送的数据数量
					USART1_SendStr("wifi init...6\r\n",sizeof("wifi init...6\r\n"));
					u8 connect_len = ConnectMqtt(NULL,UserName,PassWord);
					USART1_SendStr(MQTT_SEND_RealtimeData,connect_len);
					u8 cmd_wifi[25]={0};
					sprintf(cmd_wifi,"AT+CIPSEND=%d",connect_len);				
					if(cmdAT ( cmd_wifi, "OK", ">",strlen(cmd_wifi) ))  
						status++;
					wifiReady = 1;break;//发送数据
				
				case 7://向服务器发送MQTT连接报文
					USART1_SendStr("wifi init...7\r\n",sizeof("wifi init...7\r\n"));
					
					if(cmdAT ( MQTT_SEND_RealtimeData, "SEND OK", "+IPD,4" ,connect_len)) 
					{ 
						USART1_SendStr("connect ok!\r\n",sizeof("connect ok!\r\n"));
						status++;
					}
					break;
				case 8://设置向服务器发送的数据数量
					USART1_SendStr("wifi init...8\r\n",sizeof("wifi init...8\r\n"));
					u8 Subscribe_len = MqttSubscribeTopic("data/stream/set",0,1);//组装订阅报文
					USART1_SendStr(MQTT_SEND_RealtimeData,Subscribe_len);
					u8 cmd_Subscribe[25]={0};
					sprintf(cmd_Subscribe,"AT+CIPSEND=%d",Subscribe_len);				
					if(cmdAT ( cmd_Subscribe, "OK", ">",strlen(cmd_Subscribe) ))  
						status++;
					wifiReady = 1;break;//发送数据	
				case 9://向服务器发送MQTT订阅报文
				USART1_SendStr("wifi init...9\r\n",sizeof("wifi init...9\r\n"));
				
				if(cmdAT ( MQTT_SEND_RealtimeData, "SEND OK", "+IPD,4" ,Subscribe_len)) 
				{ 
					USART1_SendStr("Subscribe ok!\r\n",sizeof("Subscribe ok!\r\n"));
					status++;
					return 1;
				}
				default:
					return 2;	
			}
			DELAY_Nms(100);
	  }
		return 0;
}


/******************************************
进入透传后的wifi收发函数
参数：
  sendbuf：要发送的数组
  recvbuf：接收数据的数组
返回值：
  接收到的数据的数量
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
		    memcpy(recvbuf,wifiRecvBuf,recvCnt);
			  cnt = recvCnt;
			  recvCnt = 0;
			  wifiRecvOver = 0;
			  memset(wifiRecvBuf,0,sizeof(wifiRecvBuf));
			 return cnt;
		 }     
	}
return 0;	
}

//连接服务器打包函数
int ConnectMqtt(char *ClientID,char *Username,char *Password)
{
    int ClientIDLen = strlen(ClientID);
    int UsernameLen = strlen(Username);
    int PasswordLen = strlen(Password);
    int DataLen = 0;
    int Index = 0;
    int i = 0;
    DataLen = 16+ClientIDLen+UsernameLen+PasswordLen;
	//固定报头
    MQTT_SEND_RealtimeData[Index++] = 0x10;                //MQTT Message Type CONNECT
    if(DataLen<=127)
      MQTT_SEND_RealtimeData[Index++] = DataLen;
    else		
    {		
      MQTT_SEND_RealtimeData[Index++] = (DataLen%128)+0x80;//0xC3
      MQTT_SEND_RealtimeData[Index++] = DataLen/128;//78
    }
	//可变报头
    MQTT_SEND_RealtimeData[Index++] = 0;        // Protocol Name Length MSB    
    MQTT_SEND_RealtimeData[Index++] = 4;        // Protocol Name Length LSB    
    MQTT_SEND_RealtimeData[Index++] = 'M';        // ASCII Code for M    
    MQTT_SEND_RealtimeData[Index++] = 'Q';        // ASCII Code for Q    
    MQTT_SEND_RealtimeData[Index++] = 'T';        // ASCII Code for T    
    MQTT_SEND_RealtimeData[Index++] = 'T';        // ASCII Code for T    
    MQTT_SEND_RealtimeData[Index++] = 4;        // MQTT Protocol version = 4    
    MQTT_SEND_RealtimeData[Index++] = 0xc2;        // conn flags 
    MQTT_SEND_RealtimeData[Index++] = 02;        // Keep-alive Time Length MSB    
    MQTT_SEND_RealtimeData[Index++] = 0x58;        // Keep-alive Time Length LSB  300S心跳包  
	//有效载荷
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
//订阅封装函数
int MqttSubscribeTopic(char *topic,u8 qos,u8 whether)
{    
    int topiclen = strlen(topic);
    int i=0,index = 0;
  
    if(whether)//1为订阅 0为取消订阅
        MQTT_SEND_RealtimeData[index++] = 0x82;                        //0x82 //消息类型和标志 SUBSCRIBE 订阅
    else
        MQTT_SEND_RealtimeData[index++] = 0xA2;                        //0xA2 取消订阅
    MQTT_SEND_RealtimeData[index++] = topiclen + 4;                //剩余长度(不包括固定头部)
    MQTT_SEND_RealtimeData[index++] = 0;                          //消息标识符,高位
    MQTT_SEND_RealtimeData[index++] = 0x01;                    //消息标识符,低位
    MQTT_SEND_RealtimeData[index++] = (0xff00&topiclen)>>8;    //主题长度(高位在前,低位在后)
    MQTT_SEND_RealtimeData[index++] = 0xff&topiclen;              //主题长度 
    
    for (i = 0;i < topiclen; i++)
    {
        MQTT_SEND_RealtimeData[index + i] = topic[i];
    }
    index = index + topiclen;
    
    if(whether)//订阅的时候需要提供QOS要求 取消订阅不需要
    {
        MQTT_SEND_RealtimeData[index] = qos;//QoS级别
        index++;
		MQTT_SEND_RealtimeData[1]+=1;
    }

    return index;
}
//发布数据封装函数
int MqttPublishData(char * topic, char * message,int DataLen)
{  
    int topic_length = strlen(topic);    
    int message_length = DataLen; 
    int i,index=0;    
    u16 sum=0;
    
    MQTT_SEND_RealtimeData[index++] = 0x30;    // MQTT Message Type PUBLISH  

    sum = 2 + topic_length + message_length;   // 剩余长度 
    if(sum<=127)
      MQTT_SEND_RealtimeData[index++] = sum;
    else		
    {		
      MQTT_SEND_RealtimeData[index++] = (sum%128)+0x80;//0xC3
      MQTT_SEND_RealtimeData[index++] = sum/128;//78
    }
    MQTT_SEND_RealtimeData[index++] = (0xff00&topic_length)>>8;//主题长度
    MQTT_SEND_RealtimeData[index++] = 0xff&topic_length;
    for(i = 0; i < topic_length; i++)
    {
        MQTT_SEND_RealtimeData[index + i] = topic[i];//拷贝主题
    }
    index += topic_length;
    for(i = 0; i < message_length; i++)       
    {
        MQTT_SEND_RealtimeData[index + i] = message[i];//拷贝数据
    }
    index += message_length;
    return index;
}

