#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include <stdint.h>
#include "gpio.h"
#include "usart.h"
#include "oled.h"
#include "tim.h"
#include "dht11.h"
#include "wifi.h"
#include "mq2.h"
#include "adcx.h"
#include "delay.h"
#include "adc.h"
#include "key.h"
#include "softiic.h"
#include <string.h>
#include <stdio.h>

#define BEEP_ON()           BEEP_State(1)
#define BEEP_OFF()          BEEP_State(0)
// 云平台控制 LED -> D0: PB5, 低电平点亮
#define BOARD_LED_PORT      GPIOB
#define BOARD_LED_PIN       GPIO_Pin_5
#define BOARD_LED_ON()      GPIO_ResetBits(BOARD_LED_PORT, BOARD_LED_PIN)
#define BOARD_LED_OFF()     GPIO_SetBits(BOARD_LED_PORT, BOARD_LED_PIN)
// 人体检测 LED -> D1: PE5, 低电平点亮
#define HUMAN_LED_PORT      GPIOE
#define HUMAN_LED_PIN       GPIO_Pin_5
#define HUMAN_LED_RCC       RCC_APB2Periph_GPIOE
#define HUMAN_LED_ON()      GPIO_ResetBits(HUMAN_LED_PORT, HUMAN_LED_PIN)
#define HUMAN_LED_OFF()     GPIO_SetBits(HUMAN_LED_PORT, HUMAN_LED_PIN)
#define KEY1_PORT           GPIOE
#define KEY1_PIN            GPIO_Pin_3
#define KEY0_PORT           GPIOE
#define KEY0_PIN            GPIO_Pin_4
#define K_KEY_PORT          GPIOA
#define K_KEY_PIN           GPIO_Pin_0
#define K_KEY_RCC           RCC_APB2Periph_GPIOA
// JY003 Fan Module Pin
#define MOTOR_SIG_PIN       GPIO_Pin_0  // PB0 (避免与WiFi PA4冲突)
#define MOTOR_SIG_RCC       RCC_APB2Periph_GPIOB
#define MOTOR_SIG_PORT      GPIOB
#define TEMP_THRESHOLD_ON_DEFAULT    30     /* 温度开启阈值 默认°C */
#define TEMP_THRESHOLD_OFF_DEFAULT   29     /* 温度关闭阈值 默认°C */
#define HUM_ALARM_THRESHOLD_DEFAULT  80     /* 湿度报警阈值 默认% */
#define MQ2_ALARM_THRESHOLD_DEFAULT  500    /* MQ2报警阈值 默认ppm */
#define MQ2_ADC_MAX             4095UL
#define MQ2_RL_OHM              5000UL      /* 负载电阻 千欧 */
#define MQ2_R0_OHM              10000UL     /* 标准空气电阻 千欧 */
#define MQ2_CLEAN_AIR_TARGET    200UL       /* 清洁空气目标 ppm */
#define MQ2_CALIB_SAMPLES       30U         /* 校准采样次数 */
#define MQ2_REPORT_MAX          9999UL      /* 报告最大值 */
#define LOOP_TICK_MS        20U
#define TASK_SENSOR_MS      100U
#define TASK_HUMAN_MS       100U
#define TASK_UI_MS          100U
#define TASK_MQTT_MS        10000U   // MQTT上报周期延长为10秒，减少串口发送频率
#define MQTT_WARMUP_MS      300U      /* 修复3：原来 120s 改为 30s */
#define HUMAN_LED_HOLD_MS   10000U
#define HUMAN_RETRIGGER_MS  2000U
#define KEY_DEBOUNCE_MS     10U
#define WIFI_BOOT_DELAY_MS  1500U
#define BOOT_STABILIZE_MS   3000U
#define WIFI_INIT_RETRY     3U
#define WIFI_RETRY_MS       30000U
#define WIFI_AUTO_BRINGUP   1U
#define NIGHT_START_HOUR    0U
#define NIGHT_END_HOUR      7U
#define CLOCK_START_HOUR    0U
#define CLOCK_START_MIN     0U
#define CLOCK_START_SEC     0U
#define NIGHT_BEEP_MS       3000U
#define USE_RTC_CLOCK       0U
#define SERIAL_PC_BRIDGE    1U
/* ====================== MQTT Topic ====================== */
#define MQTT_PUB_TOPIC  "attributes"

static u8  Pub_Topic[]   = MQTT_PUB_TOPIC;
static u8  Pub_Message[256];
static u32 ppm              = 0;
static u8  human_status     = 0;
static u8  temp_threshold_on  = TEMP_THRESHOLD_ON_DEFAULT;
static u8  temp_threshold_off = TEMP_THRESHOLD_OFF_DEFAULT;
static u8  hum_alarm_threshold  = HUM_ALARM_THRESHOLD_DEFAULT;
static u32 mq2_alarm_threshold  = MQ2_ALARM_THRESHOLD_DEFAULT;
static u8  motor_status       = 0;     /* 0=停止, 1=运行 */
static u8  motor_control_mode = 0;     /* 0=自动控制, 1=手动 */
static u8  dht11_data[5]         = {0};
static u16 mq2_raw               = 0;
static u8  mq2_ready             = 0;
static u8  beep_remote_active  = 0;  /* 1=云端下发蜂鸣器开 */
static u8  wifi_online         = 0;
extern u32 system_timer;
static u32 last_dht_time           = 0;
static u32 last_human_check_time   = 0;
static u32 last_human_detect_time  = 0;
static u32 last_mqtt_time          = 0;
static u32 last_ui_time            = 0;
static u32 last_wifi_retry_time    = 0;
static u8  key1_latched    = 0;
static u8  key0_latched    = 0;
static u8  k_key_latched   = 0;
static u8  key_combo_latched = 0;
static u8  key_level_ready = 0;
static u8  key1_idle_level = 1;
static u8  key0_idle_level = 1;
/* WiFi 重试计数 */
static u8  wifi_retry_count = 0;
static u32 wifi_retry_delay = WIFI_RETRY_MS;
/* OLED 显示模式 0=实时数据, 1=设置菜单, 2=设置调整, 3=系统状态, 4=网络信息 */
static u8  oled_mode       = 0;
static u8  menu_index      = 0;  // 主菜单索引
static u8  sub_menu_index  = 0;  // 子菜单索引
static u32 night_beep_until = 0;
static u8  human_raw_prev   = 0;
static u8  check_char[256];  /* 下发数据预处理缓冲 */
/* ====================== 函数声明 ====================== */
static void Log_Print(const char *msg);
static u8   Wifi_Bringup(void);
static u8   IsTimeDue(u32 now, u32 *last_time, u32 period_ms);
static u32  GetSecondsOfDay(void);
static void FormatClockString(char *out, u16 out_len);
static u8   IsNightPeriod(void);
static void Human_LED_Init(void);
static void BEEP_Init(void);
static void BEEP_State(u8 state);
static void K_Key_Init(void);
static void Motor_Init(void);
static void Motor_Control(u8 status, u8 mode);
static u8   Mqtt_Publish(const char *topic, const char *message);
static void Update_SensorData(void);
static void Update_DisplayAndAlarm(void);
static void Handle_Keys(void);
/* ---- 本地辅助函数（wifi.c 中不存在，在 main.c 中实现） ---- */
static void Wifi_ClearBuf(void);
/* ====================== 日志/调试 ====================== */
static void Log_Print(const char *msg)
{
    USART1_SendStr((char *)msg, strlen(msg));
}
/* ====================== Wi-Fi 初始化 ====================== */
static u8 Wifi_Bringup(void)
{
    u8 i;

    /* 初始化 PA4(EN) / PA5(RST)，确保 ESP8266 上电 */
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4 | GPIO_Pin_5;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        WIFI_DISABLE;
        WIFI_RESET;
        DELAY_Nms(100);
        WIFI_ENABLE;
        DELAY_Nms(100);
        WIFI_UNRESET;
        DELAY_Nms(500);
    }

    UART_Init();
    DELAY_Nms(WIFI_BOOT_DELAY_MS);

    for (i = 0; i < WIFI_INIT_RETRY; i++)
    {
        if (!cmdAT("AT", "OK", NULL, 2))
        {
            WIFI_RESET;
            DELAY_Nms(300);
            WIFI_UNRESET;
            DELAY_Nms(800);
            continue;
        }

        {
            u8 tcp_result = TCP_Init();
            if (tcp_result == 1)
            {
                return 1;
            }
        }
        DELAY_Nms(500);
    }
    return 0;
}

/* ====================== 本地辅助函数 ====================== */
static void Wifi_ClearBuf(void)
{
    memset(wifiRecvBuf, 0, sizeof(wifiRecvBuf));
    recvCnt = 0;
    wifiRecvOver = 0;
}

/* ====================== MQTT 上行发布 ====================== */
/**
 * @brief  直接通过 USART2 发送 MQTT 数据，不走 cmdAT，不破坏 wifiRecvBuf
 * @retval 1=成功, 0=失败
 */
static u8 Mqtt_Publish(const char *topic, const char *message)
{
    char cmd[32];
    int pkt_len;
    u8 i;

    /* 1. 编码 MQTT PUBLISH 包 */
    pkt_len = MqttPublishData((char *)topic, (char *)message, strlen(message));
    if (pkt_len <= 0) return 0;

    /* 2. 发送 AT+CIPSEND=长度 */
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d\r\n", pkt_len);
    USART2_Send((u8 *)cmd, strlen(cmd));
    for (i = 0; i < 50; i++)
    {
        DELAY_Nms(200);
        if (strstr((const char *)wifiRecvBuf, ">") != NULL) break;
    }
    if (i >= 50) return 0;
    Wifi_ClearBuf();  /* 仅清空 AT 应答，此时无下行数据 */

    /* 3. 发送 MQTT 原始数据 */
    USART2_Send(MQTT_SEND_RealtimeData, pkt_len);
    for (i = 0; i < 50; i++)
    {
        DELAY_Nms(200);
        if (strstr((const char *)wifiRecvBuf, "SEND OK") != NULL) break;
    }
    if (i >= 50) return 0;

    /* 4. 仅在没有下行数据时才清空缓冲区 */
    if (strstr((const char *)wifiRecvBuf, "+IPD,") == NULL)
        Wifi_ClearBuf();
    else
        wifiRecvOver = 1;  /* 标记有下行数据待处理 */

    return 1;
}

/* ====================== 时间管理 ====================== */
static u8 IsTimeDue(u32 now, u32 *last_time, u32 period_ms)
{
    if ((u32)(now - *last_time) >= period_ms)
    {
        *last_time = now;
        return 1;
    }
    return 0;
}
static u32 GetSecondsOfDay(void)
{
#if USE_RTC_CLOCK
    return (u32)(RTC_GetCounter() % 86400UL);
#else
    u32 start_sec = CLOCK_START_HOUR * 3600UL + CLOCK_START_MIN * 60UL + CLOCK_START_SEC;
    return ((system_timer / 1000UL) + start_sec) % 86400UL;
#endif
}

static void FormatClockString(char *out, u16 out_len)
{
    u32 sec_of_day = GetSecondsOfDay();
    u8  hh = (u8)(sec_of_day / 3600UL);
    u8  mm = (u8)((sec_of_day % 3600UL) / 60UL);
    u8  ss = (u8)(sec_of_day % 60UL);
    snprintf(out, out_len, "%02d:%02d:%02d", hh, mm, ss);
}
static u8 IsNightPeriod(void)
{
    u32 sec_of_day = GetSecondsOfDay();
    u8  hour = (u8)(sec_of_day / 3600UL);
    if (NIGHT_START_HOUR < NIGHT_END_HOUR)
        return (hour >= NIGHT_START_HOUR && hour < NIGHT_END_HOUR);
    return (hour >= NIGHT_START_HOUR || hour < NIGHT_END_HOUR);
}
/* ====================== 硬件初始化 ====================== */
static void Human_LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(HUMAN_LED_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = HUMAN_LED_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HUMAN_LED_PORT, &GPIO_InitStructure);
    HUMAN_LED_OFF();
}
static void BEEP_Init(void)
{
    /* 先拉高 PF2 再配置为输出，避免上电瞬间蜂鸣器误响 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
    GPIOF->BSRR = GPIO_Pin_2;
    BEEP_Config();
    BEEP_OFF();
}
static void BEEP_State(u8 state)
{
    if (state == 1)
        GPIO_ResetBits(GPIOF, GPIO_Pin_2);
    else
        GPIO_SetBits(GPIOF, GPIO_Pin_2);
}
static void K_Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(K_KEY_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = K_KEY_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(K_KEY_PORT, &GPIO_InitStructure);
}
static void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(MOTOR_SIG_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = MOTOR_SIG_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_SIG_PORT, &GPIO_InitStructure);
    GPIO_SetBits(MOTOR_SIG_PORT, MOTOR_SIG_PIN);
    motor_status = 0;
    motor_control_mode = 0;
    Log_Print("[Init] Fan OFF\r\n");
}
static void Motor_Control(u8 status, u8 mode)
{
    if (status == 0)
    {
        GPIO_SetBits(MOTOR_SIG_PORT, MOTOR_SIG_PIN);
        Log_Print("[Fan] OFF\r\n");
    }
    else
    {
        GPIO_ResetBits(MOTOR_SIG_PORT, MOTOR_SIG_PIN);
        Log_Print("[Fan] ON\r\n");
    }
    motor_status = status;
    motor_control_mode = mode;
}
/* ====================== 传感器数据采集 ====================== */
static void Update_SensorData(void)
{
    u32 mq2_corrected;
    u8  i;
    for (i = 0; i < 3; i++)
    {
        if (DHT11_Getdata() == 0)
        {
            dht11_data[0] = data[0];
            dht11_data[2] = data[2];
            break;
        }
        DELAY_Nms(20);
    }
    mq2_raw = MQ2_GetData();
    ppm    = mq2_raw;
    mq2_corrected = ppm;
    if (mq2_corrected > MQ2_REPORT_MAX)
        mq2_corrected = MQ2_REPORT_MAX;
    ppm = mq2_corrected;
    if (system_timer >= BOOT_STABILIZE_MS)
        mq2_ready = 1;
}
/* ====================== OLED 显示 + 报警控制 ====================== */
static void Update_DisplayAndAlarm(void)
{
    char buf[20];
    char time_str[12];
    static u8 prev_oled_mode = 0xFF;
    u8 night_human_alarm = (system_timer < night_beep_until) ? 1 : 0;
    u8 temp_valid = (dht11_data[2] >= 5 && dht11_data[2] <= 50);
    u8 hum_valid = (dht11_data[0] >= 10 && dht11_data[0] <= 90);
    u8 mq2_valid = mq2_ready;
    u8 alarm_trigger = ((temp_valid && dht11_data[2] >= temp_threshold_on)  ||
                        (hum_valid && dht11_data[0] >  hum_alarm_threshold) ||
                        (mq2_valid && ppm           >  mq2_alarm_threshold) ||
                        night_human_alarm);
    if (alarm_trigger || beep_remote_active)
        BEEP_ON();
    else
        BEEP_OFF();
    if (prev_oled_mode != oled_mode)
    {
        OLED_Clear(0);
        prev_oled_mode = oled_mode;
    }
    FormatClockString(time_str, sizeof(time_str));
    if (oled_mode == 0)
    {
        OLED_ShowString(0, 0,  (u8 *)"TIME:");
        OLED_ShowString(0, 48, (u8 *)time_str);
        snprintf(buf, sizeof(buf), "TEMP:%dC   ", dht11_data[2]);
        OLED_ShowString(2, 0, (u8 *)buf);
        snprintf(buf, sizeof(buf), "HUM :%d%%  ", dht11_data[0]);
        OLED_ShowString(4, 0, (u8 *)buf);
        snprintf(buf, sizeof(buf), "MQ2 :%lu    ", (unsigned long)ppm);
        OLED_ShowString(6, 0, (u8 *)buf);
    }
    else if (oled_mode == 1)
    {
        OLED_ShowString(0, 0, (u8 *)"Settings Menu");
        OLED_ShowString(2, 0, (u8 *)(menu_index == 0 ? ">" : " "));
        OLED_ShowString(2, 16, (u8 *)"Temp Threshold");
        OLED_ShowString(4, 0, (u8 *)(menu_index == 1 ? ">" : " "));
        OLED_ShowString(4, 16, (u8 *)"Hum Threshold");
        OLED_ShowString(6, 0, (u8 *)(menu_index == 2 ? ">" : " "));
        OLED_ShowString(6, 16, (u8 *)"MQ2 Threshold");
    }
    else if (oled_mode == 2)
    {
        if (sub_menu_index == 0)
        {
            snprintf(buf, sizeof(buf), "TEMP:%d   ", temp_threshold_on);
            OLED_ShowString(0, 16, (u8 *)buf);
            OLED_ShowString(2, 0, (u8 *)"Use KEY0/1 to adjust");
            OLED_ShowString(4, 0, (u8 *)"KEY1: +  KEY0: -");
        }
        else if (sub_menu_index == 1)
        {
            snprintf(buf, sizeof(buf), "HUM :%d   ", hum_alarm_threshold);
            OLED_ShowString(0, 16, (u8 *)buf);
            OLED_ShowString(2, 0, (u8 *)"Use KEY0/1 to adjust");
            OLED_ShowString(4, 0, (u8 *)"KEY1: +  KEY0: -");
        }
        else if (sub_menu_index == 2)
        {
            snprintf(buf, sizeof(buf), "MQ2 :%lu  ", (unsigned long)mq2_alarm_threshold);
            OLED_ShowString(0, 16, (u8 *)buf);
            OLED_ShowString(2, 0, (u8 *)"Use KEY0/1 to adjust");
            OLED_ShowString(4, 0, (u8 *)"KEY1: +  KEY0: -");
        }
    }
    else if (oled_mode == 3)
    {
        OLED_ShowString(0, 0, (u8 *)"System Status");
        sprintf(buf, "FAN  :%s", motor_status ? "ON" : "OFF");
        OLED_ShowString(2, 0, (u8 *)buf);
        sprintf(buf, "HUMAN:%s", human_status ? "YES" : "NO");
        OLED_ShowString(4, 0, (u8 *)buf);
        sprintf(buf, "MODE :%s", motor_control_mode ? "MANUAL" : "AUTO");
        OLED_ShowString(6, 0, (u8 *)buf);
    }
    else if (oled_mode == 4)
    {
        OLED_ShowString(0, 0, (u8 *)"Network Info");
        sprintf(buf, "WiFi:%s", wifi_online ? "ONLINE" : "OFFLINE");
        OLED_ShowString(2, 0, (u8 *)buf);
        sprintf(buf, "MQTT:%s", wifi_online ? "ON" : "--");
        OLED_ShowString(4, 0, (u8 *)buf);
        OLED_ShowString(6, 0, (u8 *)"Press KEY to exit");
    }
}
/* ====================== 按键处理 ====================== */
static void Handle_Keys(void)
{
    u8 key1_now = GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN);
    u8 key0_now = GPIO_ReadInputDataBit(KEY0_PORT, KEY0_PIN);
    u8 key1_pressed = (key1_now == Bit_RESET);
    u8 key0_pressed = (key0_now == Bit_RESET);
    static u32 last_key_time = 0;
    if (!key_level_ready)
        key_level_ready = 1;
    if (system_timer - last_key_time < 100) return;
    if (key1_pressed && key0_pressed)
    {
        if (!key_combo_latched)
        {
            DELAY_Nms(KEY_DEBOUNCE_MS);
            if ((GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN) != key1_idle_level) &&
                (GPIO_ReadInputDataBit(KEY0_PORT, KEY0_PIN) != key0_idle_level))
            {
                oled_mode = (oled_mode + 1) % 5;
                menu_index = 0;
                sub_menu_index = 0;
                key_combo_latched = 1;
            }
        }
    }
    else
    {
        key_combo_latched = 0;
    }
    if (key1_pressed && key0_pressed)
        return;
    if (key1_pressed && !key1_latched)
    {
        DELAY_Nms(KEY_DEBOUNCE_MS);
        if (oled_mode == 1)
            menu_index = (menu_index + 1) % 3;
        else if (oled_mode == 2)
        {
            if (sub_menu_index == 0 && temp_threshold_on < 60)
                temp_threshold_on++;
            else if (sub_menu_index == 1 && hum_alarm_threshold < 99)
                hum_alarm_threshold++;
            else if (sub_menu_index == 2 && mq2_alarm_threshold < 10000)
                mq2_alarm_threshold += 50;
        }
        else
        {
            Log_Print("[Key] Manual Fan ON\r\n");
            Motor_Control(1, 1);
        }
        key1_latched = 1;
        last_key_time = system_timer;
    }
    else if (!key1_pressed)
        key1_latched = 0;
    if (key0_pressed && !key0_latched)
    {
        DELAY_Nms(KEY_DEBOUNCE_MS);
        if (oled_mode == 1)
        {
            sub_menu_index = menu_index;
            oled_mode = 2;
        }
        else if (oled_mode == 2)
        {
            if (sub_menu_index == 0 && temp_threshold_on > 5)
                temp_threshold_on--;
            else if (sub_menu_index == 1 && hum_alarm_threshold > 20)
                hum_alarm_threshold--;
            else if (sub_menu_index == 2 && mq2_alarm_threshold > 100)
                mq2_alarm_threshold -= 50;
        }
        else
        {
            Log_Print("[Key] Manual Fan OFF\r\n");
            Motor_Control(0, 1);
        }
        key0_latched = 1;
        last_key_time = system_timer;
    }
    else if (!key0_pressed)
        key0_latched = 0;
    if (GPIO_ReadInputDataBit(K_KEY_PORT, K_KEY_PIN))
    {
        if (!k_key_latched)
        {
            DELAY_Nms(KEY_DEBOUNCE_MS);
            if (GPIO_ReadInputDataBit(K_KEY_PORT, K_KEY_PIN))
            {
                oled_mode = (oled_mode + 1) % 5;
                menu_index = 0;
                sub_menu_index = 0;
                k_key_latched = 1;
            }
        }
    }
    else
    {
        k_key_latched = 0;
    }
}
/* ====================== 主函数 ====================== */
int main(void)
{
    NVIC_SetPriorityGrouping(2);

    /* ---- 硬件初始化 ---- */
    BEEP_Init();
    LED_Config();
    KEY_Config();
    USART1_Config(115200);
    DELAY_Nms(100);
    Log_Print("\r\n\r\n----BOOT----\r\n");
    SoftIIC_Config();
    OLED_Config();
    OLED_Clear(0);
    OLED_ShowString(0, 0, (u8 *)"TIME:");
    OLED_ShowString(2, 0, (u8 *)"TEMP:");
    OLED_ShowString(4, 0, (u8 *)"HUM :");
    OLED_ShowString(6, 0, (u8 *)"MQ2 :");
    BOARD_LED_OFF();
    DHT11_Config();
    ADCx_Init();
    MQ2_Init();
    SR602_Init();
    Motor_Init();
    Human_LED_Init();
    K_Key_Init();

    Update_SensorData();
    Log_Print("System Init OK\r\n");
    DELAY_Nms(BOOT_STABILIZE_MS);

    /* ---- WiFi 初始化 ---- */
    wifi_online = 0;
    last_wifi_retry_time = system_timer;
#if WIFI_AUTO_BRINGUP
    wifi_online = Wifi_Bringup();
#endif

    /* ====================== 主循环 ====================== */
    while (1)
    {
        DELAY_Nms(LOOP_TICK_MS);
        system_timer += LOOP_TICK_MS;

        /* --- 传感器更新 --- */
        if (IsTimeDue(system_timer, &last_dht_time, TASK_SENSOR_MS))
            Update_SensorData();

        /* --- 人体检测 --- */
        if (IsTimeDue(system_timer, &last_human_check_time, TASK_HUMAN_MS))
        {
            u8 human_raw = SR602_Detect() ? 1 : 0;
            human_status = human_raw;
            if (human_raw)
            {
                if (!human_raw_prev ||
                    ((system_timer - last_human_detect_time) >= HUMAN_RETRIGGER_MS))
                {
                    last_human_detect_time = system_timer;
                    if (IsNightPeriod())
                        night_beep_until = system_timer + NIGHT_BEEP_MS;
                }
            }
            human_raw_prev = human_raw;
        }

        /* 人体 LED */
        if ((system_timer - last_human_detect_time) < HUMAN_LED_HOLD_MS)
            HUMAN_LED_ON();
        else
            HUMAN_LED_OFF();

        /* --- 自动风扇 --- */
        if (motor_control_mode == 0)
        {
            if (dht11_data[2] >= temp_threshold_on && motor_status == 0)
                Motor_Control(1, 0);
            else if (dht11_data[2] <= temp_threshold_off && motor_status == 1)
                Motor_Control(0, 0);
        }

        /* --- OLED + 报警 --- */
        if (IsTimeDue(system_timer, &last_ui_time, TASK_UI_MS))
            Update_DisplayAndAlarm();

        /* --- 按键 --- */
        Handle_Keys();

        /* ========== WiFi 上下行处理 ========== */
        if (wifi_online)
        {
            u8 got_downlink = 0;

            /* ---- 下行优先：先处理服务器控制指令 ---- */
            if (wifiRecvOver == 1)
            {
                got_downlink = 1;
                {
                    u8 i, j = 0;
                    /* 预处理：去掉空白字符，跳过二进制0x00，再匹配 */
                    for (i = 0; i < recvCnt && j < sizeof(check_char) - 1; i++)
                    {
                        char c = wifiRecvBuf[i];
                        if (c == 0) continue;  /* 跳过MQTT包头中的0x00 */
                        if (c == ' ' || c == '\r' || c == '\n' || c == '\t') continue;
                        check_char[j++] = c;
                    }
                    check_char[j] = '\0';

                    /* LED 控制 */
                    if (strstr((char *)check_char, "{\"LED\":true}"))
                    {
                        USART1_SendStr("Recv: LED ON\r\n", sizeof("Recv: LED ON\r\n"));
                        BOARD_LED_ON();
                    }
                    if (strstr((char *)check_char, "{\"LED\":false}"))
                    {
                        USART1_SendStr("Recv: LED OFF\r\n", sizeof("Recv: LED OFF\r\n"));
                        BOARD_LED_OFF();
                    }

                    /* 蜂鸣器控制（持久标记，不被报警覆盖） */
                    if (strstr((char *)check_char, "{\"BEEP\":true}"))
                    {
                        USART1_SendStr("Recv: BEEP ON\r\n", sizeof("Recv: BEEP ON\r\n"));
                        beep_remote_active = 1;
                    }
                    if (strstr((char *)check_char, "{\"BEEP\":false}"))
                    {
                        USART1_SendStr("Recv: BEEP OFF\r\n", sizeof("Recv: BEEP OFF\r\n"));
                        beep_remote_active = 0;
                    }

                    /* 风扇/电机控制 */
                    if (strstr((char *)check_char, "{\"FAN\":true}"))
                    {
                        USART1_SendStr("Recv: FAN ON\r\n", sizeof("Recv: FAN ON\r\n"));
                        Motor_Control(1, 1);
                    }
                    if (strstr((char *)check_char, "{\"FAN\":false}"))
                    {
                        USART1_SendStr("Recv: FAN OFF\r\n", sizeof("Recv: FAN OFF\r\n"));
                        Motor_Control(0, 1);
                    }
                }

                /* 重置WiFi接收缓存 */
                memset(wifiRecvBuf, 0, recvCnt);
                recvCnt = 0;
                wifiRecvOver = 0;
            }

            /* ---- 上行：无下发时定时上报传感器数据 ---- */
#if 1
            if (!got_downlink &&
                system_timer >= MQTT_WARMUP_MS &&
                IsTimeDue(system_timer, &last_mqtt_time, TASK_MQTT_MS))
            {
                snprintf((char *)Pub_Message, sizeof(Pub_Message),
                         "{\"TEMP\":%d,\"HUM\":%d,\"MQ2\":%lu,\"HUMAN\":%s}",
                         dht11_data[2], dht11_data[0],
                         (unsigned long)ppm,
                         human_status ? "true" : "false");

                USART1_SendStr("PUB:", 4);
                USART1_SendStr((char *)Pub_Message, strlen((char *)Pub_Message));
                USART1_SendStr("\r\n", 2);

                Mqtt_Publish((const char *)Pub_Topic,
                             (const char *)Pub_Message);
            }
#endif  /* 调试阶段关闭上报 */

            wifi_retry_count = 0;
            wifi_retry_delay = WIFI_RETRY_MS;
        }
        else
        {
#if WIFI_AUTO_BRINGUP
            if (IsTimeDue(system_timer, &last_wifi_retry_time, wifi_retry_delay))
            {
                if (Wifi_Bringup())
                {
                    wifi_online = 1;
                    wifi_retry_count = 0;
                    wifi_retry_delay = WIFI_RETRY_MS;
                    Log_Print("WiFi Reconnected\r\n");
                }
                else
                {
                    wifi_retry_count++;
                    wifi_retry_delay = WIFI_RETRY_MS *
                        (1 << (wifi_retry_count > 4 ? 4 : wifi_retry_count));
                    if (wifi_retry_delay > 300000UL) wifi_retry_delay = 300000UL;
                    Log_Print("WiFi Retry Failed\r\n");
                }
            }
#endif
        }
    }
}
