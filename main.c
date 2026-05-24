
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
#include <stdarg.h>
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
#define SERVO_GPIO_PORT     GPIOA
#define SERVO_GPIO_PIN      GPIO_Pin_9  // PB9, TIM4 CH4
// JY003 Fan Module Pin
#define MOTOR_SIG_PIN       GPIO_Pin_0  // PB0 (避免与WiFi PA4冲突)
#define MOTOR_SIG_RCC       RCC_APB2Periph_GPIOB
#define MOTOR_SIG_PORT      GPIOB
#define SERVO_GPIO_RCC      RCC_APB2Periph_GPIOA
#define SERVO_ANGLE_MIN         500     /* 0度 最小脉冲 */
#define SERVO_ANGLE_MAX         2500    /* 180度 最大脉冲 */
#define SWEEP_STEP              40
#define SERVO_CONTINUOUS_MODE   1       /* 连续旋转模式, 0=180度角度扫描 */
#define SERVO_CW_PULSE          1620    /* 顺时针旋转脉冲 */
#define SERVO_BOOST_PULSE       1620
#define SERVO_BOOST_MS          0U
#define SERVO_UPDATE_DIV        1U
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
#define MQTT_SUB_TOPIC  "attributes/push"
static u8  Pub_Topic[]   = MQTT_PUB_TOPIC;
static u8  Pub_Message[256];
static u8  check_char[256];
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
static u8  remote_beep_active  = 0;
static u8  remote_led_active   = 0;
static u8  mqtt_send_pending   = 0;
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
/* ====================== 函数声明 ====================== */
static void Log_Print(const char *msg);
static void Log_ResetReason(void);
static void SerialBridge_Print(const char *json);
static u8   Wifi_Bringup(void);
static u8   IsTimeDue(u32 now, u32 *last_time, u32 period_ms);
static u32  GetSecondsOfDay(void);
static void FormatClockString(char *out, u16 out_len);
static u8   IsNightPeriod(void);
static void Human_LED_Init(void);
static void BEEP_Init_Safe(void);
static void BEEP_State(u8 state);
static void K_Key_Init(void);
static void Motor_Init(void);
static void Motor_Control(u8 status, u8 mode);
static void Send_MQTT_Process(void);
static void Update_SensorData(void);
static void Update_DisplayAndAlarm(void);
static void Handle_Keys(void);
static u8   JsonTryGetSwitch(const char *json, const char *key, u8 *out_value);
static void JsonNormalizeForMatch(char *buf);
static void Handle_WifiCommand(void);
static u8   ExtractIpdPayload(const char *src, char *dst, u16 dst_size);
_Bool Wifi_FetchDownlinkFrame(u8 *out, u16 out_size, u16 *out_len);

/* ====================== 日志/调试 ====================== */
static void Log_Print(const char *msg)
{
    USART1_SendStr((char *)msg, strlen(msg));
}
static void Log_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Log_Print(buf);
}
static void Log_ResetReason(void)
{
    Log_Print("ResetCause:");
    if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET) Log_Print(" POR");
    if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET) Log_Print(" PIN");
    if (RCC_GetFlagStatus(RCC_FLAG_SFTRST) != RESET) Log_Print(" SFT");
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) Log_Print(" IWDG");
    if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST) != RESET) Log_Print(" WWDG");
    if (RCC_GetFlagStatus(RCC_FLAG_LPWRRST) != RESET) Log_Print(" LPWR");
    Log_Print("\r\n");
    RCC_ClearFlag();
}
static void SerialBridge_Print(const char *json)
{
#if SERIAL_PC_BRIDGE
    Log_Print("BRIDGE:");
    Log_Print(json);
    Log_Print("\r\n");
#endif
}

/* ====================== Wi-Fi 初始化 ====================== */
static u8 Wifi_Bringup(void)
{
    u8 i;
    DELAY_Nms(WIFI_BOOT_DELAY_MS);
    for (i = 0; i < WIFI_INIT_RETRY; i++)
    {
        Log_Print("[WiFi] Bringup...\r\n");
        UART_Init();

        if (!cmdAT("AT", "OK", NULL, 2))
        {
            Log_Print("[WiFi] AT No Response\r\n");
            DELAY_Nms(300);
            continue;
        }
        WIFI_Init();
        if (TCP_Init() == 1)
        {
            Log_Print("[WiFi] TCP Connected\r\n");
            Log_Print("[WiFi] NET Ready -> MQTT/Downlink Enabled\r\n");
            mqtt_send_pending = 0;
            return 1;
        }
        Log_Print("[WiFi] TCP Retry...\r\n");
        DELAY_Nms(500);
    }
    Log_Print("[WiFi] TCP Connect Failed\r\n");
    mqtt_send_pending = 0;
    return 0;
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
static void BEEP_Init_Safe(void)
{
    BEEP_Config();
    BEEP_OFF();
}
static void BEEP_State(u8 state)
{
    if (state == 1)
    {
        GPIO_ResetBits(GPIOF, GPIO_Pin_2);  // 低电平使能蜂鸣器
    }
    else
    {
        GPIO_SetBits(GPIOF, GPIO_Pin_2);    // 高电平关闭蜂鸣器
    }
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
    // 上电默认关闭风扇（高电平）
    GPIO_SetBits(MOTOR_SIG_PORT, MOTOR_SIG_PIN);
    motor_status = 0;
    motor_control_mode = 0;
    Log_Print("[Init] Fan OFF\r\n");
}
/* ====================== 电机控制 ====================== */
static void Motor_Control(u8 status, u8 mode)
{
    if (status == 0)
    {
        GPIO_SetBits(MOTOR_SIG_PORT, MOTOR_SIG_PIN);    // 关闭风扇（JY003 信号线高电平关闭）
        Log_Print("[Fan] OFF\r\n");
    }
    else
    {
        GPIO_ResetBits(MOTOR_SIG_PORT, MOTOR_SIG_PIN);  // 开启风扇（JY003 信号线低电平开启）
        Log_Print("[Fan] ON\r\n");
    }
    motor_status = status;
    motor_control_mode = mode;
}
/* ====================== 修复1：MQTT 发送优化，确保 JSON 数据完整发送 ====================== */
/**
 * 原问题：每次发送只发送1个数据段，5个数据轮流发送，导致同一时刻 TEMP/HUM/MQ2/HUMAN/SERVO 五个数据不同时发送
 * 修复后：每次发送包含所有数据段的 JSON，确保同时发送所有数据
 */
static void Send_MQTT_Process(void)
{   
    int  pkt_len;
    char at_buf[32];
    if (!mqtt_send_pending)
        return;
    /* 构建包含传感器数据和人体检测结果的 JSON 上报内容 */
    snprintf((char *)Pub_Message, sizeof(Pub_Message),
             "{\"TEMP\":%d,\"HUM\":%d,\"MQ2\":%lu,\"HUMAN\":%s}",
             dht11_data[2],
             dht11_data[0],
             (unsigned long)ppm,
             human_status ? "true" : "false");

    Log_Print("PUB Topic:");
    Log_Print((char *)Pub_Topic);
    Log_Print(" | ");
    Log_Print((char *)Pub_Message);
    Log_Print("\r\n");
    SerialBridge_Print((char *)Pub_Message);
    /* 发送 MQTT PUBLISH 命令 */
    pkt_len = MqttPublishData((char *)Pub_Topic,
                              (char *)Pub_Message,
                              strlen((char *)Pub_Message));
    if (pkt_len <= 0)
    {
        Log_Print("MQTT Encode Fail\r\n");
        mqtt_send_pending = 0;
        return;
    }
    /* 发送 AT 命令 */
    snprintf(at_buf, sizeof(at_buf), "AT+CIPSEND=%d\r\n", pkt_len);
    if (cmdAT(at_buf, "OK", ">", strlen(at_buf)))
    {
        if (Wifi_SendRaw(MQTT_SEND_RealtimeData, pkt_len, "SEND OK", "SENDOK"))
        {
            Log_Print("MQTT Pub OK\r\n");
            mqtt_send_pending = 0;  /* 成功后才清除发送标志 */
        }
        else
        {
            Log_Print("MQTT Pub Fail - Buffer Content:\r\n");
            Log_Print((char *)wifiRecvBuf);
            Log_Print("\r\n");
        }
				mqtt_send_pending = 0;
    }
    else
    {
        Log_Print("AT+CIPSEND No Response\r\n");
    }
}
static void Update_SensorData(void)
{
    u32 mq2_corrected;
    u8  i;
    /* 读取 DHT11传感器数据 尝试3次 */
    for (i = 0; i < 3; i++)
    {
        if (DHT11_Getdata() == 0)
        {
            dht11_data[0] = data[0];   /* 湿度数据 */
            dht11_data[2] = data[2];   /* 温度数据 */
            break;
        }
        DELAY_Nms(20);
    }
    /* 读取 MQ2 从mq2.c 获取 ADC 原始值 */
    mq2_raw = MQ2_GetData();
    ppm    = mq2_raw;   /* 直接把 ADC 当成 PPM 上报 */

    /* 限制最大值，避免异常数据 */
    mq2_corrected = ppm;
    if (mq2_corrected > MQ2_REPORT_MAX)
        mq2_corrected = MQ2_REPORT_MAX;

    ppm = mq2_corrected;

    /* 开机前 MQ2 读数未稳定，不立即报警 */
    if (system_timer >= BOOT_STABILIZE_MS)
        mq2_ready = 1;
}

/* ====================== MQ2 自动校准 ====================== */
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

    if (alarm_trigger || remote_beep_active)
        BEEP_ON();
    else
        BEEP_OFF();

    if (prev_oled_mode != oled_mode)
    {
        OLED_Clear(0);
        prev_oled_mode = oled_mode;
    }
    FormatClockString(time_str, sizeof(time_str));
    /* 模式 0：显示实时数据 */
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
    /* 模式 1：设置菜单 */
    else if (oled_mode == 1)
    {
        OLED_ShowString(0, 0, (u8 *)"Settings Menu");
        OLED_ShowString(2, 0, (u8 *)(menu_index == 0 ? ">" : " ") );
        OLED_ShowString(2, 16, (u8 *)"Temp Threshold");
        OLED_ShowString(4, 0, (u8 *)(menu_index == 1 ? ">" : " ") );
        OLED_ShowString(4, 16, (u8 *)"Hum Threshold");
        OLED_ShowString(6, 0, (u8 *)(menu_index == 2 ? ">" : " ") );
        OLED_ShowString(6, 16, (u8 *)"MQ2 Threshold");
    }
    /* 模式 2：设置调整 */
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
    /* 模式 3：系统状态 */
    else if (oled_mode == 3)
    {
        OLED_ShowString(0, 0, (u8 *)"System Status");
        sprintf(buf, "FAN  :%s", motor_status ? "ON"     : "OFF");
        OLED_ShowString(2, 0, (u8 *)buf);

        sprintf(buf, "HUMAN:%s", human_status ? "YES"   : "NO");
        OLED_ShowString(4, 0, (u8 *)buf);

        sprintf(buf, "MODE :%s", motor_control_mode ? "MANUAL" : "AUTO");
        OLED_ShowString(6, 0, (u8 *)buf);
    }
    /* 模式 4：网络信息 */
    else if (oled_mode == 4)
    {
        OLED_ShowString(0, 0, (u8 *)"Network Info");
        sprintf(buf, "WiFi:%s", wifi_online ? "ONLINE" : "OFFLINE");
        OLED_ShowString(2, 0, (u8 *)buf);

        sprintf(buf, "MQTT:%s", mqtt_send_pending ? "PENDING" : "IDLE");
        OLED_ShowString(4, 0, (u8 *)buf);

        OLED_ShowString(6, 0, (u8 *)"Press KEY to exit");
    }
}

/* ====================== 按键处理 ====================== */
static void Handle_Keys(void)
{
    u8 key1_now = GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN);
    u8 key0_now = GPIO_ReadInputDataBit(KEY0_PORT, KEY0_PIN);

    u8 key1_pressed = (key1_now == Bit_RESET);  /* 低电平有效 */
    u8 key0_pressed = (key0_now == Bit_RESET);

    static u32 last_key_time = 0;
    if (!key_level_ready)
        key_level_ready = 1;
    // 按键消抖优化：每次有效操作后延时100ms
    if (system_timer - last_key_time < 100) return;

    /* KEY0 + KEY1 同时按下，切换 OLED 页面 */
    if (key1_pressed && key0_pressed)
    {
        if (!key_combo_latched)
        {
            DELAY_Nms(KEY_DEBOUNCE_MS);
            if ((GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN) != key1_idle_level) &&
                (GPIO_ReadInputDataBit(KEY0_PORT, KEY0_PIN) != key0_idle_level))
            {
                oled_mode = (oled_mode + 1) % 5;  // 寰?鐜? 0-4
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

    /* KEY1：增加 / 菜单导航 */
    if (key1_pressed && !key1_latched)
    {
        DELAY_Nms(KEY_DEBOUNCE_MS);
        if (oled_mode == 1)  // 菜单导航
        {
            menu_index = (menu_index + 1) % 3;
        }
        else if (oled_mode == 2)  // 设置调整
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
            Log_Print("[Key] 手动开风扇\r\n");
            Motor_Control(1, 1);
        }
        key1_latched = 1;
        last_key_time = system_timer;
    }
    else if (!key1_pressed)
        key1_latched = 0;

    /* KEY0：减少 / 菜单选择 */
    if (key0_pressed && !key0_latched)
    {
        DELAY_Nms(KEY_DEBOUNCE_MS);
        if (oled_mode == 1)  // 菜单选择
        {
            sub_menu_index = menu_index;  // 选择当前菜单项
            oled_mode = 2;  // 杩涘叆璁剧疆椤甸潰
        }
        else if (oled_mode == 2)  // 设置调整
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
            Log_Print("[Key] 手动关风扇\r\n");
            Motor_Control(0, 1);
        }
        key0_latched = 1;
        last_key_time = system_timer;
    }
    else if (!key0_pressed)
        key0_latched = 0;

    /* K_KEY锛圵KUP锛夊垏鎹? OLED 椤甸潰 */
    if (GPIO_ReadInputDataBit(K_KEY_PORT, K_KEY_PIN))
    {
        if (!k_key_latched)
        {
            DELAY_Nms(KEY_DEBOUNCE_MS);
            if (GPIO_ReadInputDataBit(K_KEY_PORT, K_KEY_PIN))
            {
                oled_mode = (oled_mode + 1) % 5;  // 寰?鐜? 0-4
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

/* ====================== ThingCloud 下行命令处理 ====================== */
/**
 * @brief  从 JSON 字符串中解析开关值
 * @retval 1=解析成功，0=未找到
 */
static u8 JsonTryGetSwitch(const char *json, const char *key, u8 *out_value)
{
    char pat_true[32], pat_false[32];
    char pat_one[32],  pat_zero[32];
    char pat_str_true[32], pat_str_false[32];
    char pat_str_one[32],  pat_str_zero[32];

    if (!json || !key || !out_value)
        return 0;
    snprintf(pat_true,      sizeof(pat_true),      "\"%s\":true",    key);
    snprintf(pat_false,     sizeof(pat_false),     "\"%s\":false",   key);
    snprintf(pat_one,       sizeof(pat_one),       "\"%s\":1",       key);
    snprintf(pat_zero,      sizeof(pat_zero),      "\"%s\":0",       key);
    snprintf(pat_str_true,  sizeof(pat_str_true),  "\"%s\":\"true\"",  key);
    snprintf(pat_str_false, sizeof(pat_str_false), "\"%s\":\"false\"", key);
    snprintf(pat_str_one,   sizeof(pat_str_one),   "\"%s\":\"1\"",   key);
    snprintf(pat_str_zero,  sizeof(pat_str_zero),  "\"%s\":\"0\"",   key);
    if (strstr(json, pat_true)  || strstr(json, pat_one) ||
        strstr(json, pat_str_true) || strstr(json, pat_str_one))
    {
        *out_value = 1;
        return 1;
    }
    if (strstr(json, pat_false) || strstr(json, pat_zero) ||
        strstr(json, pat_str_false) || strstr(json, pat_str_zero))
    {
        *out_value = 0;
        return 1;
    }
    return 0;
}

/**
 * @brief  去除 JSON 字符串中的空白符和转义符，便于 strstr 匹配
 */
static void JsonNormalizeForMatch(char *buf)
{
    u16 r = 0, w = 0;
    char c;

    if (!buf)
        return;

    while ((c = buf[r++]) != '\0')
    {
        if (c == '\\' || c == ' ' || c == '\r' || c == '\n' || c == '\t')
            continue;
        buf[w++] = c;
    }
    buf[w] = '\0';
}
static u8 ExtractIpdPayload(const char *src, char *dst, u16 dst_size)
{
    const char *ipd;
    const char *colon;
    u16 i = 0;

    if (!src || !dst || dst_size == 0)
        return 0;

    ipd = strstr(src, "+IPD,");
    if (!ipd)
        return 0;

    colon = strchr(ipd, ':');
    if (!colon)
        return 0;
    colon++;

    while (*colon != '\0' && i < (u16)(dst_size - 1U))
    {
        dst[i++] = *colon++;
    }
    dst[i] = '\0';
    return (i > 0U);
}

static void Handle_WifiCommand(void)
{
    u16 copy_len = 0;
    u8  cmd_val;
	  char parsed_buf[256];
     if (!Wifi_FetchDownlinkFrame(check_char, sizeof(check_char), &copy_len))
        return;
		 if (copy_len == 0)
        return;
     if (ExtractIpdPayload((char *)check_char, parsed_buf, sizeof(parsed_buf)))  
    {
        strncpy((char *)check_char, parsed_buf, sizeof(check_char) - 1U);
        check_char[sizeof(check_char) - 1U] = '\0';
    }
		    else
    {
        if (strcmp((char *)check_char, "SENDOK\r\n") == 0 ||
            strcmp((char *)check_char, "SEND OK\r\n") == 0 ||
            strcmp((char *)check_char, "OK\r\n") == 0)
            return;
    }
    JsonNormalizeForMatch((char *)check_char);
    if (strstr((char *)check_char, "\"BEEP\"") == NULL &&
        strstr((char *)check_char, "\"LED\"") == NULL &&
        strstr((char *)check_char, "\"FAN\"") == NULL )    
    {
        Log_Print("Downlink NoCtrlKey:");
        Log_Print((char *)check_char);
        Log_Print("\r\n");
        return;
    }
    Log_Print("Downlink:");
    Log_Print((char *)check_char);
    Log_Print("\r\n");
    cmd_val = 0;
    if (JsonTryGetSwitch((char *)check_char, "BEEP", &cmd_val))
        remote_beep_active = cmd_val;
    cmd_val = 0;
    if (JsonTryGetSwitch((char *)check_char, "LED", &cmd_val))
    {
        remote_led_active = cmd_val;
        if (remote_led_active)
            BOARD_LED_ON();
        else
            BOARD_LED_OFF();
    }
    cmd_val = 0;
    if (
        JsonTryGetSwitch((char *)check_char, "FAN", &cmd_val)
       )
    {
        Motor_Control(cmd_val, 1);
    }
}
/* ====================== 主函数 ====================== */
int main(void)
{
    /* ---------- 系统时钟和中断优先级 ---------- */
    NVIC_SetPriorityGrouping(2);
    /* ---------- 硬件初始化 ---------- */
    BEEP_OFF();
	  LED_Config();
    KEY_Config();
    USART1_Config(115200);          /* 注意：这里调用 USART1_Config 实际是 USART1_Init */
    DELAY_Nms(100);
    Log_Print("\r\n\r\n----BOOT----\r\n");
    Log_ResetReason();
    SoftIIC_Config();
    OLED_Config();
    OLED_Clear(0);
    /* OLED 显示标签 */
    OLED_ShowString(0, 0, (u8 *)"TIME:");
    OLED_ShowString(2, 0, (u8 *)"TEMP:");
    OLED_ShowString(4, 0, (u8 *)"HUM :");
    OLED_ShowString(6, 0, (u8 *)"MQ2 :");
    BOARD_LED_OFF();
    DHT11_Config();    
    ADCx_Init();                    /* MQ2 传感器 ADC 初始化，必须在 MQ2_Init 之前调用 */
    MQ2_Init();
    SR602_Init();
    Motor_Init();
    Human_LED_Init();
    K_Key_Init();
    /* 首次更新传感器数据，开机时显示全 0 */
    Update_SensorData();
    Log_Print("System Init OK\r\n");
    DELAY_Nms(BOOT_STABILIZE_MS);
    /* ---------- Wi-Fi 初始化 ---------- */
    wifi_online = 0;
    last_wifi_retry_time = system_timer;
    Log_Print("Local Mode Start\r\n");
#if WIFI_AUTO_BRINGUP
    wifi_online = Wifi_Bringup();
#endif

    /* ====================== 主循环 ====================== */
    while (1)
    {
        DELAY_Nms(LOOP_TICK_MS);
        system_timer += LOOP_TICK_MS;
        /* --- 传感器数据更新 每500ms --- */
        if (IsTimeDue(system_timer, &last_dht_time, TASK_SENSOR_MS))
           Update_SensorData();
        /* --- 人体检测 每100ms --- */
        if (IsTimeDue(system_timer, &last_human_check_time, TASK_HUMAN_MS))
        {
            u8 human_raw = SR602_Detect() ? 1 : 0;
            human_status = human_raw;
            if (!human_raw)
            {
                // last_human_low_time = system_timer;  // 删除未使用
            }
            else
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
        /* 人体检测 LED 控制，保持 HUMAN_LED_HOLD_MS */
        if ((system_timer - last_human_detect_time) < HUMAN_LED_HOLD_MS)
            HUMAN_LED_ON();
        else
            HUMAN_LED_OFF();
        /* --- 自动控制电机 --- */
        if (motor_control_mode == 0)
        {
            if (dht11_data[2] >= temp_threshold_on && motor_status == 0)
                Motor_Control(1, 0);
            else if (dht11_data[2] <= temp_threshold_off && motor_status == 1)
                Motor_Control(0, 0);
        }
        /* --- OLED 刷新 + 报警控制 每200ms --- */
        if (IsTimeDue(system_timer, &last_ui_time, TASK_UI_MS))
            Update_DisplayAndAlarm();
        /* --- 按键扫描 --- */
        Handle_Keys();
        /* --- MQTT 发送 修复3：预热 30s 后每 15s 发送一次 --- */
        if (wifi_online &&
            system_timer >= MQTT_WARMUP_MS &&
            IsTimeDue(system_timer, &last_mqtt_time, TASK_MQTT_MS))
        {
            /* 修复4：使用条件日志，避免原有的 int cnd */
            if (!mqtt_send_pending)
                mqtt_send_pending = 1;
        }
        /* --- Wi-Fi 命令处理 --- */
        if (wifi_online)
        {
          {

            Handle_WifiCommand();
            Send_MQTT_Process();
          }

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
                    wifi_retry_delay = WIFI_RETRY_MS * (1 << (wifi_retry_count > 4 ? 4 : wifi_retry_count));
                    if (wifi_retry_delay > 300000UL) wifi_retry_delay = 300000UL;
                    Log_Print("WiFi Retry Failed, next in ");
                }
            }
#endif
        }
    }
}
