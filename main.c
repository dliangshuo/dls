/**
 * @file    main.c
 * @brief   STM32 IoT 环境监控系统主程序（完整优化版）
 *
 * 传感器配置：
 *   - DHT11  温湿度传感器
 *   - MQ-2   烟雾/可燃气体传感器（AO 模拟）
 *   - SR602  人体红外传感器（PC4）
 *   - OLED   SSD1306 128x64 显示屏（I2C）
 *   - 蜂鸣器 PF0 高电平触发
 *   - 舵机   PF9 PWM 软件模拟
 *
 * Bug 修复清单（相比原版 main.c）：
 *   Fix-1  Send_MQTT_Process：一次性发送全部字段（原版 5 字段轮发，完整一轮 75s）
 *   Fix-2  Update_SensorData：去掉第二层 IIR 滤波（mq2.c 内已有一层，双重叠加导致严重滞后）
 *   Fix-3  MQTT_WARMUP_MS：从 120s 缩短至 30s，加快首次上报
 *   Fix-4  cnd：用明确的 u8 布尔标志 mqtt_send_pending 替代语义不清的 int cnd
 *
 * ThingCloud MQTT Topic：
 *   上报（PUB）: attributes
 *   下行（SUB）: data/stream/set
 */

#include "stm32f10x.h"
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
#include "beep.h"
#include "softiic.h"
#include <string.h>
#include <stdio.h>

/* ====================== 硬件引脚宏 ====================== */

/* 蜂鸣器（PF0，高电平响） */
#define BEEP_ON()           BEEP_State(1)
#define BEEP_OFF()          BEEP_State(0)

/* 板载 LED（PB5，低电平点亮） */
#define BOARD_LED_PORT      GPIOB
#define BOARD_LED_PIN       GPIO_Pin_5
#define BOARD_LED_ON()      GPIO_ResetBits(BOARD_LED_PORT, BOARD_LED_PIN)
#define BOARD_LED_OFF()     GPIO_SetBits(BOARD_LED_PORT, BOARD_LED_PIN)

/* 人体感应 LED（PE5，低电平点亮） */
#define HUMAN_LED_PORT      GPIOE
#define HUMAN_LED_PIN       GPIO_Pin_5
#define HUMAN_LED_RCC       RCC_APB2Periph_GPIOE
#define HUMAN_LED_ON()      GPIO_ResetBits(HUMAN_LED_PORT, HUMAN_LED_PIN)
#define HUMAN_LED_OFF()     GPIO_SetBits(HUMAN_LED_PORT, HUMAN_LED_PIN)

/* 按键（PE3=KEY1，PE4=KEY0，低电平有效；PA0=WKUP，高电平有效） */
#define KEY1_PORT           GPIOE
#define KEY1_PIN            GPIO_Pin_3
#define KEY0_PORT           GPIOE
#define KEY0_PIN            GPIO_Pin_4
#define K_KEY_PORT          GPIOA
#define K_KEY_PIN           GPIO_Pin_0
#define K_KEY_RCC           RCC_APB2Periph_GPIOA

/* 舵机（PF9，软件 PWM） */
#define SERVO_GPIO_PORT     GPIOF
#define SERVO_GPIO_PIN      GPIO_Pin_9
#define SERVO_GPIO_RCC      RCC_APB2Periph_GPIOF

/* ====================== 舵机参数 ====================== */
#define SERVO_ANGLE_MIN         500     /* 0° 脉宽（μs） */
#define SERVO_ANGLE_MAX         2500    /* 180° 脉宽（μs） */
#define SWEEP_STEP              40
#define SERVO_CONTINUOUS_MODE   1       /* 1=连续旋转模式, 0=180°来回扫描 */
#define SERVO_CW_PULSE          1620    /* 连续旋转顺时针脉宽（μs） */
#define SERVO_BOOST_PULSE       1620
#define SERVO_BOOST_MS          0U
#define SERVO_UPDATE_DIV        1U

/* ====================== 报警阈值默认值 ====================== */
#define TEMP_THRESHOLD_ON_DEFAULT    30     /* 温度开启阈值（°C） */
#define TEMP_THRESHOLD_OFF_DEFAULT   29     /* 温度关闭阈值（°C） */
#define HUM_ALARM_THRESHOLD_DEFAULT  80     /* 湿度报警阈值（%） */
#define MQ2_ALARM_THRESHOLD_DEFAULT  500    /* 气体报警阈值（ppm） */

/* MQ2 计算参数 */
#define MQ2_ADC_MAX             4095UL
#define MQ2_RL_OHM              5000UL      /* 负载电阻（Ω） */
#define MQ2_R0_OHM              10000UL     /* 基准电阻（Ω） */
#define MQ2_CLEAN_AIR_TARGET    200UL       /* 清洁空气基线 ppm */
#define MQ2_CALIB_SAMPLES       30U         /* 自校准采样次数 */
#define MQ2_REPORT_MAX          9999UL      /* 上报最大值 */

/* ====================== 任务周期（ms） ====================== */
#define LOOP_TICK_MS        20U
#define TASK_SENSOR_MS      500U
#define TASK_HUMAN_MS       100U
#define TASK_UI_MS          200U
#define TASK_MQTT_MS        15000U
#define MQTT_WARMUP_MS      30000U      /* Fix-3：从 120s 缩短至 30s */
#define HUMAN_LED_HOLD_MS   3000U
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
#define MQTT_SUB_TOPIC  "data/stream/set"

/* ====================== 静态变量 ====================== */
static u8  Pub_Topic[]   = MQTT_PUB_TOPIC;
static u8  Pub_Message[256];
static u8  check_char[256];

static u32 ppm              = 0;
static u8  human_status     = 0;
static u8  temp_threshold_on  = TEMP_THRESHOLD_ON_DEFAULT;
static u8  temp_threshold_off = TEMP_THRESHOLD_OFF_DEFAULT;
static u8  hum_alarm_threshold  = HUM_ALARM_THRESHOLD_DEFAULT;
static u32 mq2_alarm_threshold  = MQ2_ALARM_THRESHOLD_DEFAULT;

/* 舵机状态 */
static u8  servo_status       = 0;     /* 0=停止, 1=运行 */
static u8  servo_control_mode = 0;     /* 0=自动（温控）, 1=手动 */
static u16 servo_curr_pulse   = SERVO_ANGLE_MIN;
static u8  servo_sweep_dir    = 1;
static u32 servo_boost_until  = 0;

/* 传感器数据 */
static u8  dht11_data[5]         = {0};
static u16 mq2_raw               = 0;
static u32 mq2_baseline          = 0;
static unsigned long long mq2_cal_sum = 0;
static u8  mq2_cal_count         = 0;
static u8  mq2_cal_done          = 0;

/* 远程控制标志 */
static u8  remote_beep_active  = 0;
static u8  remote_led_active   = 0;

/* Fix-4：用明确的布尔标志替代原来的 int cnd */
static u8  mqtt_send_pending   = 0;
static u8  wifi_online         = 0;

/* 系统计时器（ms，由主循环累加） */
extern u32 system_timer;
static u32 last_dht_time           = 0;
static u32 last_human_check_time   = 0;
static u32 last_human_detect_time  = 0;
static u32 last_mqtt_time          = 0;
static u32 last_ui_time            = 0;
static u32 last_wifi_retry_time    = 0;

/* 按键状态 */
static u8  key1_latched    = 0;
static u8  key0_latched    = 0;
static u8  k_key_latched   = 0;
static u8  key_combo_latched = 0;
static u8  key_level_ready = 0;
static u8  key1_idle_level = 1;
static u8  key0_idle_level = 1;

/* OLED 显示模式：0=传感器数据, 1=阈值设置, 2=状态信息 */
static u8  oled_mode       = 0;
static u8  threshold_index = 0;
static u32 night_beep_until = 0;
static u8  human_raw_prev   = 0;
static u32 last_human_low_time = 0;

/* ====================== 内部函数前置声明 ====================== */
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
static void K_Key_Init(void);
static void Servo_Init(void);
static void Servo_OutputPulse(u16 pulse_us);
static void Servo_Control(u8 status, u8 mode);
static void Servo_UpdateSweep(void);
static void Send_MQTT_Process(void);
static void Update_SensorData(void);
static void Update_DisplayAndAlarm(void);
static void Handle_Keys(void);
static u8   JsonTryGetSwitch(const char *json, const char *key, u8 *out_value);
static void JsonNormalizeForMatch(char *buf);
static void Handle_WifiCommand(void);

/* ====================== 日志/串口 ====================== */

static void Log_Print(const char *msg)
{
    USART1_SendStr((char *)msg, strlen(msg));
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
        Log_Print("WiFi Bringup...\r\n");
        UART_Init();

        if (!cmdAT("AT\r\n", "OK", NULL, 4))
        {
            Log_Print("AT No Response\r\n");
            DELAY_Nms(300);
            continue;
        }

        WIFI_Init();

        if (TCP_Init() == 1)
        {
            Log_Print("TCP Connected\r\n");
            Log_Print("NET Ready -> MQTT/Downlink Enabled\r\n");
            mqtt_send_pending = 0;
            return 1;
        }

        Log_Print("TCP Retry...\r\n");
        DELAY_Nms(500);
    }

    Log_Print("TCP Connect Failed\r\n");
    mqtt_send_pending = 0;
    return 0;
}

/* ====================== 工具函数 ====================== */

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

static void K_Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(K_KEY_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = K_KEY_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(K_KEY_PORT, &GPIO_InitStructure);
}

static void Servo_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(SERVO_GPIO_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = SERVO_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SERVO_GPIO_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(SERVO_GPIO_PORT, SERVO_GPIO_PIN);
}

/* ====================== 舵机控制 ====================== */

static void Servo_OutputPulse(u16 pulse_us)
{
    GPIO_SetBits(SERVO_GPIO_PORT, SERVO_GPIO_PIN);
    DELAY_Nus(pulse_us);
    GPIO_ResetBits(SERVO_GPIO_PORT, SERVO_GPIO_PIN);
}

static void Servo_Control(u8 status, u8 mode)
{
    if (servo_status != status || servo_control_mode != mode)
    {
        Log_Print(mode ? "Servo: Manual Mode\r\n" : "Servo: Auto Mode\r\n");
        Log_Print(status ? "Action: SWEEP ON\r\n" : "Action: OFF\r\n");
    }

    servo_status       = status;
    servo_control_mode = mode;

    if (status == 1)
        servo_boost_until = system_timer + SERVO_BOOST_MS;
    else
        servo_boost_until = 0;
}

static void Servo_UpdateSweep(void)
{
    u16 target_pulse = servo_curr_pulse;
    static u8 servo_div_cnt = 0;

#if SERVO_CONTINUOUS_MODE
    if (servo_status == 1)
    {
        target_pulse = (system_timer < servo_boost_until) ? SERVO_BOOST_PULSE : SERVO_CW_PULSE;
    }
    else
    {
        servo_div_cnt = 0;
        GPIO_ResetBits(SERVO_GPIO_PORT, SERVO_GPIO_PIN);
        return;
    }
#else
    if (servo_status == 1)
    {
        if (servo_sweep_dir == 1)
        {
            servo_curr_pulse += SWEEP_STEP;
            if (servo_curr_pulse >= SERVO_ANGLE_MAX)
            {
                servo_curr_pulse = SERVO_ANGLE_MAX;
                servo_sweep_dir  = 0;
            }
        }
        else
        {
            if (servo_curr_pulse > (SERVO_ANGLE_MIN + SWEEP_STEP))
                servo_curr_pulse -= SWEEP_STEP;
            else
            {
                servo_curr_pulse = SERVO_ANGLE_MIN;
                servo_sweep_dir  = 1;
            }
        }
        target_pulse = servo_curr_pulse;
    }
    else
    {
        if (servo_curr_pulse != SERVO_ANGLE_MIN)
            servo_curr_pulse = SERVO_ANGLE_MIN;
        target_pulse = SERVO_ANGLE_MIN;
    }
#endif

    servo_div_cnt++;
    if (servo_div_cnt < SERVO_UPDATE_DIV)
        return;
    servo_div_cnt = 0;

    Servo_OutputPulse(target_pulse);
}

/* ====================== Fix-1：MQTT 发送（单条 JSON 包含全部字段）====================== */
/**
 * 原版每次调用仅发送 1 个字段（5 字段轮转），完整一轮需 5×15s=75s，
 * 导致云端同一时刻的 TEMP/HUM/MQ2/HUMAN/SERVO 来自不同时刻的采样值。
 *
 * 修复后：每次触发将所有字段打包为一条 JSON，确保云端数据时序一致。
 */
static void Send_MQTT_Process(void)
{
    int  pkt_len;
    char at_buf[32];

    if (!mqtt_send_pending)
        return;

    /* 构建完整 JSON（所有字段一次发出）*/
    snprintf((char *)Pub_Message, sizeof(Pub_Message),
             "{\"TEMP\":%d,\"HUM\":%d,\"MQ2\":%lu,"
             "\"HUMAN\":%s}",
             dht11_data[2],
             dht11_data[0],
             (unsigned long)ppm,
             human_status ? "true" : "false",
             //servo_status  ? "true" : "false");

    Log_Print("PUB Topic:");
    Log_Print((char *)Pub_Topic);
    Log_Print(" | ");
    Log_Print((char *)Pub_Message);
    Log_Print("\r\n");

    SerialBridge_Print((char *)Pub_Message);

    /* 编码 MQTT PUBLISH 报文 */
    pkt_len = MqttPublishData((char *)Pub_Topic,
                              (char *)Pub_Message,
                              strlen((char *)Pub_Message));
    if (pkt_len <= 0)
    {
        Log_Print("MQTT Encode Fail\r\n");
        mqtt_send_pending = 0;
        return;
    }

    /* 发送 AT 指令 */
    snprintf(at_buf, sizeof(at_buf), "AT+CIPSEND=%d\r\n", pkt_len);
    if (cmdAT(at_buf, "OK", ">", strlen(at_buf)))
    {
        if (Wifi_SendRaw(MQTT_SEND_RealtimeData, pkt_len, "SEND OK", NULL))
            Log_Print("MQTT Pub OK\r\n");
        else
            Log_Print("MQTT Pub Fail\r\n");
    }
    else
    {
        Log_Print("AT+CIPSEND No Response\r\n");
    }

    mqtt_send_pending = 0;  /* 清标志，等待下一次定时触发 */
}

/* ====================== Fix-2：传感器采集（去掉第二层 IIR）====================== */
/**
 * 原版在 mq2.c 的 MQ2_GetData_PPM() 中已有一层 IIR（α=0.8/0.2），
 * 此处又叠加了第二层（α=0.75/0.25），双重滤波导致气体浓度变化
 * 需数分钟才能反映到上报值，严重滞后。
 *
 * 修复后：去掉本层 IIR，直接使用 mq2.c 输出的已滤波值。
 */
static void Update_SensorData(void)
{
    u32 mq2_raw_ppm;
    u32 mq2_corrected;
    u8  i;

    /* 读取 DHT11（最多重试 3 次）*/
    for (i = 0; i < 3; i++)
    {
        if (DHT11_Getdata() == 0)
        {
            dht11_data[0] = data[0];   /* 湿度整数 */
            dht11_data[2] = data[2];   /* 温度整数 */
            break;
        }
        DELAY_Nms(20);
    }

    /* 读取 MQ2（mq2.c 内已完成一层 IIR 低通）*/
    mq2_raw     = MQ2_GetData();
    mq2_raw_ppm = (u32)MQ2_GetData_PPM();

    /* 溢出保护 */
    if (mq2_raw_ppm > 1000000UL)
        mq2_raw_ppm = (u32)mq2_raw * 100UL;

    /* 自校准基线（开机前 MQ2_CALIB_SAMPLES 次采样求均值）*/
    if (!mq2_cal_done)
    {
        mq2_cal_sum += mq2_raw_ppm;
        mq2_cal_count++;
        if (mq2_cal_count >= MQ2_CALIB_SAMPLES)
        {
            u32 avg  = (u32)(mq2_cal_sum / mq2_cal_count);
            mq2_baseline = (avg > MQ2_CLEAN_AIR_TARGET)
                           ? (avg - MQ2_CLEAN_AIR_TARGET) : 0;
            mq2_cal_done = 1;
            Log_Print("MQ2 Baseline OK\r\n");
        }
    }

    /* 基线修正 + 上限截断 */
    mq2_corrected = (mq2_raw_ppm > mq2_baseline)
                    ? (mq2_raw_ppm - mq2_baseline) : 0;
    if (mq2_corrected > MQ2_REPORT_MAX)
        mq2_corrected = MQ2_REPORT_MAX;

    /* Fix-2：直接赋值，不再叠加第二层 IIR */
    ppm = mq2_corrected;
}

/* ====================== OLED 显示 + 蜂鸣器报警 ====================== */

static void Update_DisplayAndAlarm(void)
{
    char buf[20];
    char time_str[12];
    static u8 prev_oled_mode = 0xFF;

    u8 night_human_alarm = (system_timer < night_beep_until) ? 1 : 0;
    u8 alarm_trigger = (dht11_data[2] >= temp_threshold_on  ||
                        dht11_data[0] >  hum_alarm_threshold ||
                        ppm           >  mq2_alarm_threshold ||
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

    /* 模式 0：传感器实时数据 */
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
    /* 模式 1：阈值设置 */
    else if (oled_mode == 1)
    {
        snprintf(buf, sizeof(buf), "TEMP:%d   ", temp_threshold_on);
        OLED_ShowString(0, 16, (u8 *)buf);

        snprintf(buf, sizeof(buf), "HUM :%d   ", hum_alarm_threshold);
        OLED_ShowString(2, 16, (u8 *)buf);

        snprintf(buf, sizeof(buf), "MQ2 :%lu  ", (unsigned long)mq2_alarm_threshold);
        OLED_ShowString(4, 16, (u8 *)buf);

        /* 光标指示 */
        OLED_ShowString(0, 0, (u8 *)(threshold_index == 0 ? ">" : " "));
        OLED_ShowString(2, 0, (u8 *)(threshold_index == 1 ? ">" : " "));
        OLED_ShowString(4, 0, (u8 *)(threshold_index == 2 ? ">" : " "));
    }
    /* 模式 2：执行器状态 */
    else if (oled_mode == 2)
    {
        sprintf(buf, "SERVO:%s", servo_status ? "ON"     : "OFF");
        OLED_ShowString(0, 0, (u8 *)buf);

        sprintf(buf, "HUMAN:%s", human_status ? "YES"   : "NO");
        OLED_ShowString(2, 0, (u8 *)buf);

        sprintf(buf, "MODE :%s", servo_control_mode ? "MANUAL" : "AUTO");
        OLED_ShowString(4, 0, (u8 *)buf);

        OLED_ShowString(6, 0, (u8 *)"                ");
    }
}

/* ====================== 按键处理 ====================== */

static void Handle_Keys(void)
{
    u8 key1_now = GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN);
    u8 key0_now = GPIO_ReadInputDataBit(KEY0_PORT, KEY0_PIN);

    u8 key1_pressed = (key1_now == Bit_RESET);  /* 低电平有效 */
    u8 key0_pressed = (key0_now == Bit_RESET);

    if (!key_level_ready)
        key_level_ready = 1;

    /* KEY0 + KEY1 同时按下：切换 OLED 页面 */
    if (key1_pressed && key0_pressed)
    {
        if (!key_combo_latched)
        {
            DELAY_Nms(KEY_DEBOUNCE_MS);
            if ((GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN) != key1_idle_level) &&
                (GPIO_ReadInputDataBit(KEY0_PORT, KEY0_PIN) != key0_idle_level))
            {
                if (oled_mode == 0)
                {
                    oled_mode = 1;
                    threshold_index = 0;
                }
                else if (oled_mode == 1)
                {
                    threshold_index++;
                    if (threshold_index > 2)
                    {
                        threshold_index = 0;
                        oled_mode = 2;
                    }
                }
                else
                {
                    oled_mode = 0;
                }
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

    /* KEY1：值增加 / 舵机开 */
    if (key1_pressed && !key1_latched)
    {
        DELAY_Nms(KEY_DEBOUNCE_MS);
        if (oled_mode == 1)
        {
            if (threshold_index == 0 && temp_threshold_on < 60)
                temp_threshold_on++;
            else if (threshold_index == 1 && hum_alarm_threshold < 99)
                hum_alarm_threshold++;
            else if (threshold_index == 2 && mq2_alarm_threshold < 10000)
                mq2_alarm_threshold += 50;
        }
        else if (oled_mode == 2)
        {
            Servo_Control(1, 1);
        }
        key1_latched = 1;
    }
    else if (!key1_pressed)
        key1_latched = 0;

    /* KEY0：值减少 / 舵机关 */
    if (key0_pressed && !key0_latched)
    {
        DELAY_Nms(KEY_DEBOUNCE_MS);
        if (oled_mode == 1)
        {
            if (threshold_index == 0 && temp_threshold_on > 5)
                temp_threshold_on--;
            else if (threshold_index == 1 && hum_alarm_threshold > 20)
                hum_alarm_threshold--;
            else if (threshold_index == 2 && mq2_alarm_threshold > 100)
                mq2_alarm_threshold -= 50;
        }
        else if (oled_mode == 2)
        {
            Servo_Control(0, 1);
        }
        key0_latched = 1;
    }
    else if (!key0_pressed)
        key0_latched = 0;

    /* K_KEY（WKUP）：高电平触发，切换 OLED 页面 */
    if (GPIO_ReadInputDataBit(K_KEY_PORT, K_KEY_PIN))
    {
        if (!k_key_latched)
        {
            DELAY_Nms(KEY_DEBOUNCE_MS);
            if (GPIO_ReadInputDataBit(K_KEY_PORT, K_KEY_PIN))
            {
                if (oled_mode == 0)
                {
                    oled_mode = 1;
                    threshold_index = 0;
                }
                else if (oled_mode == 1)
                {
                    threshold_index++;
                    if (threshold_index > 2)
                    {
                        threshold_index = 0;
                        oled_mode = 2;
                    }
                }
                else
                {
                    oled_mode = 0;
                }
                k_key_latched = 1;
            }
        }
    }
    else
    {
        k_key_latched = 0;
    }
}

/* ====================== ThingCloud 下行指令解析 ====================== */

/**
 * @brief  从 JSON 字符串中解析布尔型开关值
 * @retval 1=找到并解析成功, 0=未找到
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
 * @brief  去除 JSON 字符串中的空白符和转义符，方便 strstr 匹配
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

/**
 * @brief  处理 Wi-Fi 串口下行数据（ThingCloud MQTT 下行指令）
 *
 * 支持的控制字段（JSON 键名）：
 *   BEEP   : 0/1  蜂鸣器远程控制
 *   LED    : 0/1  板载 LED 远程控制
 *   SERVO  : 0/1  舵机远程控制
 */
static void Handle_WifiCommand(void)
{
    u16 copy_len;
    u8  cmd_val;

    if (wifiRecvOver != 1)
        return;

    copy_len = (recvCnt < sizeof(check_char) - 1U)
               ? recvCnt : (sizeof(check_char) - 1U);
    memcpy(check_char, (const void *)wifiRecvBuf, copy_len);
    check_char[copy_len] = '\0';
    JsonNormalizeForMatch((char *)check_char);

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
    if (JsonTryGetSwitch((char *)check_char, "SERVO", &cmd_val))
        Servo_Control(cmd_val, 1);

    wifiRecvOver = 0;
    recvCnt      = 0;
}

/* ====================== 主函数 ====================== */

int main(void)
{
    /* ---------- 中断优先级分组（必须最先设置）---------- */
    NVIC_SetPriorityGrouping(2);

    /* ---------- 外设初始化（与原工程顺序一致）---------- */
    LED_Config();
    KEY_Config();
    USART1_Config(115200);          /* 注意：工程中用 USART1_Config，非 USART1_Init */
    DELAY_Nms(100);

    Log_Print("\r\n\r\n----BOOT----\r\n");
    Log_ResetReason();

    SoftIIC_Config();
    OLED_Config();
    OLED_Clear(0);

    /* OLED 固定标签 */
    OLED_ShowString(0, 0, (u8 *)"TIME:");
    OLED_ShowString(2, 0, (u8 *)"TEMP:");
    OLED_ShowString(4, 0, (u8 *)"HUM :");
    OLED_ShowString(6, 0, (u8 *)"MQ2 :");

    BOARD_LED_OFF();
    BEEP_Init_Safe();

    DHT11_Config();                 /* 注意：工程中用 DHT11_Config，非 DHT11_Init */
    ADCx_Init();                    /* MQ2 依赖 ADC，需在 MQ2_Init 之前调用 */
    MQ2_Init();
    SR602_Init();
    Servo_Init();
    Human_LED_Init();
    K_Key_Init();

    /* 首次采样，避免开机时显示全 0 */
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

        /* --- 传感器采集（500ms）--- */
        if (IsTimeDue(system_timer, &last_dht_time, TASK_SENSOR_MS))
            Update_SensorData();

        /* --- 人体感应检测（100ms）--- */
        if (IsTimeDue(system_timer, &last_human_check_time, TASK_HUMAN_MS))
        {
            u8 human_raw = SR602_Detect() ? 1 : 0;
            human_status = human_raw;

            if (!human_raw)
            {
                last_human_low_time = system_timer;
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

        /* 人体感应 LED 保持亮起（检测后保持 HUMAN_LED_HOLD_MS）*/
        if ((system_timer - last_human_detect_time) < HUMAN_LED_HOLD_MS)
            HUMAN_LED_ON();
        else
            HUMAN_LED_OFF();

        /* --- 温控舵机（自动模式）--- */
        if (servo_control_mode == 0)
        {
            if (dht11_data[2] >= temp_threshold_on && servo_status == 0)
                Servo_Control(1, 0);
            else if (dht11_data[2] <= temp_threshold_off && servo_status == 1)
                Servo_Control(0, 0);
        }

        Servo_UpdateSweep();

        /* --- OLED 刷新 + 蜂鸣器（200ms）--- */
        if (IsTimeDue(system_timer, &last_ui_time, TASK_UI_MS))
            Update_DisplayAndAlarm();

        /* --- 按键扫描 --- */
        Handle_Keys();

        /* --- MQTT 上报（Fix-3：暖机 30s 后，每 15s 触发一次）--- */
        if (wifi_online &&
            system_timer >= MQTT_WARMUP_MS &&
            IsTimeDue(system_timer, &last_mqtt_time, TASK_MQTT_MS))
        {
            /* Fix-4：使用布尔标志，而非原版的 int cnd */
            if (!mqtt_send_pending)
                mqtt_send_pending = 1;
        }

        /* --- Wi-Fi 任务 --- */
        if (wifi_online)
        {
            Handle_WifiCommand();
            Send_MQTT_Process();
        }
        else
        {
#if WIFI_AUTO_BRINGUP
            if (IsTimeDue(system_timer, &last_wifi_retry_time, WIFI_RETRY_MS))
                wifi_online = Wifi_Bringup();
#endif
        }
    }
}
