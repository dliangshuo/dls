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
#include "softiic.h"
#include <string.h>
#include <stdio.h>

// ========================== 宏定义配置 ==========================
// 串口接线提示:
// 1) TX/RX 交叉连接
// 2) GND 必须共地，否则电平没有统一参考，串口数据会不稳定或完全无法识别
// 关键端口速查:
// - 蜂鸣器: GPIOB_8
// - 人体指示LED: GPIOE_5
// - 按键: KEY1=GPIOE_3, KEY0=GPIOE_4
// - 执行器(舵机信号): GPIOF_9

// --- 蜂鸣器 (GPIOB_8) ---
#define BEEP_PORT        GPIOB
#define BEEP_PIN         GPIO_Pin_8
#define BEEP_RCC         RCC_APB2Periph_GPIOB
#define BEEP_ON()        GPIO_ResetBits(BEEP_PORT, BEEP_PIN)
#define BEEP_OFF()       GPIO_SetBits(BEEP_PORT, BEEP_PIN)

// --- 板载LED (GPIOB_5, 低电平点亮) ---
#define BOARD_LED_PORT   GPIOB
#define BOARD_LED_PIN    GPIO_Pin_5
#define BOARD_LED_ON()   GPIO_ResetBits(BOARD_LED_PORT, BOARD_LED_PIN)
#define BOARD_LED_OFF()  GPIO_SetBits(BOARD_LED_PORT, BOARD_LED_PIN)

// --- 人体检测LED (GPIOE_5) ---
#define HUMAN_LED_PORT   GPIOE
#define HUMAN_LED_PIN    GPIO_Pin_5
#define HUMAN_LED_RCC    RCC_APB2Periph_GPIOE
#define HUMAN_LED_ON()   GPIO_ResetBits(HUMAN_LED_PORT, HUMAN_LED_PIN)
#define HUMAN_LED_OFF()  GPIO_SetBits(HUMAN_LED_PORT, HUMAN_LED_PIN)
// SR602(PIR) 工作特性简述:
// 1) 被动红外感知人体移动引起的红外变化，不是测距模块
// 2) 有人体运动时输出有效电平(具体高/低由模块板决定，SR602_Detect 内已封装)
// 3) 静止不动时可能恢复为无触发，因此代码做了“保持亮灯(HUMAN_LED_HOLD_MS)”防闪烁

// --- 本地按键 (GPIOE_3 / GPIOE_4, 低电平按下) ---
#define KEY1_PORT         GPIOE
#define KEY1_PIN          GPIO_Pin_3   // 手动开启扫风
#define KEY0_PORT         GPIOE
#define KEY0_PIN          GPIO_Pin_4   // 手动关闭扫风
#define K_KEY_PORT        GPIOA
#define K_KEY_PIN         GPIO_Pin_0   // 显示切换按键(低电平按下)
#define K_KEY_RCC         RCC_APB2Periph_GPIOA

// --- 180度舵机配置 (PF9) ---
#define SERVO_GPIO_PORT    GPIOF
#define SERVO_GPIO_PIN     GPIO_Pin_9
#define SERVO_GPIO_RCC     RCC_APB2Periph_GPIOF

#define SERVO_ANGLE_MIN    500    // 0度
#define SERVO_ANGLE_MAX    2500   // 180度
#define SWEEP_STEP         40
#define SERVO_CONTINUOUS_MODE 1   // 1=连续旋转舵机模式, 0=180度位置舵机模式
#define SERVO_CW_PULSE      1650  // 连续旋转舵机顺时针(降低堵转/电流冲击)
#define SERVO_BOOST_PULSE   1800  // 启动增强脉宽(温和启动)
#define SERVO_BOOST_MS      120U  // 启动增强时间(缩短尖峰电流持续时间)
#define SERVO_UPDATE_DIV    3U    // 舵机脉冲降频系数: 3 表示约 60ms 输出一次脉冲

// --- 阈值设置 ---
#define TEMP_THRESHOLD_ON_DEFAULT    30
#define TEMP_THRESHOLD_OFF_DEFAULT   29
#define HUM_ALARM_THRESHOLD_DEFAULT  80
#define MQ2_ALARM_THRESHOLD_DEFAULT  500
// MQ2标定参数（用于估算真实ppm前的中间量）
#define MQ2_ADC_MAX          4095UL
#define MQ2_RL_OHM           5000UL   // 模块负载电阻(常见5K)
#define MQ2_R0_OHM           10000UL  // 需在洁净空气中标定后替换
#define MQ2_CLEAN_AIR_TARGET 200UL    // 期望洁净空气基线(<200ppm)
#define MQ2_CALIB_SAMPLES    30U      // 启动后采样次数（2s周期约1分钟）

// --- 周期任务时间片(ms) ---
#define LOOP_TICK_MS      20U
#define TASK_SENSOR_MS    2000U
#define TASK_HUMAN_MS     100U
#define TASK_UI_MS        200U
#define TASK_MQTT_MS      10000U
#define HUMAN_LED_HOLD_MS 10000U
#define HUMAN_RETRIGGER_MS 2000U
#define KEY_DEBOUNCE_MS   10U
#define WIFI_BOOT_DELAY_MS 1500U
#define WIFI_INIT_RETRY    3U
#define WIFI_RETRY_MS      30000U
#define WIFI_AUTO_BRINGUP  1U   // 1: 开启自动重连，云平台可持续在线
#define NIGHT_START_HOUR   0U   // 夜间开始(24点)
#define NIGHT_END_HOUR     7U   // 夜间结束(7点，不含7点)
#define CLOCK_START_HOUR   0U   // 上电时刻(小时)，无RTC时可手动设置
#define CLOCK_START_MIN    0U   // 上电时刻(分钟)
#define CLOCK_START_SEC    0U   // 上电时刻(秒)
#define NIGHT_BEEP_MS      3000U
#define USE_RTC_CLOCK      0U   // 1=使用RTC_GetCounter作为时间源, 0=使用system_timer软件时钟
#define SERIAL_PC_BRIDGE    1U   // 1=串口输出JSON供PC软件转发到云平台

// ========================== 全局变量 ==========================
// MQTT联调提示:
// 使用 MQTT.fx 等工具时，不要与设备使用相同 ClientID/设备身份，
// 否则 broker 会挤掉旧连接，出现“周期性断开重连”。
// 设备正常上云不依赖额外PC软件，烧录本固件并配置好 WiFi/云端参数即可自动上报。
// ThingCloud 混合模式(本项目约定):
// 1) 上报(发布) Topic 使用云平台默认 attributes
// 2) 下发(订阅) Topic 使用自定义数据流 Topic(默认 data/stream/set)
// 3) 走标准 MQTT PUBLISH 封包，消息体为 JSON 文本
// 云平台最小配置清单:
// A. 设备鉴权(三元组/Token/ClientID/用户名密码)与固件一致
// B. MQTT服务器地址和端口与固件一致(示例: bj-2-mqtt.iot-api.com:1883)
// C. 发布Topic=MQTT_PUB_TOPIC，订阅Topic按平台下发规则配置
// D. 解析规则与消息格式匹配(JSON字段: TIME/TEMP/HUM/MQ2/HUMAN/SERVO/BEEP/LED)
#define MQTT_PUB_TOPIC "attributes"
#define MQTT_SUB_TOPIC "data/stream/set"
// 上云流程说明:
// 1) WiFi/TCP 连通后，主循环按 TASK_MQTT_MS 周期触发 Send_MQTT_Process()
// 2) 设备会自动向 MQTT_PUB_TOPIC 上报数据(无需手工点击)
// 3) 云平台需配置: 设备身份/鉴权、Topic(发布/订阅)、JSON字段解析规则
static u8 Pub_Topic[] = MQTT_PUB_TOPIC;
static u8 Pub_Message[192];
static u8 check_char[256];

static u32 ppm = 0;
static u8 human_status = 0;
static u8 temp_threshold_on = TEMP_THRESHOLD_ON_DEFAULT;
static u8 temp_threshold_off = TEMP_THRESHOLD_OFF_DEFAULT;
static u8 hum_alarm_threshold = HUM_ALARM_THRESHOLD_DEFAULT;
static u32 mq2_alarm_threshold = MQ2_ALARM_THRESHOLD_DEFAULT;

// 舵机与控制状态
static u8 servo_status = 0;       // 0-关闭, 1-开启(扫风)
static u8 servo_control_mode = 0; // 0-自动模式, 1-手动模式

// 扫风控制变量
static u16 servo_curr_pulse = SERVO_ANGLE_MIN;
static u8 servo_sweep_dir = 1; // 1=向180度转, 0=向0度转
static u32 servo_boost_until = 0;

// 传感器数据
static u8 dht11_data[5] = {0};
static u16 mq2_raw = 0;
static u32 mq2_baseline = 0;
static unsigned long long mq2_cal_sum = 0;
static u8 mq2_cal_count = 0;
static u8 mq2_cal_done = 0;

// 远程控制标志位
static u8 remote_beep_active = 0;
static u8 remote_led_active = 0;

static int cnd = 0;
static int len = 0;
static u8 wifi_online = 0;

// 时间管理 (单位: ms)
// 说明:
// - system_timer 不是RTC实时时钟，而是软件累加计时
// - 主循环每次 DELAY_Nms(LOOP_TICK_MS) 后执行 system_timer += LOOP_TICK_MS
// - 所有任务调度和夜间判断都基于这个“上电后经过时间”
extern u32 system_timer;
static u32 last_dht_time = 0;
static u32 last_human_check_time = 0;
static u32 last_human_detect_time = 0;
static u32 last_mqtt_time = 0;
static u32 last_ui_time = 0;
static u32 last_wifi_retry_time = 0;
static u8 key1_latched = 0;
static u8 key0_latched = 0;
static u8 k_key_latched = 0;
static u8 key_level_ready = 0;
static u8 key1_idle_level = 1;
static u8 key0_idle_level = 1;
// ===== 新UI系统 =====
// 0=数据页 1=阈值页 2=控制页
static u8 oled_mode = 0;
//static u8 oled_status_view = 0;  // 0=状态页 1=温度阈值 2=湿度阈值 3=MQ2阈值

// 阈值选择项（0温度 1湿度 2MQ2）
static u8 threshold_index = 0;
static u32 night_beep_until = 0;
static u8 human_raw_prev = 0;
static u32 last_human_low_time = 0;

// ========================== 函数定义 ==========================
static void Log_Print(const char *msg)
{
    USART1_SendStr((char *)msg, strlen(msg));
}

static void SerialBridge_Print(const char *json)
{
#if SERIAL_PC_BRIDGE
    Log_Print("BRIDGE:");
    Log_Print(json);
    Log_Print("\r\n");
#endif
}

static u8 Wifi_Bringup(void)
{
    u8 i;

    // 独立供电的 Wi-Fi 模块上电后通常需要更长启动时间
    DELAY_Nms(WIFI_BOOT_DELAY_MS);

    for (i = 0; i < WIFI_INIT_RETRY; i++)
    {
        Log_Print("WiFi Bringup...\r\n");
        UART_Init();

        // 先探测AT链路，便于定位“串口不通/模块未上电”问题
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
            cnd = 0;
            return 1;
        }

        Log_Print("TCP Retry...\r\n");
        DELAY_Nms(500);
    }

    Log_Print("TCP Connect Failed\r\n");
    cnd = 0;
    return 0;
}

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
    // 依赖外部RTC初始化: RTC_Config()/备份域时钟配置
    return (u32)(RTC_GetCounter() % 86400UL);
#else
    u32 start_sec = CLOCK_START_HOUR * 3600UL + CLOCK_START_MIN * 60UL + CLOCK_START_SEC;
    return ((system_timer / 1000UL) + start_sec) % 86400UL;
#endif
}

static void FormatClockString(char *out, u16 out_len)
{
    u32 sec_of_day = GetSecondsOfDay();
    u8 hh = (u8)(sec_of_day / 3600UL);
    u8 mm = (u8)((sec_of_day % 3600UL) / 60UL);
    u8 ss = (u8)(sec_of_day % 60UL);
    snprintf(out, out_len, "%02d:%02d:%02d", hh, mm, ss);
}

static u8 IsNightPeriod(void)
{
    u32 sec_of_day = GetSecondsOfDay();
    u8 hour = (u8)(sec_of_day / 3600UL);

    if (NIGHT_START_HOUR < NIGHT_END_HOUR)
        return (hour >= NIGHT_START_HOUR && hour < NIGHT_END_HOUR);
    return (hour >= NIGHT_START_HOUR || hour < NIGHT_END_HOUR);
}

static u32 MQ2_CalcRsR0_x1000(u16 adc_raw)
{
    u32 num;
    u32 den;

    if (adc_raw == 0 || adc_raw >= MQ2_ADC_MAX)
        return 0;

    // Rs/R0 = (RL/R0) * (ADCmax-ADC)/ADC
    num = MQ2_RL_OHM * (MQ2_ADC_MAX - (u32)adc_raw) * 1000UL;
    den = (u32)adc_raw * MQ2_R0_OHM;
    return (den == 0) ? 0 : (num / den);
}

static void Human_LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(HUMAN_LED_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = HUMAN_LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HUMAN_LED_PORT, &GPIO_InitStructure);
    HUMAN_LED_OFF();
}

static void BEEP_Init_Safe(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(BEEP_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = BEEP_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BEEP_PORT, &GPIO_InitStructure);
    BEEP_OFF();
}

static void K_Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(K_KEY_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = K_KEY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(K_KEY_PORT, &GPIO_InitStructure);
}

static void Servo_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(SERVO_GPIO_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = SERVO_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SERVO_GPIO_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(SERVO_GPIO_PORT, SERVO_GPIO_PIN);
}

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

    servo_status = status;
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
        if (system_timer < servo_boost_until)
            target_pulse = SERVO_BOOST_PULSE;
        else
            target_pulse = SERVO_CW_PULSE; // 按需求固定顺时针
    }
    else
    {
        // 关闭状态不再输出脉冲，避免不同舵机中位偏差导致“慢速自转”
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
                servo_sweep_dir = 0;
            }
        }
        else
        {
            if (servo_curr_pulse > (SERVO_ANGLE_MIN + SWEEP_STEP))
                servo_curr_pulse -= SWEEP_STEP;
            else
            {
                servo_curr_pulse = SERVO_ANGLE_MIN;
                servo_sweep_dir = 1;
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

    // 软PWM降频：降低CPU阻塞时间与舵机平均电流，减少串口/网络被拖慢的概率
    servo_div_cnt++;
    if (servo_div_cnt < SERVO_UPDATE_DIV)
        return;
    servo_div_cnt = 0;

    Servo_OutputPulse(target_pulse);
}

static void Send_MQTT_Process(void)
{
    switch (cnd)
    {
    case 0:
        break;
    case 1:
    {
        cnd = 0; // 先清状态，避免AT等待期间反复触发超时状态
        char time_str[12];
        FormatClockString(time_str, sizeof(time_str));
        // 云平台字段类型建议:
        // TIME -> String(字符串, 例如 "08:12:30")
        // TEMP/HUM/MQ2 -> Number
        // HUMAN/SERVO/BEEP/LED -> Bool(布尔)
        snprintf((char *)Pub_Message, sizeof(Pub_Message),
                 "{\"TIME\":\"%s\",\"TEMP\":%d,\"HUM\":%d,\"MQ2\":%lu,\"HUMAN\":%s,\"SERVO\":%s,\"BEEP\":%s,\"LED\":%s}",
                 time_str,
                 dht11_data[2], dht11_data[0], (unsigned long)ppm,
                 human_status ? "true" : "false",
                 servo_status ? "true" : "false",
                 remote_beep_active ? "true" : "false",
                 remote_led_active ? "true" : "false");
        Log_Print("PUB Topic:");
        Log_Print((char *)Pub_Topic);
        Log_Print("\r\n");
        SerialBridge_Print((char *)Pub_Message);
        len = MqttPublishData((char *)Pub_Topic, (char *)Pub_Message, strlen((char *)Pub_Message));
        if (len <= 0)
        {
            Log_Print("MQTT Encode Fail\r\n");
            cnd = 0;
            break;
        }
        {
            char at_buf[30] = {0};
            snprintf(at_buf, sizeof(at_buf), "AT+CIPSEND=%d\r\n", len);
            if (cmdAT(at_buf, "OK", ">", strlen(at_buf)))
            {
                if (cmdAT((char *)MQTT_SEND_RealtimeData, "SEND OK", NULL, len))
                    Log_Print("MQTT Pub Success\r\n");
                else
                    Log_Print("MQTT Pub Fail\r\n");
            }
        }
        break;
    }
    default:
        cnd = 0;
        break;
    }
}

static void Update_SensorData(void)
{
    u32 mq2_raw_ppm;

    if (DHT11_Getdata() == 0)
    {
        dht11_data[0] = data[0];
        dht11_data[2] = data[2];
    }

    mq2_raw = MQ2_GetData();
    mq2_raw_ppm = (u32)MQ2_GetData_PPM();
    if (mq2_raw_ppm > 1000000UL)
    {
        // 异常值保护：部分库在边界条件下会返回不合理大值
        mq2_raw_ppm = (u32)mq2_raw * 100UL;
    }

    // 启动阶段在洁净空气中自动建立基线，降低系统性偏高
    if (!mq2_cal_done)
    {
        mq2_cal_sum += mq2_raw_ppm;
        mq2_cal_count++;
        if (mq2_cal_count >= MQ2_CALIB_SAMPLES)
        {
            u32 avg = (u32)(mq2_cal_sum / mq2_cal_count);
            mq2_baseline = (avg > MQ2_CLEAN_AIR_TARGET) ? (avg - MQ2_CLEAN_AIR_TARGET) : 0;
            mq2_cal_done = 1;
            Log_Print("MQ2 Baseline Ready\r\n");
        }
    }

    ppm = (mq2_raw_ppm > mq2_baseline) ? (mq2_raw_ppm - mq2_baseline) : 0;
    if (ppm > 999999UL) ppm = 999999UL;
}

static void Update_DisplayAndAlarm(void)
{
    char buf[20];
    char time_str[12];

    // ===== 报警逻辑（保留原逻辑）=====
    u8 night_human_alarm = (system_timer < night_beep_until) ? 1 : 0;
    u8 alarm_trigger = (dht11_data[2] > temp_threshold_on ||
                        dht11_data[0] > hum_alarm_threshold ||
                        ppm > mq2_alarm_threshold ||
                        night_human_alarm);

    if (alarm_trigger || remote_beep_active)
        BEEP_ON();
    else
        BEEP_OFF();

    OLED_Clear(0);
    FormatClockString(time_str, sizeof(time_str));

    // ================= 模式1：数据页 =================
    if (oled_mode == 0)
    {
        OLED_ShowString(0, 0, (u8 *)"TIME:");
        OLED_ShowString(0, 48, (u8 *)time_str);

        sprintf(buf, "TEMP:%dC", dht11_data[2]);
        OLED_ShowString(2, 0, (u8 *)buf);

        sprintf(buf, "HUM :%d%%", dht11_data[0]);
        OLED_ShowString(4, 0, (u8 *)buf);

        sprintf(buf, "MQ2 :%lu", ppm);
        OLED_ShowString(6, 0, (u8 *)buf);
    }

    // ================= 模式2：阈值页 =================
    else if (oled_mode == 1)
    {
        sprintf(buf, "TEMP:%d", temp_threshold_on);
        OLED_ShowString(0, 16, (u8 *)buf);

        sprintf(buf, "HUM :%d", hum_alarm_threshold);
        OLED_ShowString(2, 16, (u8 *)buf);

        sprintf(buf, "MQ2 :%lu", mq2_alarm_threshold);
        OLED_ShowString(4, 16, (u8 *)buf);

        // 箭头指示当前选择
        OLED_ShowString(0, 0, (u8 *)(threshold_index == 0 ? ">" : " "));
        OLED_ShowString(2, 0, (u8 *)(threshold_index == 1 ? ">" : " "));
        OLED_ShowString(4, 0, (u8 *)(threshold_index == 2 ? ">" : " "));
    }

    // ================= 模式3：控制页 =================
    else if (oled_mode == 2)
    {
        sprintf(buf, "SERVO:%s", servo_status ? "ON" : "OFF");
        OLED_ShowString(0, 0, (u8 *)buf);

        sprintf(buf, "HUMAN:%s", human_status ? "YES" : "NO");
        OLED_ShowString(2, 0, (u8 *)buf);

        sprintf(buf, "MODE :%s", servo_control_mode ? "MANUAL" : "AUTO");
        OLED_ShowString(4, 0, (u8 *)buf);

        OLED_ShowString(6, 0, (u8 *)"KEY1:ON KEY0:OFF");
    }
}

static void Handle_Keys(void)
{
    u8 key1_now = GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN);
    u8 key0_now = GPIO_ReadInputDataBit(KEY0_PORT, KEY0_PIN);

    u8 key1_pressed;
    u8 key0_pressed;

    if (!key_level_ready)
    {
        key1_idle_level = key1_now;
        key0_idle_level = key0_now;
        key_level_ready = 1;
        return;
    }

    key1_pressed = (key1_now != key1_idle_level);
    key0_pressed = (key0_now != key0_idle_level);

    // ===== KEY1 =====
    if (key1_pressed && !key1_latched)
    {
        DELAY_Nms(KEY_DEBOUNCE_MS);

        if (oled_mode == 1) // 阈值页
        {
            if (threshold_index == 0 && temp_threshold_on < 60)
                temp_threshold_on++;
            else if (threshold_index == 1 && hum_alarm_threshold < 99)
                hum_alarm_threshold++;
            else if (threshold_index == 2 && mq2_alarm_threshold < 10000)
                mq2_alarm_threshold += 50;
        }
        else if (oled_mode == 2) // 控制页
        {
            Servo_Control(1, 1);
        }

        key1_latched = 1;
    }
    else if (!key1_pressed)
        key1_latched = 0;

    // ===== KEY0 =====
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

    // ===== K_KEY =====
    if (!GPIO_ReadInputDataBit(K_KEY_PORT, K_KEY_PIN))
    {
        if (!k_key_latched)
        {
            DELAY_Nms(KEY_DEBOUNCE_MS);

            if (!GPIO_ReadInputDataBit(K_KEY_PORT, K_KEY_PIN))
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

// 从JSON文本中提取布尔字段，命中返回1并写入out_value(0/1)，未命中返回0
static u8 JsonTryGetBool(const char *json, const char *key, u8 *out_value)
{
    char pat_true[32];
    char pat_false[32];

    if (!json || !key || !out_value)
        return 0;

    snprintf(pat_true, sizeof(pat_true), "\"%s\":true", key);
    snprintf(pat_false, sizeof(pat_false), "\"%s\":false", key);

    if (strstr(json, pat_true))
    {
        *out_value = 1;
        return 1;
    }
    if (strstr(json, pat_false))
    {
        *out_value = 0;
        return 1;
    }
    return 0;
}

// 平台下发可能带转义符/空白，先就地归一化，便于 strstr 匹配
static void JsonNormalizeForMatch(char *buf)
{
    u16 r = 0;
    u16 w = 0;
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

static void Handle_WifiCommand(void)
{
    if (wifiRecvOver != 1)
        return;

    {
        u16 copy_len = (recvCnt < sizeof(check_char) - 1U) ? recvCnt : (sizeof(check_char) - 1U);
        memcpy(check_char, wifiRecvBuf, copy_len);
        check_char[copy_len] = '\0';
        JsonNormalizeForMatch((char *)check_char);
        Log_Print("Downlink:");
        Log_Print((char *)check_char);
        Log_Print("\r\n");

        {
            u8 cmd_val = 0;
            if (JsonTryGetBool((char *)check_char, "BEEP", &cmd_val))
                remote_beep_active = cmd_val;
        }

        {
            u8 cmd_val = 0;
            if (JsonTryGetBool((char *)check_char, "LED", &cmd_val))
            {
                remote_led_active = cmd_val;
                if (remote_led_active)
                {
                    BOARD_LED_ON();
                    Log_Print("Recv: LED ON\r\n");
                }
                else
                {
                    BOARD_LED_OFF();
                    Log_Print("Recv: LED OFF\r\n");
                }
            }
        }

        {
            u8 cmd_val = 0;
            if (JsonTryGetBool((char *)check_char, "SERVO", &cmd_val))
                Servo_Control(cmd_val ? 1 : 0, 1);
        }

        if (strstr((char *)check_char, "\"MODE\":\"AUTO\""))
            servo_control_mode = 0;
        else if (strstr((char *)check_char, "\"MODE\":\"MANUAL\""))
            servo_control_mode = 1;

        memset(wifiRecvBuf, 0, copy_len);
    }
    recvCnt = 0;
    wifiRecvOver = 0;
}

int main(void)
{
    NVIC_SetPriorityGrouping(2);
    // 注意:
    // 部分USB转串口/下载器在“打开串口调试助手”时会拉低复位线，导致MCU重启。
    // 现象是云平台短暂掉线，设备完成 WiFi/TCP 重连后会恢复在线。

    LED_Config();
    KEY_Config();
    USART1_Config(115200);
    DELAY_Nms(100);
    // 串口终端对齐(清理上电瞬态乱码)
    Log_Print("\r\n\r\n----BOOT----\r\n");
    SoftIIC_Config();
    OLED_Config();
    OLED_Clear(0);

    OLED_ShowString(0, 0, (u8 *)"TIME:");
    OLED_ShowString(2, 0, (u8 *)"TEMP:");
    OLED_ShowString(4, 0, (u8 *)"HUM :");
    OLED_ShowString(6, 0, (u8 *)"MQ2 :");

    BOARD_LED_OFF();
    BEEP_Init_Safe();

    DHT11_Config();
    ADCx_Init();
    MQ2_Init();
    SR602_Init();
    Servo_Init();
    Human_LED_Init();
    K_Key_Init();

    Log_Print("System Init OK\r\n");

    // 先进入本地工作模式，避免Wi-Fi初始化阻塞导致界面卡在初始画面
    wifi_online = 0;
    last_wifi_retry_time = system_timer;
    Log_Print("Local Mode Start\r\n");
#if WIFI_AUTO_BRINGUP
    // 启动后先尝试一次联网，失败则保持本地模式并后台重连
    wifi_online = Wifi_Bringup();
#endif

    while (1)
    {
        DELAY_Nms(LOOP_TICK_MS);
        system_timer += LOOP_TICK_MS;

        if (IsTimeDue(system_timer, &last_dht_time, TASK_SENSOR_MS))
        {            
            Update_SensorData();
        }

        if (IsTimeDue(system_timer, &last_human_check_time, TASK_HUMAN_MS))
        {
            // 周期采样PIR触发状态(关注“运动变化”而非绝对存在)
            // SR602_Detect() 约定: 1=检测到人体, 0=未检测到人体
            u8 human_raw = SR602_Detect() ? 1 : 0;
            human_status = human_raw; // 遥测保留原始状态

            if (!human_raw)
            {
                last_human_low_time = system_timer;
            }
            else
            {
                // 上升沿立即刷新；持续高电平时按固定间隔续期，避免“亮几秒后再也不亮”
                if (!human_raw_prev || ((system_timer - last_human_detect_time) >= HUMAN_RETRIGGER_MS))
                {
                    last_human_detect_time = system_timer;
                    if (IsNightPeriod())
                        night_beep_until = system_timer + NIGHT_BEEP_MS;
                }
            }
            human_raw_prev = human_raw;
        }

        if ((system_timer - last_human_detect_time) < HUMAN_LED_HOLD_MS)
            HUMAN_LED_ON();
        else
            HUMAN_LED_OFF();

        if (servo_control_mode == 0)
        {
            if (dht11_data[2] >= temp_threshold_on && servo_status == 0)
                Servo_Control(1, 0);
            else if (dht11_data[2] <= temp_threshold_off && servo_status == 1)
                Servo_Control(0, 0);
        }

        Servo_UpdateSweep();

        if (IsTimeDue(system_timer, &last_ui_time, TASK_UI_MS))
        {
            Update_DisplayAndAlarm();
        }

        Handle_Keys();

        if (wifi_online && IsTimeDue(system_timer, &last_mqtt_time, TASK_MQTT_MS))
        {
            if (cnd == 0)
                cnd = 1;
        }
        if (wifi_online)
        {
            Send_MQTT_Process();
            Handle_WifiCommand();
        }
        else
        {
            // Wi-Fi离线时保持本地功能（OLED/按键/舵机）正常运行，后台周期性重连
#if WIFI_AUTO_BRINGUP
            if (IsTimeDue(system_timer, &last_wifi_retry_time, WIFI_RETRY_MS))
                wifi_online = Wifi_Bringup();
#endif
        }

    }
}
