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

// ========================== ???????? ==========================
// ??????????:
// 1) TX/RX ????????
// 2) GND ???????????????????????????????????????????????
// ?????????:
// - ??????: GPIOB_8
// - ??????LED: GPIOE_5
// - ????: KEY1=GPIOE_3, KEY0=GPIOE_4
// - ?????(??????): GPIOF_9

// --- ?????? (GPIOB_8) ---
#define BEEP_PORT        GPIOB
#define BEEP_PIN         GPIO_Pin_8
#define BEEP_RCC         RCC_APB2Periph_GPIOB
#define BEEP_ON()        GPIO_ResetBits(BEEP_PORT, BEEP_PIN)
#define BEEP_OFF()       GPIO_SetBits(BEEP_PORT, BEEP_PIN)

// --- ????LED (GPIOB_5, ????????) ---
#define BOARD_LED_PORT   GPIOB
#define BOARD_LED_PIN    GPIO_Pin_5
#define BOARD_LED_ON()   GPIO_ResetBits(BOARD_LED_PORT, BOARD_LED_PIN)
#define BOARD_LED_OFF()  GPIO_SetBits(BOARD_LED_PORT, BOARD_LED_PIN)

// --- ??????LED (GPIOE_5) ---
#define HUMAN_LED_PORT   GPIOE
#define HUMAN_LED_PIN    GPIO_Pin_5
#define HUMAN_LED_RCC    RCC_APB2Periph_GPIOE
#define HUMAN_LED_ON()   GPIO_ResetBits(HUMAN_LED_PORT, HUMAN_LED_PIN)
#define HUMAN_LED_OFF()  GPIO_SetBits(HUMAN_LED_PORT, HUMAN_LED_PIN)
// SR602(PIR) ???????????:
// 1) ??????????????????????????????????????
// 2) ????????????????????(?????/?????????????SR602_Detect ??????)
// 3) ?????????????????????????????????????????(HUMAN_LED_HOLD_MS)???????

// --- ??????? (GPIOE_3 / GPIOE_4, ????????) ---
#define KEY1_PORT         GPIOE
#define KEY1_PIN          GPIO_Pin_3   // ??????????
#define KEY0_PORT         GPIOE
#define KEY0_PIN          GPIO_Pin_4   // ?????????
#define K_KEY_PORT        GPIOA
#define K_KEY_PIN         GPIO_Pin_0   // ???????????(????????)
#define K_KEY_RCC         RCC_APB2Periph_GPIOA

// --- 180???????? (PF9) ---
#define SERVO_GPIO_PORT    GPIOF
#define SERVO_GPIO_PIN     GPIO_Pin_9
#define SERVO_GPIO_RCC     RCC_APB2Periph_GPIOF

#define SERVO_ANGLE_MIN    500    // 0??
#define SERVO_ANGLE_MAX    2500   // 180??
#define SWEEP_STEP         40
#define SERVO_CONTINUOUS_MODE 1   // 1=????????????, 0=180??????????
#define SERVO_CW_PULSE      1620  // reduce current spike while keeping 360 rotation
#define SERVO_BOOST_PULSE   1620  // keep same as CW to avoid startup jerk
#define SERVO_BOOST_MS      0U    // disable boost phase for smooth continuous run
#define SERVO_UPDATE_DIV    1U    // 20ms update (50Hz) for smoother servo motion

// --- ??????? ---
#define TEMP_THRESHOLD_ON_DEFAULT    30
#define TEMP_THRESHOLD_OFF_DEFAULT   29
#define HUM_ALARM_THRESHOLD_DEFAULT  80
#define MQ2_ALARM_THRESHOLD_DEFAULT  500
// MQ2??????????????????ppm???????????
#define MQ2_ADC_MAX          4095UL
#define MQ2_RL_OHM           5000UL   // ????????(????5K)
#define MQ2_R0_OHM           10000UL  // ?????????????????I
#define MQ2_CLEAN_AIR_TARGET 200UL    // ??????????????(<200ppm)
#define MQ2_CALIB_SAMPLES    30U      // ???????????????2s?????1?????
#define MQ2_REPORT_MAX        9999UL   // ????????MQ2??????????????

// --- ????????????(ms) ---
#define LOOP_TICK_MS      20U
#define TASK_SENSOR_MS    500U
#define TASK_HUMAN_MS     100U
#define TASK_UI_MS        200U
#define TASK_MQTT_MS      15000U
#define MQTT_WARMUP_MS    120000U
#define HUMAN_LED_HOLD_MS 3000U
#define HUMAN_RETRIGGER_MS 2000U
#define KEY_DEBOUNCE_MS   10U
#define WIFI_BOOT_DELAY_MS 1500U
#define BOOT_STABILIZE_MS  3000U
#define WIFI_INIT_RETRY    3U
#define WIFI_RETRY_MS      30000U
#define WIFI_AUTO_BRINGUP  1U   // 1: auto WiFi bring-up enabled
#define NIGHT_START_HOUR   0U   // ????(24??)
#define NIGHT_END_HOUR     7U   // ??????(7??????7??)
#define CLOCK_START_HOUR   0U   // ??????(???)????RTC??????????
#define CLOCK_START_MIN    0U   // ??????(????)
#define CLOCK_START_SEC    0U   // ??????(??)
#define NIGHT_BEEP_MS      3000U
#define USE_RTC_CLOCK      0U   // 1=???RTC_GetCounter???????, 0=???system_timer???????
#define SERIAL_PC_BRIDGE    1U   // 1=???????JSON??PC?????????????

// ========================== ?????? ==========================
// MQTT???????:
// ??? MQTT.fx ?????????????????????? ClientID/????????
// ???? broker ????????????????????????????????
// ????????????????????PC?????????????????????? WiFi/??????????????????
// ThingCloud ?????(????????):
// 1) ???(????) Topic ?????????? attributes
// 2) ????(????) Topic ?????????????? Topic(??? data/stream/set)
// 3) ???? MQTT PUBLISH ??????????? JSON ???
// ???????????????:
// A. ??????(?????/Token/ClientID/?????????)???????
// B. MQTT????????????????????(???: bj-2-mqtt.iot-api.com:1883)
// C. ????Topic=MQTT_PUB_TOPIC??????Topic????????????????
// D. ???????????????????(JSON???: TIME/TEMP/HUM/MQ2/HUMAN/SERVO/BEEP/LED)
#define MQTT_PUB_TOPIC "attributes"
#define MQTT_SUB_TOPIC "data/stream/set"
// ???????????:
// 1) WiFi/TCP ???????????? TASK_MQTT_MS ??????? Send_MQTT_Process()
// 2) ?????????? MQTT_PUB_TOPIC ???????(??????????)
// 3) ??????????: ???????/?????Topic(????/????)??JSON???????????
static u8 Pub_Topic[] = MQTT_PUB_TOPIC;
static u8 Pub_Message[192];
static u8 check_char[256];
static u8 pub_field_index = 0;

static u32 ppm = 0;
static u8 human_status = 0;
static u8 temp_threshold_on = TEMP_THRESHOLD_ON_DEFAULT;
static u8 temp_threshold_off = TEMP_THRESHOLD_OFF_DEFAULT;
static u8 hum_alarm_threshold = HUM_ALARM_THRESHOLD_DEFAULT;
static u32 mq2_alarm_threshold = MQ2_ALARM_THRESHOLD_DEFAULT;

// ??????????
static u8 servo_status = 0;       // 0-???, 1-????(???)
static u8 servo_control_mode = 0; // 0-?????, 1-?????

// ?????????
static u16 servo_curr_pulse = SERVO_ANGLE_MIN;
static u8 servo_sweep_dir = 1; // 1=??180???, 0=??0???
static u32 servo_boost_until = 0;

// ??????????
static u8 dht11_data[5] = {0};
static u16 mq2_raw = 0;
static u32 mq2_baseline = 0;
static unsigned long long mq2_cal_sum = 0;
static u8 mq2_cal_count = 0;
static u8 mq2_cal_done = 0;
static u32 mq2_filtered_ppm = 0;

// ??????????
static u8 remote_beep_active = 0;
static u8 remote_led_active = 0;

static int cnd = 0;
static int len = 0;
static u8 wifi_online = 0;

// ?????? (????: ms)
// ???:
// - system_timer ????RTC???????????????????
// - ???????? DELAY_Nms(LOOP_TICK_MS) ????? system_timer += LOOP_TICK_MS
// - ?????????????????????????????????????
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
static u8 key_combo_latched = 0;
static u8 key_level_ready = 0;
static u8 key1_idle_level = 1;
static u8 key0_idle_level = 1;
// ===== ??UI?? =====
// 0=????? 1=???? 2=?????
static u8 oled_mode = 0;
//static u8 oled_status_view = 0;  // 0=??? 1=?????? 2=?????? 3=MQ2???

// ????????0??? 1??? 2MQ2??
static u8 threshold_index = 0;
static u32 night_beep_until = 0;
static u8 human_raw_prev = 0;
static u32 last_human_low_time = 0;

// ========================== ???????? ==========================
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

static u8 Wifi_Bringup(void)
{
    u8 i;

    // ????????? Wi-Fi ????????????????????????
    DELAY_Nms(WIFI_BOOT_DELAY_MS);

    for (i = 0; i < WIFI_INIT_RETRY; i++)
    {
        Log_Print("WiFi Bringup...\r\n");
        UART_Init();

        // ?????AT?????????????????????/?????????????
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
    // ??????RTC?????: RTC_Config()/?????????????
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
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
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
            target_pulse = SERVO_CW_PULSE; // ????????????
    }
    else
    {
        // ????????????????????????????????????????????
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

    // ??PWM?????????CPU???????????????????????????/??????????????
    servo_div_cnt++;
    if (servo_div_cnt < SERVO_UPDATE_DIV)
        return;
    servo_div_cnt = 0;

    Servo_OutputPulse(target_pulse);
}

static void Send_MQTT_Process(void)
{
    if (cnd == 0)
        return;

    // Rate-limit mode: publish one field per period.
    switch (pub_field_index)
    {
    case 0:
        snprintf((char *)Pub_Message, sizeof(Pub_Message), "{\"TEMP\":%d}", dht11_data[2]);
        break;
    case 1:
        snprintf((char *)Pub_Message, sizeof(Pub_Message), "{\"HUM\":%d}", dht11_data[0]);
        break;
    case 2:
        snprintf((char *)Pub_Message, sizeof(Pub_Message), "{\"MQ2\":%lu}", (unsigned long)ppm);
        break;
    case 3:
        snprintf((char *)Pub_Message, sizeof(Pub_Message), "{\"HUMAN\":%s}", human_status ? "true" : "false");
        break;
    case 4:
        snprintf((char *)Pub_Message, sizeof(Pub_Message), "{\"SERVO\":%s}", servo_status ? "true" : "false");
        break;
    default:
        snprintf((char *)Pub_Message, sizeof(Pub_Message), "{\"TEMP\":%d}", dht11_data[2]);
        break;
    }

    Log_Print("PUB Topic:");
    Log_Print((char *)Pub_Topic);
    Log_Print("\r\n");
    SerialBridge_Print((char *)Pub_Message);
    len = MqttPublishData((char *)Pub_Topic, (char *)Pub_Message, strlen((char *)Pub_Message));
    if (len <= 0)
    {
        Log_Print("MQTT Encode Fail\r\n");
    }
    else
    {
        char at_buf[30] = {0};
        snprintf(at_buf, sizeof(at_buf), "AT+CIPSEND=%d", len);
        if (cmdAT(at_buf, "OK", ">", strlen(at_buf)))
        {
            if (Wifi_SendRaw(MQTT_SEND_RealtimeData, len, "SEND OK", NULL))
                Log_Print("MQTT Pub Success\r\n");
            else
                Log_Print("MQTT Pub Fail\r\n");
        }
    }

    if (pub_field_index < 4)
        pub_field_index++;
    else
        pub_field_index = 0;

    if (cnd > 0)
        cnd--;
}

static void Update_SensorData(void)
{
    u32 mq2_raw_ppm;
    u32 mq2_corrected;
    u8 i;

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
    mq2_raw_ppm = (u32)MQ2_GetData_PPM();
    if (mq2_raw_ppm > 1000000UL)
    {
        // ???????????????????????????????????
        mq2_raw_ppm = (u32)mq2_raw * 100UL;
    }

    // ???????????????????????????????????????
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

    mq2_corrected = (mq2_raw_ppm > mq2_baseline) ? (mq2_raw_ppm - mq2_baseline) : 0;
    if (mq2_corrected > MQ2_REPORT_MAX)
        mq2_corrected = MQ2_REPORT_MAX;

    // 1st-order IIR: new = old*3/4 + sample*1/4, reduce sudden spikes.
    if (mq2_filtered_ppm == 0)
        mq2_filtered_ppm = mq2_corrected;
    else
        mq2_filtered_ppm = (mq2_filtered_ppm * 3UL + mq2_corrected) / 4UL;

    ppm = mq2_filtered_ppm;
}

static void Update_DisplayAndAlarm(void)
{
    char buf[20];
    char time_str[12];
    static u8 prev_oled_mode = 0xFF;

    // ===== ???????????????????=====
    u8 night_human_alarm = (system_timer < night_beep_until) ? 1 : 0;
    u8 alarm_trigger = (dht11_data[2] >= temp_threshold_on ||
                        dht11_data[0] > hum_alarm_threshold ||
                        ppm > mq2_alarm_threshold ||
                        night_human_alarm);

    if (alarm_trigger || remote_beep_active)
        BEEP_ON();
    else
        BEEP_OFF();

    // Avoid full-screen clear on every refresh to prevent flicker.
    if (prev_oled_mode != oled_mode)
    {
        OLED_Clear(0);
        prev_oled_mode = oled_mode;
    }
    FormatClockString(time_str, sizeof(time_str));

    // ================= ??1??????? =================
    if (oled_mode == 0)
    {
        OLED_ShowString(0, 0, (u8 *)"TIME:");
        OLED_ShowString(0, 48, (u8 *)time_str);

        snprintf(buf, sizeof(buf), "TEMP:%dC   ", dht11_data[2]);
        OLED_ShowString(2, 0, (u8 *)buf);

        snprintf(buf, sizeof(buf), "HUM :%d%%  ", dht11_data[0]);
        OLED_ShowString(4, 0, (u8 *)buf);

        snprintf(buf, sizeof(buf), "MQ2 :%lu    ", (unsigned long)ppm);
        OLED_ShowString(6, 0, (u8 *)buf);
    }

    // ================= ??2?????? =================
    else if (oled_mode == 1)
    {
        snprintf(buf, sizeof(buf), "TEMP:%d   ", temp_threshold_on);
        OLED_ShowString(0, 16, (u8 *)buf);

        snprintf(buf, sizeof(buf), "HUM :%d   ", hum_alarm_threshold);
        OLED_ShowString(2, 16, (u8 *)buf);

        snprintf(buf, sizeof(buf), "MQ2 :%lu  ", (unsigned long)mq2_alarm_threshold);
        OLED_ShowString(4, 16, (u8 *)buf);

        // ???????????
        OLED_ShowString(0, 0, (u8 *)(threshold_index == 0 ? ">" : " "));
        OLED_ShowString(2, 0, (u8 *)(threshold_index == 1 ? ">" : " "));
        OLED_ShowString(4, 0, (u8 *)(threshold_index == 2 ? ">" : " "));
    }

    // ================= ??3??????? =================
    else if (oled_mode == 2)
    {
        sprintf(buf, "SERVO:%s", servo_status ? "ON" : "OFF");
        OLED_ShowString(0, 0, (u8 *)buf);

        sprintf(buf, "HUMAN:%s", human_status ? "YES" : "NO");
        OLED_ShowString(2, 0, (u8 *)buf);

        sprintf(buf, "MODE :%s", servo_control_mode ? "MANUAL" : "AUTO");
        OLED_ShowString(4, 0, (u8 *)buf);

        OLED_ShowString(6, 0, (u8 *)"                ");
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
        key_level_ready = 1;
    }

    // KEY0/KEY1 are pull-up inputs: pressed when level is low.
    key1_pressed = (key1_now == Bit_RESET);
    key0_pressed = (key0_now == Bit_RESET);

    // ????(KEY1+KEY0)??????????????K_KEY??????????????????????
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

    // ???????????????????????????????????????????
    if (key1_pressed && key0_pressed)
        return;

    // ===== KEY1 =====
    if (key1_pressed && !key1_latched)
    {
        DELAY_Nms(KEY_DEBOUNCE_MS);

        if (oled_mode == 1) // ????
        {
            if (threshold_index == 0 && temp_threshold_on < 60)
                temp_threshold_on++;
            else if (threshold_index == 1 && hum_alarm_threshold < 99)
                hum_alarm_threshold++;
            else if (threshold_index == 2 && mq2_alarm_threshold < 10000)
                mq2_alarm_threshold += 50;
        }
        else if (oled_mode == 2) // ?????
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

// ??JSON?????????????????????????1??????out_value(0/1)????????????0
static u8 JsonTryGetSwitch(const char *json, const char *key, u8 *out_value)
{
    char pat_true[32];
    char pat_false[32];

    if (!json || !key || !out_value)
        return 0;

    snprintf(pat_true, sizeof(pat_true), "\"%s\":true", key);
    snprintf(pat_false, sizeof(pat_false), "\"%s\":false", key);
    snprintf(pat_one, sizeof(pat_one), "\"%s\":1", key);
    snprintf(pat_zero, sizeof(pat_zero), "\"%s\":0", key);
    snprintf(pat_str_true, sizeof(pat_str_true), "\"%s\":\"true\"", key);
    snprintf(pat_str_false, sizeof(pat_str_false), "\"%s\":\"false\"", key);
    snprintf(pat_str_one, sizeof(pat_str_one), "\"%s\":\"1\"", key);
    snprintf(pat_str_zero, sizeof(pat_str_zero), "\"%s\":\"0\"", key);

    if (strstr(json, pat_true) || strstr(json, pat_one) || strstr(json, pat_str_true) || strstr(json, pat_str_one))
    {
        *out_value = 1;
        return 1;
    }
    if (strstr(json, pat_false) || strstr(json, pat_zero) || strstr(json, pat_str_false) || strstr(json, pat_str_zero))
    {
        *out_value = 0;
        return 1;
    }
    return 0;
}

// ???????????????/?????????????????? strstr ???
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
        memcpy(check_char, (const void *)wifiRecvBuf, copy_len);
        check_char[copy_len] = '\0';
        JsonNormalizeForMatch((char *)check_char);
        Log_Print("Downlink:");
        Log_Print((char *)check_char);
        Log_Print("\r\n");

        {
            u8 cmd_val = 0;
            if (JsonTryGetSwitch((char *)check_char, "BEEP", &cmd_val))
                remote_beep_active = cmd_val;
        }

        {
            u8 cmd_val = 0;
            if (JsonTryGetSwitch((char *)check_char, "LED", &cmd_val))
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
            if (JsonTryGetSwitch((char *)check_char, "SERVO", &cmd_val))
                Servo_Control(cmd_val ? 1 : 0, 1);
        }

        if (strstr((char *)check_char, "\"MODE\":\"AUTO\""))
            servo_control_mode = 0;
        else if (strstr((char *)check_char, "\"MODE\":\"MANUAL\""))
            servo_control_mode = 1;

        memset((void *)wifiRecvBuf, 0, copy_len);
    }
    recvCnt = 0;
    wifiRecvOver = 0;
}

int main(void)
{
    NVIC_SetPriorityGrouping(2);
    // ???:
    // ????USB?????/???????????????????????????????????????MCU??????
    // ???????????????????????? WiFi/TCP ??????????????

    LED_Config();
    KEY_Config();
    USART1_Config(115200);
    DELAY_Nms(100);
    // ??????????(?????????????)
    Log_Print("\r\n\r\n----BOOT----\r\n");
    Log_ResetReason();
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
    Update_SensorData();

    Log_Print("System Init OK\r\n");
    DELAY_Nms(BOOT_STABILIZE_MS);

    // ??????????????????Wi-Fi????????????????B????????
    wifi_online = 0;
    last_wifi_retry_time = system_timer;
    Log_Print("Local Mode Start\r\n");
#if WIFI_AUTO_BRINGUP
    // ????????????????????????????????????????
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
            // ???????PIR??????(???????????????????????)
            // SR602_Detect() ???: 1=???????, 0=?????????
            u8 human_raw = SR602_Detect() ? 1 : 0;
            human_status = human_raw; // ?????????

            if (!human_raw)
            {
                last_human_low_time = system_timer;
            }
            else
            {
                // ????????????????????????????????????????????????????????
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

        if (wifi_online && system_timer >= MQTT_WARMUP_MS && IsTimeDue(system_timer, &last_mqtt_time, TASK_MQTT_MS))
        {
            if (cnd == 0)
            {
                cnd = 1;
            }
        }
        if (wifi_online)
        {
            Handle_WifiCommand();
            Send_MQTT_Process();
        }
        else
        {
            // Wi-Fi????????????????OLED/????/????????????????????????????
#if WIFI_AUTO_BRINGUP
            if (IsTimeDue(system_timer, &last_wifi_retry_time, WIFI_RETRY_MS))
                wifi_online = Wifi_Bringup();
#endif
        }

    }
}
