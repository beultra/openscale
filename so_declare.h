#ifndef DECLARE_H
#define DECLARE_H
#include "so_config.h"

#include <HX711_ADC.h>
HX711_ADC scale(HX711_SDA, HX711_SCL);  //HX711模数转换初始化

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
BLEServer *pServer = NULL;
BLEService *pService = NULL;
BLEAdvertising *pAdvertising = NULL;
BLECharacteristic *pReadCharacteristic = NULL;
BLECharacteristic *pWriteCharacteristic = NULL;
bool deviceConnected = false;
bool b_ble_enabled = true;

// The model byte is always 03 for Decent scales
const byte modelByte = 0x03;
unsigned long t_heartBeat = 0;
unsigned long t_firstConnect = 0;
bool b_requireHeartBeat = true;

unsigned long lastBleWeightNotifyTime = 0;  // Stores the last time the weight notification was sent
unsigned long weightBleNotifyInterval = 100;  // Interval at which to send weight notifications (milliseconds)
int i_onWrite_counter = 0;
bool b_bootTare = false;
int i_bootTareDelay = 1000;
//int i_tareDelay = 200;             //tare delay for button
int i_tareDelay = 0;             //tare delay 0ms for finger detection
unsigned long t_tareByButton = 0;  //tare time stamp used by button to mimic delay
bool b_tareByButton = false;
unsigned long t_tareByBle = 0;
bool b_tareByBle = false;
unsigned long t_tareStatus = 0;  //tare done time stamp
unsigned long t_power_off;       //关机倒计时
bool b_powerOff = false;


//电子秤参数和计时点
// Enhanced tracking system global variables
static float f_tracking_offset = 0.0;              // Current tracking offset
static float f_tracking_target = 0.0;              // Current tracking target weight
static unsigned long t_last_tracking_update = 0;   // Last tracking update time
static unsigned long TRACKING_UPDATE_INTERVAL = 1000; // Tracking update interval 1 seconds
static float TRACKING_THRESHOLD = 0.1;      // Tracking stability threshold
static const int i_STABLE_COUNT_THRESHOLD = 5;     // Stable count threshold
static const float MAX_TRACKING_ADJUSTMENT = 0.5;  // Maximum single adjustment

static unsigned long t_last_status_display = 0;
static const unsigned long STATUS_DISPLAY_INTERVAL = 5000;
static bool b_weight_in_serial = false;

static int i_stable_count = 0;                     // Stable state counter
static bool b_tracking_enabled = true;          // Tracking enable flag
static bool b_tracking_active = false;          // Whether tracking is currently active

// Stable output system global variables
static float f_previous_stable_value = 0.0;        // Previous stable output value
static float f_current_raw_value = 0.0;            // Current raw input value
static float STABLE_OUTPUT_THRESHOLD = 0.1;       // Minimum change to update output
static bool b_stable_output_enabled = true;     // Stable output enable flag
static unsigned long t_last_stable_change = 0;     // Time of last stable change
static float f_driftCompensation = 0.0;  // Continuous temperature drift compensation
static float f_maxDriftCompensation = 0.05;  // Maximum micro-drift range for temperature compensation (g)
// Range: 0.01g to this value will be considered as temperature drift
// Values above this are considered as real weight changes, not drift
//bool b_tempDisablePowerOff = true;

bool b_negativeWeight = false;

bool b_weight_quick_zero = false;           //Tare后快速显示为0优化
char c_weight[10];                          //咖啡重量显示
char c_brew_ratio[10];                      //粉水比显示
unsigned long t_extraction_begin = 0;       //开始萃取打点
unsigned long t_extraction_first_drop = 0;  //下液第一滴打点
unsigned long t_extraction_last_drop = 0;   //下液结束打点
unsigned long t_ready_to_brew = 0;          //准备好冲煮的时刻(手冲)
int i_extraction_minimium_timer = 7;        //前7秒不判断停止计时

unsigned long t_PowerDog = 0;             //电源5s看门狗
int tareCounter = 0;                      //不稳计数器
const float f_weight_default_coffee = 0;  //默认咖啡粉重量

float aWeight = 0;         //稳定状态比对值（g）
float aWeightDiff = 0.15;  //稳定停止波动值（g）
float atWeight = 0;        //自动归零比对值（g）
float atWeightDiff = 0.3;  //自动归零波动值（g）
float asWeight = 0;        //下液停止比对值（g）
float asWeightDiff = 0.1;  //下液停止波动值（g）
float f_weight_adc = 0.0;  //原始读出值（g）
float f_weight_smooth;
float f_displayedValue;
float f_flow_rate;

#endif