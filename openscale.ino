/*
  2021-07-25 新增开机时校准，双击咖啡录入关机
  2021-07-26 新增开机按住清零校准，取消双击录入关机
  2021-08-02 新增开机修改sample，显示版本信息，手柄录入功能
  2021-08-07 v1.1 新增手柄录入功能
  2021-08-15 v1.1 去掉手柄录入（因为双头手柄含水量不一定），修复进入意式模式时未恢复参数，新增电量检测
  2021-09-01 v1.2 重新加入手柄录入 修复sample不可更改bug
  2021-09-03 v1.3 修复切换到意式模式直接计时问题 修复录入可能产生负值问题
  2021-10-02 v1.4 修复切换到意式模式 下液计时不清零问题
  2021-10-10 v1.5 修复手柄录入产生的计时问题 新增显示旋转功能
  2022-02-03 v1.6 二按钮模式
  2022-03-04 v1.7 流速计
  2022-04-12 v1.8 优化电量显示为0时闪屏
  2022-08-01 v1.9 尝试支持3.3v芯片
  2022-08-06 v2.0 换用rp2040，支持无操作自动关机
  2022-09-11 v2.1 支持双模式，支持六轴传感器侧放关机
  2022-11-02 v2.2 使用ESP32 wemos lite，支持esp32 休眠，支持esp32电容按钮开关机，去掉5v充放一体单元，去掉了电量显示
  2023-02-11 v2.3 倾斜时不开机，避免误触
    bug fix:  setsamplesinuse只能缩小原来config.h中的SAMPLES，不能增加。
  2023-03-06 v2.4 大幅改进显示稳定性
  2023-03-11 v3.0 WiFi OTA，更换字体大小
  2023-06-24 v3.1 去掉WiFi功能，加入蓝牙串口，可自定义所有参数。
  2023-12-11 v3.2 ESPNow无线传输参数显示
  2023-12-23 v3.3 ESPNow左键开启，和蓝牙一样，加入ssd1312
  2024-01-27 v3.4 加入静音后代替蜂鸣器的LED
  2024-03-25 v3.5 Add early verion of English translation
  2024-04-06 v4.0 Add BLE and uuid.

  todo
  开机M进入菜单
  //2023-02-11 关闭蜂鸣器 DONE(2023-06-25)
  2023-03-06 使用enum菜单
  2024-03-23 Impliment ble function using service uuid and charactoristic uuid to let other apps/devices to get the scale data, or have it tared.
*/

//include

#include <Arduino.h>

#if defined(ESP8266) || defined(ESP32) || defined(AVR) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_MBED_RP2040)
#include <EEPROM.h>
#endif

#include "so_parameter.h"
#include "so_declare.h"
#include "so_config.h"


#define CUUID_DECENTSCALE_READ "fff4"
#define CUUID_DECENTSCALE_WRITE "36f5"
// #define CUUID_DECENTSCALE_WRITEBACK "83CDC3D4-3BA2-13FC-CC5E-106C351A9352"
#define SUUID_DECENTSCALE "fff0"

#define BTLOG_print(a...) {}
#define BTLOG_println(a...) {}

// #include <BluetoothSerial.h>
// BluetoothSerial SerialBT;
enum BleState {
  DISCONNECTED,
  CONNECTED
};
BleState bleState = DISCONNECTED;
const unsigned long HEARTBEAT_TIMEOUT = 5000;  // 5 seconds
unsigned long t_lastDisconnectAttempt = 0;
unsigned long t_lastDisconnectAttemptNotice = 0;

//functions
void sendBleVoltage();
void sendBleLedResponse();
#if defined(ACC_MPU6050) || defined(ACC_BMA400)
void sendBleGyro();
#endif
uint16_t connId = 0xFFFF; // not set to 0 because 0 could be a valid client id.


//ble
// Function to calculate XOR for validation (assuming this might still be needed)
uint8_t calculateXOR(uint8_t *data, size_t len) {
  uint8_t xorValue = 0x03;                // Starting value for XOR as per your example
  for (size_t i = 1; i < len - 1; i++) {  // Start from 1 to len - 1 assuming last byte is XOR value
    xorValue ^= data[i];
  }
  return xorValue;
}

// Encode weight into two bytes, big endian
void encodeWeight(float weight, byte &byte1, byte &byte2) {
  int weightInt = (int)(weight * 10);  // Convert to grams * 10
  byte1 = (byte)((weightInt >> 8) & 0xFF);
  byte2 = (byte)(weightInt & 0xFF);
}

// This callback will be invoked when a device connects or disconnects
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    connId = pServer->getConnId();  // 保存连接 ID
    Serial.print("BLE connID is: ");
    Serial.println(connId);
    t_firstConnect = millis();
    t_heartBeat = millis();
    bleState = CONNECTED;
    deviceConnected = true;
#ifdef BUZZER
    b_beep = false;  //disable buzzer once when connected, and wait for ble command to enable it
#endif
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer *pServer) {
    connId = 0xFFFF; // not set to 0 because 0 could be a valid client id

    deviceConnected = false;
    bleState = DISCONNECTED;

//Serial.println("ble 01");
   //Serial.println("ble 02");

    Serial.println("Device disconnected, restarting advertising...");

    delay(100);  // advertise after a short delay
    
    // maybe remove delay in future version or use somthing like this:
    /*
    void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    restartAdvertising = true; // Just set the flag
    }

    void loop() {
        if (restartAdvertising) {
            delay(10); // Very short delay outside the callback is safer
            pAdvertising->start();
            restartAdvertising = false;
            Serial.println("Advertising restarted safely in loop");
        }
    }*/
    pAdvertising->start();
    //Serial.println("ble 5");
  }
};
/*  
        Weight received on	FFF4 (0000FFF4-0000-1000-8000-00805F9B34FB)

        Firmware v1.0 and v1.1 sends weight as a 7 byte message:
        03CE 0000 0000 CD = 0.0 grams
        03CE 0065 0000 A8 = 10.1 grams
        03CE 0794 0000 5E = 194.0 grams
        03CE 1B93 0000 5E = 705.9 grams
        03CE 2BAC 0000 4A = 1118.0 grams

        Firmware v1.2 and newer sends weight with a timestamp as a 10 byte message:
        03CE 0000 010203 0000 CD = 0.0 grams - (1 minute, 2 seconds, 3 milliseconds)
        03CE 0065 010204 0000 A8 = 10.1 grams - (1 minute, 2 seconds, 4 milliseconds)
        03CE 0794 010205 0000 5E = 194.0 grams - (1 minute, 2 seconds, 5 milliseconds)
        03CE 1B93 010206 0000 5E = 705.9 grams - (1 minute, 2 seconds, 6 milliseconds)
        03CE 2BAC 010207 0000 4A = 1118.0 grams - (1 minute, 2 seconds, 7 milliseconds)

        030A LED and Power
        LED on [requires v1.1 firmware]
        030A 0101 000009 (grams)
        030A 0101 010008 (ounces) 
        LED off	
        030A 0000 000009
        Power off (new in v1.2 firmware)
        030A 0200 00000B

        030B Timer
        Timer start	
        030B 0300 00000B
        Timer stop	
        030B 0000 000008
        Timer zero	
        030B 0200 00000A

        030F Tare (set weight to zero)	
        030F 0000 00000C
        030F B900 0000B5
        
        sofronio edit 2024-08-31
        031A Start calibration
        031B Start WiFi OTA
        031C 00 Buzzer off
        031C 01 Buzzer on

        03 2A 00 Power off by button
        03 2A 01 Power off by low power        
*/
class MyCallbacks : public BLECharacteristicCallbacks {
  uint8_t calculateChecksum(uint8_t *data, size_t len) {
    uint8_t xorSum = 0;
    // Iterate over each byte in the data, excluding the last one assumed to be the checksum
    for (size_t i = 0; i < len - 1; i++) {
      xorSum ^= data[i];
    }
    return xorSum;
  }

  // Validate the checksum of the data
  bool validateChecksum(uint8_t *data, size_t len) {
    if (len < 2) {  // Need at least 1 byte of data and 1 byte of checksum
      return false;
    }
    uint8_t expectedChecksum = data[len - 1];
    uint8_t calculatedChecksum = calculateChecksum(data, len);
    return expectedChecksum == calculatedChecksum;
  }

  void onWrite(BLECharacteristic *pWriteCharacteristic) {
    //this is what the esp32 received via ble
    Serial.print("Timer");
    Serial.print(millis());
    /* 
    millis() function returns an unsigned long. It will overflow (roll over back to zero) after approximately 49.7 days.
    This will NOT overflow for 290,000 years:
    uint64_t permanent_millis = esp_timer_get_time() / 1000; 
    */
  
    if (pWriteCharacteristic != nullptr) {                         // Check if the characteristic is valid
      size_t len = pWriteCharacteristic->getLength();              // Get the data length
      uint8_t *data = (uint8_t *)pWriteCharacteristic->getData();  // Get the data pointer

      // Optionally print the received HEX for verification or debugging
      Serial.print("Received HEX: ");
      for (size_t i = 0; i < len; i++) {
        if (data[i] < 0x10) {  // Check if the byte is less than 0x10
          Serial.print("0");   // Print a leading zero
        }
        Serial.print(data[i], HEX);  // Print the byte in HEX
      }
      Serial.print(" ");
      if (data[0] == 0x03) {
        //check if it's a decent scale message
        if (data[1] == 0x0F) {
          //taring
          if (validateChecksum(data, len)) {
            Serial.println("Valid checksum for tare operation. Taring");
          } else {
            Serial.println("Invalid checksum for tare operation.");
          }
          b_tareByBle = true;
          t_tareByBle = millis();
          if (data[5] == 0x00) {
            /*
            Tare the scale by sending "030F000000000C" (old version, disables heartbeat)
            Tare the scale by sending "030F000000010D" (new version, leaves heartbeat as set)
            */
            b_requireHeartBeat = false;
            Serial.println("*** Heartbeat detection Off ***");
          }
          if (data[5] == 0x01) {
            /*
            Tare the scale by sending "030F000000000C" (old version, disables heartbeat)
            Tare the scale by sending "030F000000010D" (new version, leaves heartbeat as set)
            */
            Serial.print("*** Heartbeat detection remained ");
            if (b_requireHeartBeat)
              Serial.print("On");
            else
              Serial.print("Off");
            Serial.println(" ***");
          }
        } else if (data[1] == 0x0A) {
          if (data[2] == 0x00) {
            Serial.println("LED off detected. Turn off OLED.");
            sendBleLedResponse();//include weight voltage version
          } else if (data[2] == 0x01) {
            Serial.println("LED on detected. Turn on OLED.");
            sendBleLedResponse();//including weight voltage version
            if (data[5] == 0x00) {
              b_requireHeartBeat = false;
              Serial.println("*** Heartbeat detection Off ***");
            }
            if (data[5] == 0x01) {
              Serial.print("*** Heartbeat detection remained ");
              if (b_requireHeartBeat)
                Serial.print("On");
              else
                Serial.print("Off");
              Serial.println(" ***");
            }
          } else if (data[2] == 0x02) {
            Serial.println("Power off detected.");
             b_powerOff = true;
          } else if (data[2] == 0x03) {
            if (data[3] == 0x01) {
              Serial.println("Start Low Power Mode.");
            
            } else if (data[3] == 0x00) {
              Serial.println("Exit low power mode.");
              
            } else if (data[3] == 0xFF) {
              if (data[4] == 0xFF) {
                if (data[5] == 0x00) {
                  if (data[6] == 0x0A) {
                    t_heartBeat = millis();
                    Serial.print("*** Heartbeat at ");
                    Serial.print(t_heartBeat);
                    Serial.println(" ***");
                  }
                }
              }
            }
          } else if (data[2] == 0x04) {
            if (data[3] == 0x01) {
              Serial.println("Start Soft Sleep.");
              
              //b_softSleep = true;
              
              //#endif
            } else if (data[3] == 0x00) {
              Serial.println("Exit Soft Sleep.");
              
             // b_softSleep = false;
            }
          }
        } else if (data[1] == 0x0B) {
          if (data[2] == 0x03) {
            Serial.println("Timer start detected.");
            
          } else if (data[2] == 0x00) {
            Serial.println("Timer stop detected.");
            
          } else if (data[2] == 0x02) {
            Serial.println("Timer zero detected.");
            
          }
        } else if (data[1] == 0x1A) {
          if (data[2] == 0x00) {
            Serial.println("Manual Calibration via BLE");
            //i_button_cal_status = 1;
            //i_calibration = 0;
            //b_calibration = true;
          } else if (data[2] == 0x01) {
            Serial.println("Smart Calibration via BLE");
            //i_button_cal_status = 1;
            //i_calibration = 1;
            //b_calibration = true;
          }
        } else if (data[1] == 0x1B) {
          Serial.println("Start WiFi OTA");
   //       wifiUpdate();
        }
        else if (data[1] == 0x1D) {  //Sample settings
          if (data[2] == 0x00) {
            scale.setSamplesInUse(1);
            Serial.print("Samples in use set to: ");
            Serial.println(scale.getSamplesInUse());
          } else if (data[2] == 0x01) {
            scale.setSamplesInUse(2);
            Serial.print("Samples in use set to: ");
            Serial.println(scale.getSamplesInUse());
          } else if (data[2] == 0x03) {
            scale.setSamplesInUse(4);
            Serial.print("Samples in use set to: ");
            Serial.println(scale.getSamplesInUse());
          }
        } else if (data[1] == 0x1F) {
          // reset();
        }
        else if (data[1] == 0x22) {
          sendBleVoltage();
        }
      }
    }
  }
};


void ble_init() {
  //turn on ble
  BLEDevice::init("Decent Scale");
  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create BLE Service
  pService = pServer->createService(SUUID_DECENTSCALE);
  pWriteCharacteristic = pService->createCharacteristic(
    CUUID_DECENTSCALE_WRITE,
    BLECharacteristic::PROPERTY_WRITE);
  pWriteCharacteristic->setCallbacks(new MyCallbacks());
  pReadCharacteristic = pService->createCharacteristic(
    CUUID_DECENTSCALE_READ,
    BLECharacteristic::PROPERTY_READ
      | BLECharacteristic::PROPERTY_NOTIFY);
  pService->start();
  // Start advertising
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->start();

  Serial.println("Waiting for a client connection to notify...");
}

void disconnectBLE() {
  if (deviceConnected) {
    Serial.println("***No heartbeat for 5 seconds. Disconnecting BLE...***");
    // Only try disconnecting every 5 seconds.
    if (millis() - t_lastDisconnectAttempt < 5000) {
      if (millis() - t_lastDisconnectAttemptNotice > 1000){
        Serial.println("Disconnect attempt too frequent, skipping...");
        t_lastDisconnectAttemptNotice = millis();
      }
      return;
    }
    t_lastDisconnectAttempt = millis();
    pServer->disconnect(connId, 0x13); // must prove connID for proper disconnecting. 0x13 for disconnect from remote device ESP_GAP_BLE_UPDATE_CONN_PARAMS_ERR_REMOTE_DEVICE_DISCONN. 
  }
}

// Build voltage data packet
void buildVoltagePacket(byte data[7]) {
  byte voltageByte1, voltageByte2;
  float f_batteryVoltage = 3.3;
  encodeWeight(f_batteryVoltage, voltageByte1, voltageByte2);

  data[0] = modelByte;
  data[1] = 0x22;  // Voltage type
  data[2] = voltageByte1;
  data[3] = voltageByte2;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = calculateXOR(data, 6);
}

// Send voltage via BLE
void sendBleVoltage() {
  if (!(b_ble_enabled && deviceConnected && pReadCharacteristic)) return;
  byte data[7];
  buildVoltagePacket(data);
  pReadCharacteristic->setValue(data, 7);
  pReadCharacteristic->notify();
}

// Build heartbeat packet
void buildHeartBeatPacket(byte data[7]) {
  data[0] = modelByte;
  data[1] = 0x0A;  // Heartbeat type
  data[2] = 0x03;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0x00;
  data[6] = 0x0A;  // Checksum (can also use calculateXOR)
}

// Send heartbeat via BLE
void sendBleHeartBeat() {
  if (!(b_ble_enabled && deviceConnected && pReadCharacteristic)) return;
  byte data[7];
  buildHeartBeatPacket(data);
  pReadCharacteristic->setValue(data, 7);
  pReadCharacteristic->notify();
}

#if defined(ACC_MPU6050) || defined(ACC_BMA400)
void buildGyroPacket(byte data[7]) {
  float gyro = gyro_z();
  byte gyroByte1, gyroByte2;
  encodeWeight(gyro, gyroByte1, gyroByte2);

  data[0] = modelByte;
  data[1] = 0x21;  // Gyro type
  data[2] = gyroByte1;
  data[3] = gyroByte2;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = calculateXOR(data, 6);
}

void sendBleGyro() {
  if (!(b_ble_enabled && deviceConnected && pReadCharacteristic)) return;
  byte data[7];
  buildGyroPacket(data);
  pReadCharacteristic->setValue(data, 7);
  pReadCharacteristic->notify();
}
#endif

void buildWeightPacket(byte data[7]) {
  float weight = f_displayedValue;
  byte weightByte1, weightByte2;
  encodeWeight(weight, weightByte1, weightByte2);

  data[0] = modelByte;
  data[1] = 0xCE;  // Weight type
  data[2] = weightByte1;
  data[3] = weightByte2;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = calculateXOR(data, 6);
}

void sendBleWeight() {
  if (!(b_ble_enabled && deviceConnected && pReadCharacteristic)) return;

  unsigned long currentMillis = millis();
  if (currentMillis - lastBleWeightNotifyTime < weightBleNotifyInterval) return;
  lastBleWeightNotifyTime = currentMillis;

  byte data[7];
  buildWeightPacket(data);
  pReadCharacteristic->setValue(data, 7);
  pReadCharacteristic->notify();
}

void buildButtonPacket(byte data[7], int buttonNumber, int buttonShortPress) {
  //buttonNumber 1 for button O, 2 for button[]
  //buttonShortPress 1 for short press, 2 for long press
  data[0] = modelByte;
  data[1] = 0xAA;  // Button byte
  data[2] = buttonNumber;
  data[3] = buttonShortPress;
  // 0 for release, 1 for click, 2 for long press
  // Fill the rest with dummy data or real data as needed
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = calculateXOR(data, 6);
}

void sendBleButton(int buttonNumber, int buttonShortPress) {
  if (!(b_ble_enabled && deviceConnected && pReadCharacteristic)) return;
  byte data[7];
  buildButtonPacket(data, buttonNumber, buttonShortPress);
  pReadCharacteristic->setValue(data, 7);
  pReadCharacteristic->notify();
}

// Build PowerOff packet into provided data array
void buildPowerOffPacket(byte data[7], int i_reason) {
  data[0] = modelByte;       // Model byte
  data[1] = 0x2A;            // Command ID for PowerOff

  // Initialize default values
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;

  // Fill reason codes
  switch (i_reason) {
    case 0: data[2] = 0x00; break; // Power off failed: disabled
    case 1: data[2] = 0x10; break; // "O" button double-click
    case 2: data[2] = 0x11; break; // "[]" button double-click
    case 3: data[3] = 0x20; break; // Low battery
#if defined(ACC_MPU6050) || defined(ACC_BMA400)
    case 4: data[2] = 0x30; break; // Power off from gyro
#endif
    default: data[2] = 0x00; break; // Invalid reason
  }

  // XOR checksum
  data[6] = calculateXOR(data, 6);
}

void sendBlePowerOff(int i_reason) {
  // Check BLE enabled, device connected, characteristic exists
  if (!(b_ble_enabled && deviceConnected && pReadCharacteristic)) return;

  byte data[7];
  buildPowerOffPacket(data, i_reason);

  // Send BLE notification
  pReadCharacteristic->setValue(data, 7);
  pReadCharacteristic->notify();
}


/*
Automatic firmware version extraction from LINE1 (FW: x.y.z)
BCD encoding for firmware version
Weight encoding using encodeWeight()
Charging detection (USB_DET or BATTERY_CHARGING)
Battery byte: 0xFF if charging, 0x64 (100%) otherwise
No XOR checksum
*/

// -----------------------------
// Build common LED response packet
// -----------------------------
void buildLedResponsePacket(byte data[7]) {
  int major = 0, minor = 0, patch = 0;

  // 1. Extract firmware version
    major = 0;
    minor = 0;
    patch = 0;

  // Convert to BCD format
  byte verHigh = (byte)(((major / 10) << 4) | (major % 10));
  byte verLow  = (byte)((minor << 4) | patch);

  // 2. Get current weight
  float weight = f_displayedValue;
  byte weightByte1, weightByte2;
  encodeWeight(weight, weightByte1, weightByte2);

  // 3. Detect charging status
  bool b_is_charging = false;

  // 4. Compute battery level
  byte batteryByte;
  if (b_is_charging) {
    batteryByte = 0xFF; // Charging
  }

  // 5. Fill packet
  data[0] = 0x03;         // Header
  data[1] = 0x0A;         // Type (LED response)
  data[2] = weightByte1;  // Weight high
  data[3] = weightByte2;  // Weight low
  data[4] = batteryByte;  // Battery / charging indicator
  data[5] = verHigh;      // Firmware version high
  data[6] = verLow;       // Firmware version low
}

void sendBleLedResponse() {
  // Check BLE enabled, device connected, characteristic exists
  if (!(b_ble_enabled && deviceConnected && pReadCharacteristic)) return;

  byte data[7];
  buildLedResponsePacket(data);
  pReadCharacteristic->setValue(data, 7);
  pReadCharacteristic->notify();
}


void setup() {
  delay(50);  //有些单片机会重启两次
  //some soc may reset twice
  Serial.begin(115200);

  ble_init();


  EEPROM.begin(512);
  //delay(2000);
  Serial.println("Begin!");

  unsigned long stabilizingtime = 500;  //去皮时间(毫秒)，增加可以提高去皮精确度
  //taring duration. longer for better reading.
  boolean _tare = true;  //电子秤初始化去皮，如果不想去皮则设为false
  //whether the scale will tare on start.
  scale.begin();
  scale.start(stabilizingtime, _tare);
  Serial.println("Scale started ...");
  //检查校准值合法性
  EEPROM.get(i_addr_calibration_value, f_calibration_value);
  if (isnan(f_calibration_value)) {
    f_calibration_value = 1.0;
  }
  scale.setCalFactor(f_calibration_value);  //设置偏移量
  scale.setSamplesInUse(16);  //设置灵敏度
  scale.tareNoDelay();
  Serial.println("Setup complete...");
}

void serialCommand() {
  if (Serial.available()) {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim();

    if (inputString.startsWith("c")) {
      Serial.println("***");
      Serial.println("Start calibration:");
      Serial.println("Place the load cell an a level stable surface.");
      Serial.println("Remove any load applied to the load cell.");
      Serial.println("Send 't' from serial monitor to set the tare offset.");

      boolean _resume = false;
      while (_resume == false) {
        scale.update();
        if (Serial.available() > 0) {
          if (Serial.available() > 0) {
            char inByte = Serial.read();
            if (inByte == 't') scale.tareNoDelay();
          }
        }
        if (scale.getTareStatus() == true) {
          Serial.println("Tare complete");
          _resume = true;
        }
      }

      Serial.println("Now, place your known mass on the loadcell.");
      Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

      float known_mass = 0;
      _resume = false;
      while (_resume == false) {
        scale.update();
        if (Serial.available() > 0) {
          known_mass = Serial.parseFloat();
          if (known_mass != 0) {
            Serial.print("Known mass is: ");
            Serial.println(known_mass);
            _resume = true;
          }
        }
      }

      scale.refreshDataSet();                                           //refresh the dataset to be sure that the known mass is measured correct
      float newCalibrationValue = scale.getNewCalibration(known_mass);  //get the new calibration value

      Serial.print("New calibration value has been set to: ");
      Serial.print(newCalibrationValue);
      Serial.println(", use this as calibration value (calFactor) in your project sketch.");
      Serial.print("Save this value to EEPROM? y/n");

      _resume = false;
      while (_resume == false) {
        if (Serial.available() > 0) {
          char inByte = Serial.read();
          if (inByte == 'y') {
            EEPROM.put(i_addr_calibration_value, newCalibrationValue);
            EEPROM.commit();
            EEPROM.get(i_addr_calibration_value, newCalibrationValue);
            Serial.print("Value ");
            Serial.print(newCalibrationValue);
            Serial.print(" saved to EEPROM");
            _resume = true;

          } else if (inByte == 'n') {
            Serial.println("Value not saved to EEPROM");
            _resume = true;
          }
        }
      }

      Serial.println("End calibration");
      Serial.println("***");
      Serial.println("To re-calibrate, send 'r' from serial monitor.");
      Serial.println("For manual edit of the calibration value, send 'c' from serial monitor.");
      Serial.println("***");
    }
  }
}


/**
 * Enhanced adaptive tracking system
 * Tracks both zero and stable weights to prevent oscillation
 */
void updateAdaptiveTracking(float current_weight) {
  unsigned long current_time = millis();
  
  if (!b_tracking_enabled) {
    return;
  }
  
  // Calculate weight difference from current tracking target
  float weight_diff = current_weight - f_tracking_target;
  
  // Check if weight is stable (within tracking threshold)
  if (fabs(weight_diff) <= TRACKING_THRESHOLD) {
    i_stable_count++;
    
    // Update tracking target to slowly follow stable weights
    if (i_stable_count >= 3) { // Start adjusting target after 3 stable readings
      float adjustment = weight_diff * 0.1; // Slow adaptation
      
      // Limit maximum adjustment to prevent large jumps
      if (fabs(adjustment) > MAX_TRACKING_ADJUSTMENT) {
        adjustment = (adjustment > 0) ? MAX_TRACKING_ADJUSTMENT : -MAX_TRACKING_ADJUSTMENT;
      }
      
      f_tracking_target += adjustment;
   }
    
  } else {
    // Weight changed significantly - likely a real weight change
    i_stable_count = 0;
    b_tracking_active = false;
    
    // If weight change is large and persistent, update tracking target
    if (fabs(weight_diff) > TRACKING_THRESHOLD * 2) {
      // Consider this as a new stable weight after verification
      if (verifyWeightStability(current_weight)) {
        f_tracking_target = current_weight;
        b_tracking_active = true;
        if (b_weight_in_serial) {
          Serial.print("New weight target set: ");
          Serial.println(f_tracking_target, 4);
        }
      }
    }
  }
  
  // Perform tracking adjustment when conditions are met
  if (i_stable_count >= i_STABLE_COUNT_THRESHOLD) {
    if (current_time - t_last_tracking_update >= TRACKING_UPDATE_INTERVAL) {
      performTrackingAdjustment(current_weight);
    }
  }
}

/**
 * Perform the actual tracking adjustment
 */
void performTrackingAdjustment(float current_weight) {
  float old_offset = f_tracking_offset;
  
  // Calculate new offset based on current weight and target
  float calculated_offset = current_weight - f_tracking_target;
  
  // Apply slow adaptation to prevent sudden changes
  f_tracking_offset = f_tracking_offset * 0.8 + calculated_offset * 0.2;
  
  // Activate tracking if not already active
  if (!b_tracking_active) {
    b_tracking_active = true;
  }
  
  // Debug output
  if (b_weight_in_serial) {
    Serial.print("Tracking adjustment: Offset ");
    Serial.print(old_offset, 4);
    Serial.print("g -> ");
    Serial.print(f_tracking_offset, 4);
    Serial.print("g | Target: ");
    Serial.print(f_tracking_target, 4);
    Serial.print("g | Raw: ");
    Serial.print(current_weight, 4);
    Serial.println("g");
  }
  
  // Reset counters
  i_stable_count = i_STABLE_COUNT_THRESHOLD - 2; // Keep near threshold for continuous tracking
  t_last_tracking_update = millis();
}

/**
 * Verify if a weight is stable enough to be considered a new target
 */
bool verifyWeightStability(float current_weight) {
  static float last_verified_weight = 0.0;
  static int verification_count = 0;
  
  if (fabs(current_weight - last_verified_weight) <= TRACKING_THRESHOLD) {
    verification_count++;
  } else {
    verification_count = 0;
  }
  
  last_verified_weight = current_weight;
  
  // Require 3 consecutive stable readings to verify new weight
  return (verification_count >= 3);
}

/**
 * Apply tracking compensation to raw weight
 */
float applyTrackingCompensation(float raw_weight) {
  if (b_tracking_active && b_tracking_enabled) {
    return raw_weight - f_tracking_offset;
  }
  return raw_weight;
}

/**
 * Apply stable output filtering
 * Returns the same value if change is below threshold
 */
float applyStableOutput(float current_value) {
  if (!b_stable_output_enabled) {
    return current_value; // Bypass stable filtering if disabled
  }
  
  float change = fabs(current_value - f_previous_stable_value);
  
  // If change is significant, update the stable value
  if (change >= STABLE_OUTPUT_THRESHOLD) {
    f_previous_stable_value = current_value;
    t_last_stable_change = millis();
    
    // Debug output for significant changes
    if (b_weight_in_serial) {
      Serial.print("Output updated: ");
      Serial.print(current_value, 4);
      Serial.print("g (Change: ");
      Serial.print(change, 4);
      Serial.println("g)");
    }
  }
  
  // Always return the stable value (may be same as previous)
  return f_previous_stable_value;
}

void pureScale() {
  static bool b_newDataReady = 0;
  static float f_last_displayed = 0.0;  // Last displayed value
  
  if (scale.update()) b_newDataReady = true;
  
  if (b_newDataReady) {
    float raw_weight = scale.getData();
   
    f_current_raw_value = raw_weight; // Store for status display
    
    // Continuous temperature drift detection and compensation
    // 1. Calculate difference between current raw and displayed value
    float current_diff = raw_weight - f_displayedValue - f_driftCompensation;
    
    // 2. If difference is small (0.01g-0.1g), accumulate to continuous compensation
    if (fabs(current_diff) > 0.01 && fabs(current_diff) < f_maxDriftCompensation) {
      static int f_similar_diff_count = 0;
      static float f_last_diff = current_diff;
      
      // Check if continuous same direction change
      if ((f_last_diff * current_diff) > 0) {  // Same direction
        f_similar_diff_count++;
        
        // If continuous micro changes, increase continuous compensation
        if (f_similar_diff_count >= 3) {
          // Increase continuous compensation (slowly)
          f_driftCompensation += current_diff * 0.3;  // Compensate 30% each time
          
          // Limit compensation range
          if (fabs(f_driftCompensation) > 2.0) {
            f_driftCompensation = (f_driftCompensation > 0) ? 2.0 : -2.0;
          }
          
          if (b_weight_in_serial) {
            Serial.print("TEMP-DRIFT-COMP: diff=");
            Serial.print(current_diff, 4);
            Serial.print("g, total_comp=");
            Serial.print(f_driftCompensation, 4);
            Serial.print("g, count=");
            Serial.println(f_similar_diff_count);
          }
          
          f_similar_diff_count = 2;  // Keep partial count for continued detection
        }
      } else {
        // Direction changed, reset
        f_similar_diff_count = 1;
      }
      
      f_last_diff = current_diff;
    } else {
      // Difference too large or too small, reset detection
      static int f_similar_diff_count = 0;
      f_similar_diff_count = 0;
    }
    
    // 3. Apply continuous temperature compensation
    float temperature_compensated = raw_weight - f_driftCompensation;
    
    // 4. Original processing pipeline
    float tracking_compensated = applyTrackingCompensation(temperature_compensated);
    float stable_output = applyStableOutput(tracking_compensated);
    
    if (stable_output >= -0.14 && stable_output <= 0.14) {
      f_displayedValue = 0.0;
    } else {
      // scale value is outside tolerance range, update displayed value
      f_displayedValue = stable_output;
    }
    // Update adaptive tracking (use temperature compensated value)
    updateAdaptiveTracking(tracking_compensated);
    
    // Display and debugging
    dtostrf(f_displayedValue, 7, 1, c_weight);
    if (b_weight_in_serial == true) {
      unsigned long current_time = millis();
      if (current_time - t_last_status_display >= STATUS_DISPLAY_INTERVAL) {
        Serial.println("=== Temperature Drift Status ===");
        Serial.print("Raw: ");
        Serial.print(raw_weight, 4);
        Serial.print("g | TempComp: ");
        Serial.print(f_driftCompensation, 4);
        Serial.print("g | AfterTempComp: ");
        Serial.print(temperature_compensated, 4);
        Serial.println("g");
        
        Serial.print("Displayed: ");
        Serial.print(f_displayedValue, 4);
        Serial.print("g | Raw-Display Diff: ");
        Serial.print(raw_weight - f_displayedValue, 4);
        Serial.println("g");
        
        displayEnhancedStatus(temperature_compensated, tracking_compensated, stable_output);
        t_last_status_display = current_time;
      }
    }
    
    b_newDataReady = false;
  }
  
  // Reset temperature compensation on TARE
  if (scale.getTareStatus()) {
    t_tareStatus = millis();
    b_weight_quick_zero = false;
    resetTracking();
    resetStableOutput();
    f_driftCompensation = 0.0;
    f_displayedValue = 0.0;
    if (b_weight_in_serial) {
      Serial.println("TARE: Temperature drift compensation reset");
    }
  }
  
  // Quick zero handling
  if (b_weight_quick_zero || b_bootTare) {
    f_displayedValue = 0.0;
    f_driftCompensation = 0.0;
  }
}

/**
 * Get current temperature compensation value
 */
float getTemperatureDriftCompensation() {
  return f_driftCompensation;
}

/**
 * Manually adjust temperature compensation
 */
void adjustTemperatureDriftCompensation(float amount) {
  f_driftCompensation += amount;
  Serial.print("Manual temp-comp adjust: ");
  Serial.print(amount, 4);
  Serial.print("g, total: ");
  Serial.println(f_driftCompensation, 4);
}

/**
 * Reset tracking system (for tare/zero operations)
 */
void resetTracking() {
  f_tracking_offset = 0.0;
  f_tracking_target = 0.0;
  i_stable_count = 0;
  b_tracking_active = false;
  t_last_tracking_update = millis();
  if (b_weight_in_serial) {
    Serial.println("Tracking system reset");
  }
}

/**
 * Reset stable output system
 */
void resetStableOutput() {
  f_previous_stable_value = 0.0;
  t_last_stable_change = millis();
  if (b_weight_in_serial) {
    Serial.println("Stable output reset");
  }
}

/**
 * Enable/disable stable output
 */
void setStableOutputEnabled(bool enabled) {
  b_stable_output_enabled = enabled;
  if (!enabled) {
    resetStableOutput();
  }
  Serial.print("Stable output ");
  Serial.println(enabled ? "enabled" : "disabled");
}

/**
 * Set stable output threshold
 */
void setStableOutputThreshold(float threshold) {
  STABLE_OUTPUT_THRESHOLD = threshold;
  Serial.print("Stable threshold set to: ");
  Serial.println(threshold, 4);
}

void setTrackingThreshold(float threshold) {
  TRACKING_THRESHOLD = threshold;
  Serial.print("Tracking threshold set to: ");
  Serial.println(threshold, 4);
}

void setTrackingUpdateInterval(float interval) {
  TRACKING_UPDATE_INTERVAL = interval;
  Serial.print("Tracking update interval set to: ");
  Serial.println(interval, 4);
}


/**
 * Enable/disable tracking system
 */
void setTrackingEnabled(bool enabled) {
  b_tracking_enabled = enabled;
  if (!enabled) {
    resetTracking();
  }
  Serial.print("Tracking system ");
  Serial.println(enabled ? "enabled" : "disabled");
}

/**
 * Enhanced status display with all system info
 */
void displayEnhancedStatus(float raw_weight, float compensated_weight, float stable_weight) {
  Serial.println("=== Enhanced Scale Status ===");
  Serial.print("Raw Input: ");
  Serial.print(raw_weight, 4);
  Serial.print("g | Compensated: ");
  Serial.print(compensated_weight, 4);
  Serial.print("g | Stable Output: ");
  Serial.print(stable_weight, 4);
  Serial.println("g");
  
  Serial.print("Stable Output: ");
  Serial.print(b_stable_output_enabled ? "ON" : "OFF");
  Serial.print(" | Threshold: ±");
  Serial.print(STABLE_OUTPUT_THRESHOLD, 4);
  Serial.println("g");
  
  Serial.print("Last Stable Change: ");
  Serial.print((millis() - t_last_stable_change) / 1000);
  Serial.println("s ago");
  
  // Tracking status
  Serial.print("Tracking System: ");
  Serial.print(b_tracking_enabled ? "ON" : "OFF");
  Serial.print(" | Active: ");
  Serial.println(b_tracking_active ? "YES" : "NO");
  
  Serial.print("Tracking Offset: ");
  Serial.print(f_tracking_offset, 4);
  Serial.print("g | Target: ");
  Serial.print(f_tracking_target, 4);
  Serial.println("g");
  
  Serial.print("Stable Count: ");
  Serial.print(i_stable_count);
  Serial.print("/");
  Serial.println(i_STABLE_COUNT_THRESHOLD);
  
  Serial.println("=============================");
}

/**
 * Get current tracking offset
 */
float getTrackingOffset() {
  return f_tracking_offset;
}

/**
 * Get current stable output value
 */
float getStableOutputValue() {
  return f_previous_stable_value;
}

// Optional: Manual control functions
/**
 * Manual override - set specific tracking offset
 */
void setManualTrackingOffset(float offset) {
  f_tracking_offset = offset;
  b_tracking_active = true;
  Serial.print("Manual tracking offset set: ");
  Serial.println(offset, 4);
}

/**
 * Manual override - set specific stable value
 */
void setManualStableValue(float value) {
  f_previous_stable_value = value;
  t_last_stable_change = millis();
  Serial.print("Manual stable value set: ");
  Serial.println(value, 4);
}


void loop() {
  
  if (bleState == CONNECTED && b_requireHeartBeat && millis() - t_firstConnect > HEARTBEAT_TIMEOUT) {
    long mill = millis();
    if (mill > t_heartBeat && mill - t_heartBeat > HEARTBEAT_TIMEOUT) {
      Serial.printf("Millis: %d Hearbeat %d\n", mill, t_heartBeat);
      disconnectBLE();
      t_heartBeat = millis() + 10000; //only disconnect after 10 seconds, avoid frequent disconnecting.
    } 
  }

if (b_tareByBle) {
          // Tare by BLE, performed instantly without delay
          scale.tareNoDelay();
          b_tareByBle = false;  // reset status
          Serial.println("Tare by BLE");
        }
  pureScale();
  serialCommand();
  sendBleWeight();
  unsigned long currentMillis = millis();
  if (currentMillis - lastWeightSerialTime >= weightSerialInterval) {
    // Save the last time you sent the weight notification
    lastWeightSerialTime = currentMillis;
    Serial.println(scale.getData());
  }
}
