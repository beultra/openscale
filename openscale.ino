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

//#include "so_config.h"
#include "so_parameter.h"
//#include "so_power.h"
//#include "so_display.h"
//#include "so_menu.h"
//#include "so_boot_menu.h"
#include "so_declare.h"
//#include "so_wifi_ota.h"
//#include "so_espnow.h"
#include "so_config.h"

#define BTLOG_print(a...) Serial.print(a)
#define BTLOG_println(a...) Serial.println(a)

//#include <BluetoothSerial.h>
//BluetoothSerial SerialBT;

// #ifndef BT
// #ifdef DEBUG_BT
// #include <BluetoothSerial.h>
// BluetoothSerial SerialBT;
// #endif
// #endif

//functions 函数

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
    deviceConnected = true;
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

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

        
*/

  void onWrite(BLECharacteristic *pWriteCharacteristic) {
    BTLOG_print("Timer");
    BTLOG_print(millis());
    BTLOG_print(" onWrite counter:");
    BTLOG_println(i_onWrite_counter++);
    if (pWriteCharacteristic != nullptr) {                         // Check if the characteristic is valid
      size_t len = pWriteCharacteristic->getLength();              // Get the data length
      uint8_t *data = (uint8_t *)pWriteCharacteristic->getData();  // Get the data pointer

      // Optionally print the received HEX for verification or debugging
      BTLOG_print("Received HEX: ");
      for (size_t i = 0; i < len; i++) {
        if (data[i] < 0x10) {  // Check if the byte is less than 0x10
          BTLOG_print("0");    // Print a leading zero
        }
        BTLOG_print(data[i], HEX);  // Print the byte in HEX
      }
      BTLOG_println();  // New line for readability
      if (data[0] == 0x03) {
        if (data[1] == 0x0F) {
          if (validateChecksum(data, len)) {
            BTLOG_println("Valid checksum for tare operation. Taring");
          } else {
            BTLOG_println("Invalid checksum for tare operation.");
          }
          scale.tare();
        } else if (data[1] == 0x0A) {
          if (data[2] == 0x00) {
            BTLOG_println("LED off detected.");
          } else if (data[2] == 0x01) {
            BTLOG_println("LED on detected.");
          } else if (data[2] == 0x02) {
            BTLOG_println("Power off detected.");
            //shut_down_now_nobeep();
          }
        } else if (data[1] == 0x0B) {
          if (data[2] == 0x03) {
            BTLOG_println("Timer start detected.");

          } else if (data[2] == 0x00) {
            BTLOG_println("Timer stop detected.");

          } else if (data[2] == 0x02) {
            BTLOG_println("Timer zero detected.");
          }
        }
      }
    }
  }
};

//buttons

void cal() {
}

void setContainerWeight() {
}

void setup() {
  delay(50);  //有些单片机会重启两次
  //some soc may reset twice
  Serial.begin(115200);

  while (!Serial)
    ;
  delay(200);
  Serial.println("Hola!");
  BLEDevice::init("Decent Scale");

  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service
  BLEService *pService = pServer->createService(SUUID_DECENTSCALE);


  pWriteCharacteristic = pService->createCharacteristic(
    CUUID_DECENTSCALE_WRITE,
    BLECharacteristic::PROPERTY_WRITE);
  pWriteCharacteristic->setCallbacks(new MyCallbacks());
  // Create BLE Characteristic for RX
  pReadCharacteristic = pService->createCharacteristic(
    CUUID_DECENTSCALE_READ,
    BLECharacteristic::PROPERTY_READ
      | BLECharacteristic::PROPERTY_NOTIFY
    //| BLECharacteristic::PROPERTY_WRITE
    //| BLECharacteristic::PROPERTY_INDICATE
    //| BLECharacteristic::PROPERTY_BROADCAST
    //| BLECharacteristic::PROPERTY_WRITE_NR
  );

  pReadCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");


  EEPROM.begin(512);
  //delay(2000);
  Serial.println("Begin!");
  analogReadResolution(ADC_BIT);

  //power_off(15);

  i_button_cal_status = 1;  //校准状态归1
  //calibration status goes to 1
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
    b_f_calibration = true;  //让按钮进入校准状态3
    //put calibration to status 3
    //cal();  //无有效读取，进入校准模式
    //eeprom calibration value is not valid, go to calibration procedure.
  } else
    scale.setCalFactor(f_calibration_value);  //设定校准值

  //检查sample值合法性
  //check sample number is valid
  // EEPROM.get(i_addr_sample, i_sample);
  // if (isnan(i_sample)) {
  //   b_f_set_sample = true;
  //   i_sample = 0;  //读取失败 默认值为3 对应sample为8
  //   i_sample_step = 0;
  //   setSample();
  // }
  //重新校准
  //recalibration

  //灵敏度设置
  //sensitivity
  // if (digitalRead(BUTTON_SET) == LOW) {
  //   b_f_set_sample = true;
  //   i_sample_step = 0;
  //   setSample();
  // }

  //4按键模式时显示信息


 // f_calibration_value = 170200.0 / 168.04 * 814.3 / 168.04;
  scale.setCalFactor(f_calibration_value);  //设置偏移量
  //set the calibration value
  //scale.setSamplesInUse(sample[i_sample]);  //设置灵敏度
  scale.setSamplesInUse(16);  //设置灵敏度
  scale.tareNoDelay();
  Serial.println("Setup complete...");
}

void pureScale() {
  static boolean newDataReady = 0;
  static boolean scaleStable = 0;
  float f_weight_adc_raw = 0;
  if (scale.update()) newDataReady = true;
  if (newDataReady) {
    f_weight_adc = scale.getData();

    circularBuffer[bufferIndex] = f_weight_adc;
    bufferIndex = (bufferIndex + 1) % windowLength;
    // calculate moving average
    f_weight_smooth = 0;
    for (int i = 0; i < windowLength; i++) {
      f_weight_smooth += circularBuffer[i];
    }
    f_weight_smooth /= windowLength;
    if (!b_f_minus_container_button) {
      //自动减去容器重量
      if (f_weight_container >= 1 && f_weight_smooth >= f_weight_container - NegativeTolerance && f_weight_smooth <= f_weight_container + PositiveTolerance) {
        // calculate difference between scale value and container value
        f_weight_smooth = f_weight_smooth - f_weight_container;
        b_f_minus_container = true;
      } else
        b_f_minus_container = false;
    } else if (f_weight_container >= 1) {
      //手动减去容器重量
      f_weight_smooth = f_weight_smooth - f_weight_container;
      b_f_minus_container = true;
    }
    // print smoothed reading
    newDataReady = false;
    if (f_weight_smooth >= f_displayedValue - OledTolerance && f_weight_smooth <= f_displayedValue + OledTolerance) {
      // scale value is within tolerance range, do nothing
      // or weight is around 0, then set to 0.
      if (f_weight_smooth > -0.1 && f_weight_smooth < 0.1)
        f_displayedValue = 0.0;
    } else {
      // scale value is outside tolerance range, update displayed value
      f_displayedValue = f_weight_smooth;
      // print result to serial monitor
    }

    f_weight_before_input = f_displayedValue;

    //串口输出原始重量读数

    dtostrf(f_weight_adc, 7, i_decimal_precision, c_weight);
  }
  if (scale.getTareStatus()) {
    b_f_weight_quick_zero = false;
  }
  //记录咖啡粉时，将重量固定为0
  if (b_f_weight_quick_zero)
    f_displayedValue = 0.0;

  float ratio_temp = f_displayedValue / f_weight_dose;
  if (ratio_temp < 0)
    ratio_temp = 0.0;
  if (f_weight_dose < 0.1)
    ratio_temp = 0.0;
  dtostrf(ratio_temp, 7, i_decimal_precision, c_brew_ratio);
  // if (millis() - t_temp > 500) {
  //   // Serial.print("temperature:");
  //   Serial.print(f_filtered_temperature);
  //   Serial.print("\t");
  //   // Serial.print("weightCompensation:");
  //   Serial.print(f_weight_smooth);
  //   Serial.print("\t");
  //   // Serial.print("f_weight_smooth:");
  //   Serial.println(f_displayedValue);
  //   t_temp = millis();
  // }
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

void loop() {
  //Serial.println(getDoubleClickDelay());
  //power_off(15);  //power off after 15 minutes

  serialCommand();
  if (deviceConnected) {
    unsigned long currentMillis = millis();

    if (currentMillis - lastWeightNotifyTime >= weightNotifyInterval) {
      // Save the last time you sent the weight notification
      lastWeightNotifyTime = currentMillis;

      byte data[7];
      float weight = scale.getData();
      byte weightByte1, weightByte2;

      encodeWeight(weight, weightByte1, weightByte2);

      data[0] = modelByte;
      data[1] = 0xCE;  // Type byte for weight stable
      data[2] = weightByte1;
      data[3] = weightByte2;
      // Fill the rest with dummy data or real data as needed
      data[4] = 0x00;
      data[5] = 0x00;
      data[6] = calculateXOR(data, 6);  // Last byte is XOR validation

      pReadCharacteristic->setValue(data, 7);
      pReadCharacteristic->notify();
    }
  }

  pureScale();
  unsigned long currentMillis = millis();
  if (currentMillis - lastWeightSerialTime >= weightSerialInterval) {
    // Save the last time you sent the weight notification
    lastWeightSerialTime = currentMillis;
    Serial.println(c_weight);
  }
}