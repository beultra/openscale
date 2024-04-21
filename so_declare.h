#ifndef DECLARE_H
#define DECLARE_H
#include "so_config.h"

#include <HX711_ADC.h>
HX711_ADC scale(HX711_SDA, HX711_SCL);  //HX711模数转换初始化
CoffeeData coffeeData;


#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
BLEServer *pServer = NULL;
BLECharacteristic *pReadCharacteristic = NULL;
BLECharacteristic *pWriteCharacteristic = NULL;
bool deviceConnected = false;

// The model byte is always 03 for Decent scales
const byte modelByte = 0x03;

#endif