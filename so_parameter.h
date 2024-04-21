#ifndef PARAMETER_H
#define PARAMETER_H
//declaration

//ble
unsigned long lastWeightNotifyTime = 0;  // Stores the last time the weight notification was sent
const long weightNotifyInterval = 100;   // Interval at which to send weight notifications (milliseconds)
unsigned long lastWeightSerialTime = 0;  // Stores the last time the weight notification was sent
const long weightSerialInterval = 1000;   // Interval at which to send weight notifications (milliseconds)

float f_calibration_value;  //称重单元校准值
const int i_addr_calibration_value = 0;

#endif