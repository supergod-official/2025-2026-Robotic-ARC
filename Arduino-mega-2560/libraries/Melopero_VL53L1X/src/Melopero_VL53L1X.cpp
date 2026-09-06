//Author: Leonardo La Rocca

#include "Melopero_VL53L1X.h"

Melopero_VL53L1X::Melopero_VL53L1X(){

}

VL53L1_Error Melopero_VL53L1X::initI2C(uint8_t i2cAddr, TwoWire &bus){
    device = &dev;
    device->I2cDevAddr = i2cAddr;
    i2c_handle.i2c = &bus;
    device->I2cHandle = &i2c_handle;
    return VL53L1_ERROR_NONE;
}
/*
VL53L1_Error Melopero_VL53L1X::initSensor(){
    VL53L1_Error stat = 0;
    stat = softwareReset();
    getErrorDescription(stat);
    if (stat != VL53L1_ERROR_NONE) return stat;
    stat = waitDeviceBooted();
    getErrorDescription(stat);
    if (stat != VL53L1_ERROR_NONE) return stat;
    stat = dataInit();
    getErrorDescription(stat);
    if (stat != VL53L1_ERROR_NONE) return stat;
    stat = staticInit();
    getErrorDescription(stat);
    if (stat != VL53L1_ERROR_NONE) return stat;
    stat = setDistanceMode(VL53L1_DISTANCEMODE_MEDIUM);
    getErrorDescription(stat);
    if (stat != VL53L1_ERROR_NONE) return stat;
    stat = setMeasurementTimingBudgetMicroSeconds(66000);
    getErrorDescription(stat);
    if (stat != VL53L1_ERROR_NONE) return stat;
    stat = setInterMeasurementPeriodMilliSeconds(75);
    getErrorDescription(stat);
    return stat;
}*/
VL53L1_Error Melopero_VL53L1X::initSensor() {
    VL53L1_Error stat = 0;

    Serial.println("S1 softwareReset");
    stat = softwareReset();
    Serial.print("S1 done: "); Serial.println((int)stat);
    if (stat != VL53L1_ERROR_NONE) return stat;

    Serial.println("S2 waitDeviceBooted");
    stat = waitDeviceBooted();
    Serial.print("S2 done: "); Serial.println((int)stat);
    if (stat != VL53L1_ERROR_NONE) return stat;

    Serial.println("S3 dataInit");
    stat = dataInit();
    Serial.print("S3 done: "); Serial.println((int)stat);
    if (stat != VL53L1_ERROR_NONE) return stat;

    Serial.println("S4 staticInit");
    stat = staticInit();
    Serial.print("S4 done: "); Serial.println((int)stat);
    if (stat != VL53L1_ERROR_NONE) return stat;

    Serial.println("S5 setDistanceMode");
    stat = setDistanceMode(VL53L1_DISTANCEMODE_MEDIUM);
    Serial.print("S5 done: "); Serial.println((int)stat);
    if (stat != VL53L1_ERROR_NONE) return stat;

    Serial.println("S6 setTimingBudget");
    stat = setMeasurementTimingBudgetMicroSeconds(66000);
    Serial.print("S6 done: "); Serial.println((int)stat);
    if (stat != VL53L1_ERROR_NONE) return stat;

    Serial.println("S7 setInterMeasurement");
    stat = setInterMeasurementPeriodMilliSeconds(75);
    Serial.print("S7 done: "); Serial.println((int)stat);

    return stat;
}


//Device initialization functions
VL53L1_Error Melopero_VL53L1X::setDeviceAddress(uint8_t newAddress){
    VL53L1_Error status =  VL53L1_SetDeviceAddress(device, newAddress);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::dataInit(){
    VL53L1_Error status = VL53L1_DataInit(device);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::staticInit(){
    VL53L1_Error status = VL53L1_StaticInit(device);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::waitDeviceBooted(){
    VL53L1_Error status = VL53L1_WaitDeviceBooted(device);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::softwareReset(){
    VL53L1_Error status = VL53L1_software_reset(device);
    getErrorDescription(status);
    return status;
}

//Parameters
VL53L1_Error Melopero_VL53L1X::setPresetMode(VL53L1_PresetModes mode){
    VL53L1_Error status = VL53L1_SetPresetMode(device, mode);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::getPresetMode(){
    VL53L1_Error status = VL53L1_GetPresetMode(device, &presetMode);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::setDistanceMode(VL53L1_DistanceModes newDistanceMode){
    VL53L1_Error status = VL53L1_SetDistanceMode(device, newDistanceMode);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::getDistanceMode(){
    VL53L1_Error status = VL53L1_GetDistanceMode(device, &distanceMode);
    getErrorDescription(status);
    return status;
}

/*  Timing budget is the time required by the sensor to perform one range measurement. The minimum and maximum 
    timing budgets are [20 ms, 1000 ms] */
VL53L1_Error Melopero_VL53L1X::setMeasurementTimingBudgetMicroSeconds(uint32_t measurementTimingBudgetMicroSeconds){
    VL53L1_Error status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(device, measurementTimingBudgetMicroSeconds);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::getMeasurementTimingBudgetMicroSeconds(){
    VL53L1_Error status = VL53L1_GetMeasurementTimingBudgetMicroSeconds(device, &measurementTimingBudgetMicros);
    getErrorDescription(status);
    return status;
}

/*  Sets the inter-measurement period (the delay between two ranging operations) in milliseconds. The minimum 
    inter-measurement period must be longer than the timing budget + 4 ms.*/
VL53L1_Error Melopero_VL53L1X::setInterMeasurementPeriodMilliSeconds (uint32_t interMeasurementPeriodMilliSeconds){
    VL53L1_Error status = VL53L1_SetInterMeasurementPeriodMilliSeconds(device, interMeasurementPeriodMilliSeconds);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::getInterMeasurementPeriodMilliSeconds(){
    VL53L1_Error status = VL53L1_GetInterMeasurementPeriodMilliSeconds(device, &interMeasurementPeriodMillis);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::setUserROI(uint8_t topLeftX, uint8_t topLeftY, uint8_t botRightX, uint8_t botRightY){
    userROI.TopLeftX = topLeftX;
    userROI.TopLeftY = topLeftY;
    userROI.BotRightX = botRightX;
    userROI.BotRightY = botRightY;
    VL53L1_Error status = VL53L1_SetUserROI(device, &userROI);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::getUserROI(){
    VL53L1_Error status = VL53L1_GetUserROI(device, &userROI);
    getErrorDescription(status);
    return status;
}

//Ranging functions
VL53L1_Error Melopero_VL53L1X::startMeasurement(){
    VL53L1_Error status = VL53L1_StartMeasurement(device);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::stopMeasurement(){
    VL53L1_Error status = VL53L1_StopMeasurement(device);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::clearInterruptAndStartMeasurement(){
    VL53L1_Error status = VL53L1_ClearInterruptAndStartMeasurement(device);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::getMeasurementDataReady(){
    uint8_t isDataReady;
    VL53L1_Error status = VL53L1_GetMeasurementDataReady(device, &isDataReady);
    dataReady = isDataReady == 0? false : true;
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::waitMeasurementDataReady(){
    VL53L1_Error status = VL53L1_WaitMeasurementDataReady(device);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::getRangingMeasurementData(){
    VL53L1_Error status = VL53L1_GetRangingMeasurementData(device, &measurementData);
    getErrorDescription(status);
    return status;
}

//Interrupt functions
VL53L1_Error Melopero_VL53L1X::enableInterruptOnDataReady(){
    interruptConfig.DetectionMode = VL53L1_DETECTION_NORMAL_RUN;
    VL53L1_Error status = VL53L1_SetThresholdConfig(device, &interruptConfig);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::enableInterruptOnDistance(VL53L1_ThresholdMode mode, uint16_t high, uint16_t low){
    interruptConfig.DetectionMode = VL53L1_DETECTION_DISTANCE_ONLY;
    interruptConfig.Distance.CrossMode = mode;
    interruptConfig.Distance.High = high;
    interruptConfig.Distance.Low = low;
    VL53L1_Error status = VL53L1_SetThresholdConfig(device, &interruptConfig);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::getInterruptConfig(){
    VL53L1_Error status = VL53L1_GetThresholdConfig(device, &interruptConfig);
    getErrorDescription(status);
    return status;
}

VL53L1_Error Melopero_VL53L1X::getErrorDescription(VL53L1_Error error){
    return VL53L1_GetPalErrorString(error, errorString);
    return VL53L1_ERROR_NONE;
}
