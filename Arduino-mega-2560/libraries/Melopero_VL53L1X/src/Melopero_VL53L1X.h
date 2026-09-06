//Author: Leonardo La Rocca
#ifndef Melopero_VL53L1X_H_INCLUDED
#define Melopero_VL53L1X_H_INCLUDED

#include "Arduino.h"
#include "Wire.h"
#include "api/vl53l1_api.h"
//#include "api/vl53l1_error_codes.h"
//#include "api/vl53l1_error_strings.h"
//#include "api/vl53l1_platform_user_data.h"

#define DEFAULT_I2C_ADDRESS 0x29 //0x29

class Melopero_VL53L1X {
    //Members
    public:
        I2C_HandleTypeDef i2c_handle;
        VL53L1_Dev_t dev;
        VL53L1_DEV device;
        VL53L1_PresetModes presetMode;
        VL53L1_DistanceModes distanceMode;
        uint32_t measurementTimingBudgetMicros;
        uint32_t interMeasurementPeriodMillis;
        VL53L1_UserRoi_t userROI;
        VL53L1_RangingMeasurementData_t measurementData;
        bool dataReady;
        VL53L1_DetectionConfig_t interruptConfig;
        char errorString[VL53L1_MAX_STRING_LENGTH];

    //Methods
    public:
        Melopero_VL53L1X();

        //Device initialization functions
        VL53L1_Error initI2C(uint8_t i2cAddr = DEFAULT_I2C_ADDRESS, TwoWire &bus = Wire);
        VL53L1_Error initSensor();
        VL53L1_Error setDeviceAddress(uint8_t newAddress);
        VL53L1_Error dataInit();
        VL53L1_Error staticInit();
        VL53L1_Error waitDeviceBooted();
        VL53L1_Error softwareReset();

        //Parameters
        VL53L1_Error setPresetMode(VL53L1_PresetModes mode);
        VL53L1_Error getPresetMode();
        VL53L1_Error setDistanceMode(VL53L1_DistanceModes newDistanceMode);
        VL53L1_Error getDistanceMode();
        VL53L1_Error setMeasurementTimingBudgetMicroSeconds(uint32_t measurementTimingBudgetMicroSeconds);
        VL53L1_Error getMeasurementTimingBudgetMicroSeconds();
        VL53L1_Error setInterMeasurementPeriodMilliSeconds (uint32_t interMeasurementPeriodMilliSeconds);
        VL53L1_Error getInterMeasurementPeriodMilliSeconds ();
        VL53L1_Error setUserROI(uint8_t topLeftX, uint8_t topLeftY, uint8_t botRightX, uint8_t botRightY);
        VL53L1_Error getUserROI();

        //Ranging functions
        VL53L1_Error startMeasurement();
        VL53L1_Error stopMeasurement();
        VL53L1_Error clearInterruptAndStartMeasurement();
        VL53L1_Error getMeasurementDataReady();
        VL53L1_Error waitMeasurementDataReady();
        VL53L1_Error getRangingMeasurementData();

        //Interrupt functions
        VL53L1_Error enableInterruptOnDataReady();
        VL53L1_Error enableInterruptOnDistance(VL53L1_ThresholdMode mode, uint16_t high, uint16_t low);
        VL53L1_Error disableInterrupt();
        VL53L1_Error getInterruptConfig();

        VL53L1_Error getErrorDescription(VL53L1_Error error);
};
#endif // Melopero_VL53L1X_H_INCLUDED
