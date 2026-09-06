# Melopero_VL53L1X_Arduino_Library

**Warning:** This library is based on the STMicroelectronics API for the vl53l1x sensor which uses a meaningful amount of memory available on the Arduino boards. It is recommended to use this library on boards that have more memory such as the Arduino MKR. It won't compile/work on the Arduino UNO due to the high amount of memory needed.

## Usage

First you need to include the Melopero VL53L1X library and create the device object:  

```C++
#include "Melopero_VL53L1X.h"

Melopero_VL53L1X sensor; 

```

Than in the setup function you have to initialize the device:

```C++
Melopero_VL53L1X sensor;

void setup() {
    sensor.initDevice();
    ...
 }

 ```

The initDevice function configures the Wire library needed to communicate with the sensor and performs all the required reset/initialization functions.

### Sensor settings

You can set the sensor settings with the following functions:  

```C++
sensor.setDistanceMode(distanceMode);
// The distance mode can be one of the following:
// VL53L1_DISTANCEMODE_SHORT
// VL53L1_DISTANCEMODE_MEDIUM
// VL53L1_DISTANCEMODE_LONG

sensor.setMeasurementTimingBudgetMicroSeconds(micros);
// Timing budget is the time required by the sensor to perform 
// one range measurement. The minimum and maximum 
// timing budgets are [20 ms, 1000 ms] ([20'000 - 10'000'000]).

sensor.setInterMeasurementPeriodMilliSeconds(millis);
// Sets the inter-measurement period (the delay between two 
// ranging operations) in milliseconds. The minimum 
// inter-measurement period must be longer than the timing budget
// + 4 ms.

```

### Taking a measurement

There are two ways to take a measurement and read the result :
  * Starting a measurement and actively waiting for the result
  * Starting a measurement and wait for the interrupt when the data is ready

Method 1:

```C++
...

void setup(){
    ...
    sensor.startMeasurement();
}

void update(){
    sensor.waitMeasurementDataReady(); // wait for the data
    sensor.getRangingMeasurementData(); // get the data
    // the measurement data is stored in an instance variable:
    // sensor.measurementData.RangeMilliMeter
    Serial.println(sensor.measurementData.RangeMilliMeter);

    sensor.clearInterruptAndStartMeasurement(); // Starts a new measurement cycle
}

```

Method 2:

```C++
...

bool interruptOccurred = false;
byte intPin = 1;

void interruptCallback(){
    interruptOccurred = true;
}

void setup(){
    ...
    //interrupt code
    attachInterrupt(digitalPinToInterrupt(intPin), interruptCallback, CHANGE);

    sensor.enableInterruptOnDataReady();
    sensor.startMeasurement();
}

void update(){
    if (interruptOccurred){
        interruptOccurred = false;
        sensor.getRangingMeasurementData();
        // do stuff with data
        Serial.println(sensor.measurementData.RangeMilliMeter);
        sensor.clearInterruptAndStartMeasurement(); // Starts a new measurement cycle
    }
    // do other stuff
}

```

### Error messages

Each time a method from the library the instance variable `errorString` is set and the error code is returned.  
If a function returns the error code: `VL53L1_ERROR_NONE` there is no error and the errorString will be set to `"No Error"`.  
Otherwise an error occurred and you can get more information about the error by printing the `errorString`.  
Example:

```C++
VL53L1_ERROR status = VL53L1_ERROR_NONE;

status = sensor.setInterMeasurementPeriodMilliSeconds(75);
Serial.print("Set inter measurement time: ");
Serial.println(sensor.errorString);

```
