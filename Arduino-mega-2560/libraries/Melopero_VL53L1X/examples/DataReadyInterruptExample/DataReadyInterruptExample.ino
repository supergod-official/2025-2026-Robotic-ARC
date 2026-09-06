#include <Melopero_VL53L1X.h>

Melopero_VL53L1X sensor;

// Interrupt variables and callback
bool interruptOccurred = false;
byte intPin = 24;

void interruptCallback(){
    interruptOccurred = true;
}


void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Starting...");

  VL53L1_Error status = 0;
  Wire.begin(); // use Wire1.begin() to use I2C-1 
  sensor.initI2C(0x29, Wire); // use sensor.initI2C(0x29, Wire1); to use I2C-1
  
  status = sensor.initSensor();
  printStatus("Device initialized : ", status);
  
  status = sensor.setDistanceMode(VL53L1_DISTANCEMODE_MEDIUM);
  printStatus("Set distance mode : ", status);

  /*  Timing budget is the time required by the sensor to perform one range 
   *  measurement. The minimum and maximum timing budgets are [20 ms, 1000 ms] */
  status = sensor.setMeasurementTimingBudgetMicroSeconds(66000);
  printStatus("Set timing budget: ", status);

  /*  Sets the inter-measurement period (the delay between two ranging operations) in milliseconds. The minimum 
   *  inter-measurement period must be longer than the timing budget + 4 ms.*/
  status = sensor.setInterMeasurementPeriodMilliSeconds(75);
  printStatus("Set inter measurement time: ", status);

  //If the above constraints are not respected the status is -4: VL53L1_ERROR_INVALID_PARAMS
  
  // Set the intPin pin to listen for hardware interrupts
  attachInterrupt(digitalPinToInterrupt(intPin), interruptCallback, CHANGE);
  
  // Enable the data ready interrupts
  status = sensor.enableInterruptOnDataReady();
  printStatus("Enable interrupts: ", status);
  status = sensor.clearInterruptAndStartMeasurement();
  printStatus("Start measurement: ", status);
}

void loop() {
  VL53L1_Error status = 0;
  
  if (interruptOccurred){
    status = sensor.getRangingMeasurementData();
    if (status != VL53L1_ERROR_NONE) printStatus("Error in get measurement data: ",  status);

    status = sensor.clearInterruptAndStartMeasurement();
    if (status != VL53L1_ERROR_NONE) printStatus("Error in clear interrupts: ",  status);

    Serial.print((float)sensor.measurementData.RangeMilliMeter + (float)sensor.measurementData.RangeFractionalPart/256.0);
    Serial.println(" mm");
  }
}

void printStatus(String msg, VL53L1_Error status){
  Serial.print(msg);
  Serial.println(status);
}
