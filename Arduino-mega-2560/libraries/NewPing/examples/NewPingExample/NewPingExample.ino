// ---------------------------------------------------------------------------
// Example NewPing library sketch that does a ping about 20 times per second.
// ---------------------------------------------------------------------------

#include <NewPing.h>

#define TRIGGER_PINf  18  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PINf     19  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCEf 40 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define TRIGGER_PINl  A9  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PINl     A8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCEl 20
//#define TRIGGER_PINr  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
//#define ECHO_PINr     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
//#define MAX_DISTANCEr 20
NewPing sonarf(TRIGGER_PINf, ECHO_PINf, MAX_DISTANCEf); // NewPing setup of pins and maximum distance.
NewPing sonarl(TRIGGER_PINl, ECHO_PINl, MAX_DISTANCEl);
void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
}

unsigned long previousMillis = 0;
const long interval = 29;  // 每 50 毫秒一次

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    float pingValuef = sonarf.ping();
    float pingValuel = sonarl.ping();
    float fd = pingValuef * 0.01717;
    float ld = pingValuel * 0.01717;
    
    Serial.println(fd); // 输出距离
    Serial.println(ld); // 输出距离
  }
}
