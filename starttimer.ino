#include <ADCTouchSensor.h>
#define USE_RTC

#ifdef USE_RTC
//#include <RTClock.h>
//#include <libmaple/rcc.h>
#include "safe_rtc.h"
#define SAFE_TIME_SEC (60*30-10)
#define CLOCK_MIN_SEC (10*60) // used to detect if clock lost power
#endif

#define SPEAKER PB11
#define DELAY 30000

int pins[] = {PA0,PA1,PA2,PA3,PA4,PA5,PA6,PA7};
const int numPins = sizeof(pins)/sizeof(*pins);
ADCTouchSensor* sensors[numPins];

#define LED PC13

uint32 t0;
bool stopped;
bool silent;
#ifdef USE_RTC
bool rtc;
#endif

void setup() {
  Serial.begin();
  while(!Serial);
  Serial.println("hello");
  pinMode(LED, OUTPUT);
  pinMode(PB12, OUTPUT);
  digitalWrite(LED, 0);
  t0 = 0;  
  stopped = false;
  silent = false;
  for (int i=0; i<numPins; i++) {
      sensors[i] = new ADCTouchSensor(pins[i]);
      sensors[i]->begin();
  }

#ifdef USE_RTC
  rtc = false;
  Serial.println("rtc initializing");
  if (!safe_rtc_start_lse()) {
    Serial.println("failed to start");
    return;
  }
  Serial.print("count ");
  uint32_t curT = safe_rtc_get_count(&rtc);
  if (!rtc)
    return;
  Serial.println(curT);
  if (curT >= CLOCK_MIN_SEC && curT <= CLOCK_MIN_SEC+SAFE_TIME_SEC) {
    for (int i=0;i<7;i++) {
      digitalWrite(LED,1);
      delay(50);
      digitalWrite(LED,0);
      delay(50);
    }
    
    stopped = true;
  }
  else {
    safe_rtc_set_count(0);
  }
#endif    
}

bool checkForExit(uint32 dt) {
  uint32 t0 = millis();
  do {
    if (millis() >= DELAY) {
      stopped = true;
      return true;
    }
    if (!silent) {
      for (int i=0; i<numPins; i++) {
        if (sensors[i]->read() > 100) {
          silent = true;
          tone(SPEAKER, 0);
          break;
        }
      }
    }
  } while(millis() - t0 < dt);
  return false;
}

void loop() {
  if (stopped) {
    tone(SPEAKER, 0);
    digitalWrite(SPEAKER, 0);
    digitalWrite(LED, 1);
#ifdef USE_RTC    
    if (rtc)
      safe_rtc_set_count(CLOCK_MIN_SEC);
#endif    
    delay(5000);
    return;
  }
  uint32_t delta = millis() - t0;
  if (delta >= DELAY) {
    stopped = true;
    return;
  }
  digitalWrite(LED, 0);
  tone(SPEAKER, silent ? 0 : ( (800 * delta + 1500 * (DELAY-delta)) / DELAY ));
  if (checkForExit(250))
    return;
  tone(SPEAKER, 0);
  digitalWrite(LED, 1);
  if (checkForExit(250)) 
    return;
}

