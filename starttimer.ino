#include <ADCTouchSensor.h>
#define USE_RTC
#define TWO_PINS

#ifdef USE_RTC
//#include <RTClock.h>
//#include <libmaple/rcc.h>
#include "safe_rtc.h"
#define SAFE_TIME_SEC (60*30-10)
#define CLOCK_MIN_SEC (10*60) // used to detect if clock lost power
#endif

#define SPEAKER PA3
#ifdef TWO_PINS
#define SPEAKER2 PA2
static unsigned channel;
static unsigned channel2;
static timer_dev* dev;

static void pwm_mode(timer_dev *dev, uint8 channel, bool reversed) {
    timer_disable_irq(dev, channel);
    timer_set_mode(dev,channel, TIMER_PWM );    
    timer_oc_set_mode(dev, channel, reversed?TIMER_OC_MODE_PWM_2:TIMER_OC_MODE_PWM_1, TIMER_OC_PE);
    timer_cc_enable(dev, channel);
}


#endif
#define DELAY 30000

int pins[] = {PA0,PA1,PA4,PA5,PA6,PA7,PA8,PA9};
const int numPins = sizeof(pins)/sizeof(*pins);
ADCTouchSensor* sensors[numPins];

#define LED PC13

uint32 t0;
bool stopped;
bool silent;
#ifdef USE_RTC
bool rtc;
#endif

void myTone(uint32_t frequency) {
  Serial.println(frequency);
#ifdef TWO_PINS
  timer_cc_disable(dev, channel);
  timer_cc_disable(dev, channel2);
  if (frequency==0) {
    return;
  }
  gpio_set_mode(PIN_MAP[SPEAKER].gpio_device, PIN_MAP[SPEAKER].gpio_bit, GPIO_AF_OUTPUT_PP);
  gpio_set_mode(PIN_MAP[SPEAKER2].gpio_device, PIN_MAP[SPEAKER2].gpio_bit, GPIO_AF_OUTPUT_PP);
  pwm_mode(dev, channel, true);
  pwm_mode(dev, channel2, false);
  
  uint32_t prescale = (CYCLES_PER_MICROSECOND*1000000ul / 128) / frequency;
  timer_set_prescaler(dev, prescale);
  timer_set_reload(dev, 64);
  timer_set_compare(dev, channel, 64);
  timer_set_compare(dev, channel2, 64);
#else
  tone(SPEAKER, frequency);
#endif
}

void disable() {
    pinMode(SPEAKER, OUTPUT);
    digitalWrite(SPEAKER, 0);
#ifdef TWO_PINS    
    pinMode(SPEAKER2, OUTPUT);
    digitalWrite(SPEAKER2, 0);  
#endif    
}

void setup() {
  Serial.begin();
  pinMode(LED, OUTPUT);
  pinMode(SPEAKER, OUTPUT);
  digitalWrite(SPEAKER,0);

#ifdef TWO_PINS  
  pinMode(SPEAKER2, OUTPUT);
  digitalWrite(SPEAKER2,0);
  dev = PIN_MAP[SPEAKER].timer_device;
  channel = PIN_MAP[SPEAKER].timer_channel;
  channel2 = PIN_MAP[SPEAKER2].timer_channel;  
#endif  
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
  if (curT >= CLOCK_MIN_SEC && curT <= CLOCK_MIN_SEC+SAFE_TIME_SEC) {
    for (int i=0;i<7;i++) {
      digitalWrite(LED,1);
      delay(50);
      digitalWrite(LED,0);
      delay(50);
    }

    Serial.println("stopping");
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
          myTone(0);
          break;
        }
      }
    }
  } while(millis() - t0 < dt);
  return false;
}

void loop() {
  Serial.print("stopped ");
  Serial.println(stopped);
  if (stopped) {
    myTone(0);
#ifdef TWO_PINS
    pinMode(SPEAKER,OUTPUT);
    pinMode(SPEAKER2,OUTPUT);
    digitalWrite(SPEAKER2, 0);
#endif    
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
  myTone(silent ? 0 : ( (800 * delta + 1500 * (DELAY-delta)) / DELAY ));
  if (checkForExit(250))
    return;
  myTone(0);
  digitalWrite(LED, 1);
  if (checkForExit(250)) 
    return;
}
