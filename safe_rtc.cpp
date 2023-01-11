#include <Arduino.h>
#include <libmaple/rcc.h>
#define TIMEOUT 4000
#include <RTClock.h>

#define DEBUG

bool safe_rtc_wait_sync() {
  uint32_t t0 = millis();
  while (*bb_perip(&(RTC->regs)->CRL, RTC_CRL_RSF_BIT) == 0) {
    if (millis()-t0 >= TIMEOUT) {
#ifdef DEBUG
      Serial.println("timed out wait_sync");
#endif            
      
      return false;
    }
  }
  return true;
}

bool safe_rtc_wait_finished() {
  uint32_t t0 = millis();
  while (*bb_perip(&(RTC->regs)->CRL, RTC_CRL_RTOFF_BIT) == 0) {
    if (millis()-t0 > TIMEOUT) {
      return false;
    }
  }
  return true;
}

bool safe_rtc_clear_sync() {
  if (!safe_rtc_wait_finished())
    return false;
  *bb_perip(&(RTC->regs)->CRL, RTC_CRL_RSF_BIT) = 0;
  return true;
}

bool safe_rtc_enter_config_mode() {
  if (!safe_rtc_wait_finished())
    return false;
  *bb_perip(&(RTC->regs)->CRL, RTC_CRL_CNF_BIT) = 1;
  return true;
}

bool safe_rtc_exit_config_mode() {
  if (!safe_rtc_wait_finished())
    return false;
  *bb_perip(&(RTC->regs)->CRL, RTC_CRL_CNF_BIT) = 0;
  return true;
}

bool safe_rtc_set_prescaler_load(uint32 value) {
  if (!safe_rtc_clear_sync() ||
      !safe_rtc_wait_sync() ||
      !safe_rtc_wait_finished() ||
      !safe_rtc_enter_config_mode()) {
#ifdef DEBUG
    Serial.println("failed in set_prescaler_load");
#endif            
    return false;
      }
  RTC->regs->PRLH = (value >> 16) & 0xffff;
  RTC->regs->PRLL = value & 0xffff;
  return safe_rtc_exit_config_mode() &&
    safe_rtc_wait_finished();
}

void rtc_enable_lse(void) {
  RCC_BASE->BDCR |= RCC_BDCR_RTCSEL_LSE;
  bb_peri_set_bit(&RCC_BASE->BDCR, RCC_BDCR_RTCEN_BIT, 1); // Enable the RTC  
}

bool safe_rtc_start_lse(void) {
  bkp_init();
  bkp_enable_writes();
  RCC_BASE->BDCR &= ~RCC_BDCR_RTCSEL;
  bb_peri_set_bit(&RCC_BASE->BDCR, RCC_BDCR_LSEBYP_BIT, 0);
  bb_peri_set_bit(&RCC_BASE->BDCR, RCC_BDCR_LSEON_BIT, 1);
  uint32_t t0 = millis();
  while (bb_peri_get_bit(&RCC_BASE->BDCR, RCC_BDCR_LSERDY_BIT ) == 0) {
     if (millis()-t0 >= TIMEOUT) {
#ifdef DEBUG
         Serial.println("timed out waiting for LSERDY");
#endif            
         return false;
     }
  }
  
  rtc_enable_lse();

  if (!safe_rtc_set_prescaler_load(0x7fff))
    return false;

  return safe_rtc_clear_sync() && 
    safe_rtc_wait_sync() && 
    safe_rtc_wait_finished();
}

uint32 safe_rtc_get_count(bool* successP) {
  uint32 h, l;
  if (!safe_rtc_clear_sync() ||
      !safe_rtc_wait_sync() ||
      !safe_rtc_wait_finished()) {    
    *successP = false;
    return 0;
  }
  
  do {
    h = RTC->regs->CNTH & 0xffff;
    l = RTC->regs->CNTL & 0xffff;
  } while (h != (RTC->regs->CNTH & 0xffff));

  *successP = true;
  return (h << 16) | l;
}

bool safe_rtc_set_count(uint32 value) {
  if (! safe_rtc_clear_sync() ||
      ! safe_rtc_wait_sync() ||
      ! safe_rtc_wait_finished() ||
      ! safe_rtc_enter_config_mode())
     return false;
  RTC->regs->CNTH = (value >> 16) & 0xffff;
  RTC->regs->CNTL = value & 0xffff;
  return safe_rtc_exit_config_mode() && safe_rtc_wait_finished();
}

