/*
 * File:   gpio.c
 * Author: T00904
 *
 * Created on October 13, 2026, 10:48 AM
 */

#include "gpio.h"
#include "mcc_generated_files/mcc.h"

extern volatile uint16_t TurnOffCnt_1ms;
uint8_t fLightState = false;

void POS_ON(void) {
  fLightState = ePOS;
  EPWM1_LoadDutyValue(PWM_DUTY_10P);
}
void POS_OFF(void) {
  fLightState = eOFF;
  EPWM1_LoadDutyValue(PWM_DUTY_0P);
}
void DRL_ON(void) {
  fLightState = eDRL;
  if(TurnOffCnt_1ms==0)
    //方向燈關閉後1秒內，DRL不可以亮起
    EPWM1_LoadDutyValue(PWM_DUTY_100P);
}
void DRL_OFF(void) {
  fLightState = eOFF;
  EPWM1_LoadDutyValue(PWM_DUTY_0P);
}

// 方向燈改變燈條狀態，不動DRL/POS旗標
void TURN_IN_DRL_ON(void) { EPWM1_LoadDutyValue(PWM_DUTY_100P); }

// 方向燈改變燈條狀態，不動DRL/POS旗標
void TURN_IN_DRL_OFF(void) { EPWM1_LoadDutyValue(PWM_DUTY_0P); }

void HIBEAM_ON(void) { HiBeam_ON(); }
void HIBEAM_OFF(void) { HiBeam_OFF(); }
