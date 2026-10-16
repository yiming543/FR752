/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules
  selected in the GUI. Generation Information : Product Revision  :  PIC10 /
  PIC12 / PIC16 / PIC18 MCUs - 1.81.8 Device            :  PIC16F1936 Driver
  Version    :  2.00
*/

// 20251125 V01 CS:2EC1
// LIN ==> HiBeam
// LIN ==> DRL 100%
// LIN ==> POS  10%
// TURN_IN ==> TURN_ON

// 20251226 V02 CS:9830
// DRL/POS/LED OFF/HiBeam OK
// 修正timeout bug

// 20260727 FR752_V03 CS:5CE2
// 修正接收CODE和顯示

// 20260728 FR752_V04 CS:CA6B
// 遠燈可以單獨點亮，配光用。

// 20260731 FR752_V05 CS:AC21
// UNLOCK KEY 關閉遠燈

// 20260821 FR752_V06 CS:ED23
// 依據最近量測信號更新程式(LampControl_t)

// 20260825
// 新增2秒無正確信號，POS亮
// 取消AutoBaud_Detect，因為和收不到信號有衝突，開機時斷線(沒接線)無法達到收不到信號亮近燈和POS功能。
// P2是電源直接給電.
// 方向燈是電源直接給電.
// Lobeam是電源直接給電.
// 只能控制Hibeam和光條(DRL/POS)。

// 20261013 FR750_V07 CS:5AEC
// 方向燈 ON，光條亮度100%，增加亮度(不論原本是在OFF/DRL/POS)。
// 方向燈 OFF一秒內DRL不可以亮起。
// 方向燈 OFF一秒內POS正常亮，超過一秒也維持在POS。
// 有lock信號(01 08)時,所有LED OFF
// 有time out信號(31 08)時,所有LED OFF
// 有unlock信號(01 XX)時,POS ON.
// 新增gpio.c和gpio.h

#include "gpio.h"
#include "mcc_generated_files/LINDrivers/lin_slave.h"
#include "mcc_generated_files/mcc.h"

extern volatile uint16_t TurnOffCnt_1ms;

// 消除未呼叫警告 假裝有使用這些函數
void Clear_No_Call_Warning(void) {
  uint8_t i = 0;
  if (i == 1) {
    TMR0_Reload();
    TMR0_ReadTimer();
  } else if (i == 2) {
    PIN_MANAGER_IOC();
  } else if (i == 3) {
    EUSART_get_last_status();
    EUSART_is_tx_done();
    EUSART_is_tx_ready();
  } else if (i == 4) {
    //    TMR2_HasOverflowOccured();
    TMR2_LoadPeriodRegister(0);
    TMR2_WriteTimer(0);
    TMR2_ReadTimer();
    TMR2_StopTimer();
    TMR2_StartTimer();
  } else if (i == 5) {
    DRL_OFF();
  } else if (i == 6) {
  } else if (i == 7) {
  }
}

int main(void) {
  // initialize the device
  SYSTEM_Initialize();

  // Enable the Global Interrupts
  INTERRUPT_GlobalInterruptEnable();
  // Enable the Peripheral Interrupts
  INTERRUPT_PeripheralInterruptEnable();

  Clear_No_Call_Warning();

  while (1) {
    // Add your application code
    LIN_handler();
    // 2秒沒收到正確的信號
    if (LIN_signalTimeoutOccurred()) {
      POS_ON();
      HIBEAM_OFF();
      // P2亮,但P2是電源直接給電.
    }

    if (TURN_IN) {
      TurnOffCnt_1ms = 1000; //1ms*1000 = 1秒
      TURN_IN_DRL_ON();
    } else if ((TurnOffCnt_1ms != 0) && (fLightState != ePOS)) {
      // 方向燈OFF1秒內DRL不可以亮起.
      TURN_IN_DRL_OFF();
    }
  }
  return 0;
}
/**
 End of File
**/
