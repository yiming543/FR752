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
//修正timeout bug

//20260727 FR752_V03 CS:5CE2
//修正接收CODE和顯示

//20260728 FR752_V04 CS:CA6B
//遠燈可以單獨點亮，配光用。

//20260731 FR752_V05 CS:AC21
//UNLOCK KEY 關閉遠燈

//20260821 FR752_V06 CS:ED23
//依據最近量測信號更新程式(LampControl_t)

//20260825
//新增2秒無正確信號，POS亮
//取消AutoBaud_Detect，因為和無線通訊有衝突，無線通訊會被AutoBaud_Detect()中斷，導致無法接收資料。
//P2是電源直接給電.
//方向燈是電源直接給電.
//Lobeam是電源直接給電.


#include "mcc_generated_files/LINDrivers/lin_slave.h"
#include "mcc_generated_files/mcc.h"
#define TURN_IN TURN_IN_PORT
#define TRUN_ON() DRL_ON
#define TRUN_OFF() DRL_OFF

/*
                         Main application
 */
// 啟動UART自動波特率偵測
void AutoBaud_Detect_ON(void) {
  BAUDCONbits.ABDOVF = 0;
  BAUDCONbits.ABDEN = 1;
  BAUDCONbits.WUE = 1;
}

// 設定通信波特率
void AutoBaud_Detect(void) {
  while (1) {
    AutoBaud_Detect_ON();
    while (!BAUDCONbits.ABDOVF) // wait for auto baud to complete
    {
      if (!BAUDCONbits.ABDEN) // if auto baud is disabled, break the loop
      {
        break;
      }
    }

    // 找到正確波特率,沒有超時離開無限迴圈
    if (!BAUDCONbits.ABDOVF) // if auto baud overflowed, try again
    {
      break;
    }
  }
}

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
    TMR2_HasOverflowOccured();
    TMR2_LoadPeriodRegister(0);
    TMR2_WriteTimer(0);
    TMR2_ReadTimer();
    TMR2_StopTimer();
    TMR2_StartTimer();
  } else if (i == 5) {
    PIN_MANAGER_IOC();
  } else if (i == 6) {
    DRL_ON();
    DRL_OFF();
  } else if (i == 7) {
    AutoBaud_Detect();
  }
}

int main(void) {
  // initialize the device
  SYSTEM_Initialize();

  // TMR0_Reload();
  // while (1) {
  //   if (RC7_GetValue() == 1) {
  //     if(INTCONbits.TMR0IF==1){
  //       break;
  //     }
  //   } else {
  //     TMR0_Reload();
  //     INTCONbits.TMR0IF;
  //   }
  // }
  // AutoBaud_Detect();

  // Enable the Global Interrupts
  INTERRUPT_GlobalInterruptEnable();
  // Enable the Peripheral Interrupts
  INTERRUPT_PeripheralInterruptEnable();

  Clear_No_Call_Warning();
  while (1) {
    // Add your application code
    LIN_handler();
    //2秒沒收到正確的信號
    if (LIN_signalTimeoutOccurred()) {
      POS_ON();
      HIBEAM_OFF();
      //P2亮,但P2是電源直接給電.
    }
    if (TURN_IN) {
      TRUN_ON();
    } else {
      TRUN_OFF();
    }
  }
  return 0;
}
/**
 End of File
*/
