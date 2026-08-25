/**
  LIN Slave Application

  Company:
    Microchip Technology Inc.

  File Name:
    lin_app.c

  Summary:
    LIN Slave Application

  Description:
    This source file provides the interface between the user and
    the LIN drivers.

 */

// *****************************************************************************
// Section: Included Files
// *****************************************************************************
#include <pic.h>
#include "lin_app.h"
#include "../epwm1.h"
#include "../pin_manager.h"

#define HiBeam_ON() HB_EN_SetHigh()
#define HiBeam_OFF() HB_EN_SetLow()

#define PWM_DUTY_0P 0
#define PWM_DUTY_10P (50)
#define PWM_DUTY_100P (500)

// 遠燈bit6,超車bit7
#define mHIBEAM 0B11000000 // 0x40 data[1]
// 左方向
#define mTURN_LEFT 0B00010000 // 0x10 data[1]
// 右方向
#define mTURN_RIGHT 0B00100000 // 0x20 data[1]

void POS_ON(void) { EPWM1_LoadDutyValue(PWM_DUTY_10P); }
void POS_OFF(void) { EPWM1_LoadDutyValue(PWM_DUTY_0P); }
void DRL_ON(void) { EPWM1_LoadDutyValue(PWM_DUTY_100P); }
void DRL_OFF(void) { EPWM1_LoadDutyValue(PWM_DUTY_0P); }
void HIBEAM_ON(void) { HiBeam_ON(); }
void HIBEAM_OFF(void) { HiBeam_OFF(); }


LampControl_t lampControl;
void LIN_Slave_Initialize(void)
{
  LIN_init(TABLE_SIZE, scheduleTable, processLIN);
}

void processLIN(void) // LIN接收資料處理
{
  uint8_t tempRxData[8];
  uint8_t cmd;

  cmd = LIN_getPacket(tempRxData);

  // ID NG return
  if (cmd != SIGNAL)
  {
    return;
  }

  if (tempRxData[0] == 0x24)
  {
    switch (tempRxData[2])
    {
    case eOffDayP:
      POS_OFF();
      break;

    case eOffDayN:
      POS_OFF();
      break;

    case eParkDayP:
      POS_ON();
      break;

    case eParkDayN:
      POS_ON();
      break;

    case eLobeamDayP:
      POS_ON();
      break;

    case eLobeamDayN:
      POS_ON();
      break;

    case eAutoDayP:
      POS_OFF();
      break;

    case eAutoDayN:
      DRL_ON();
      break;

    case eOffNightP:
      POS_OFF();
      break;

    case eOffNightN:
      POS_OFF();
      break;

    case eParkNightP:
      POS_ON();
      break;

    case eParkNightN:
      POS_ON();
      break;

    case eLobeamNightP:
      POS_ON();
      break;

    case eLobeamNightN:
      POS_ON();
      break;

    case eAutoNightP:
      POS_ON();
      break;

    case eAutoNightN:
      POS_ON();
      break;

    default:
      // 無法識別的資料，維持原本的狀態
      break;
    }

    if (tempRxData[1] & mHIBEAM)
    {
      HIBEAM_ON();
    }
    else
    {
      HIBEAM_OFF();
    }
  }
  else if ((tempRxData[0] == 0x31) && (tempRxData[1] == 0x08))
  { // LED OFF
    POS_OFF();
    HIBEAM_OFF();
  }
  else if ((tempRxData[0] == 0x31) && (tempRxData[1] == 0x0A))
  { // Ignition OFF，維持原本的狀態
  }
  else
  {
    // 無法識別的資料，維持原本的狀態
  }
}