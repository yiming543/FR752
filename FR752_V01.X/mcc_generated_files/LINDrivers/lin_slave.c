/**
  LIN Slave Driver

  Company:
    Microchip Technology Inc.

  File Name:
    lin_slave.c

  Summary:
    LIN Slave Driver

  Description:
    This source file provides the driver for LIN slave nodes

 */

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#include "lin_slave.h"
#include "../eusart.h"
#include "../tmr0.h"
#include <string.h>

#define READ_TIMEOUT 20 // ms

static void (*LIN_processData)(void);

lin_packet_t LIN_packet;
bool LIN_rxInProgress = false;
const lin_rx_cmd_t *LIN_rxCommand;
uint8_t LIN_rxCommandLength;

static uint8_t LIN_timeout = 10; // TODO: Make dependent on Baudrate
static bool LIN_timerRunning = false;
static volatile uint8_t CountCallBack = 0;

void LIN_init(uint8_t tableLength, const lin_rx_cmd_t *const command,
              void (*processData)(void)) {
  LIN_rxCommand = command;
  LIN_rxCommandLength = tableLength;
  LIN_processData = processData;
  LIN_stopTimer();
  LIN_enableRx();
  LIN_setTimerHandler();
}

void LIN_queuePacket(uint8_t cmd) {
  const lin_rx_cmd_t *tempSchedule =
      LIN_rxCommand; // copy table pointer so we can modify it

  cmd &= 0x3F; // clear possible parity bits
  for (uint8_t i = 0; i < LIN_rxCommandLength; i++) {
    if (cmd == tempSchedule->cmd) {
      break;
    }
    tempSchedule++; // go to next entry
  }

  LIN_packet.type = tempSchedule->type;
  LIN_packet.length = tempSchedule->length;

  // Build Packet - User defined data
  // add data
  memcpy(LIN_packet.data, tempSchedule->data, LIN_packet.length);

  // Add Checksum
  LIN_packet.checksum =
      LIN_getChecksum(LIN_packet.length, LIN_packet.PID, LIN_packet.data);
  LIN_sendPacket(LIN_packet.length, LIN_packet.PID, LIN_packet.data);
}

lin_rx_state_t LIN_handler(void) {
  static lin_rx_state_t LIN_rxState = LIN_RX_IDLE;
  static uint8_t rxDataIndex = 0;

  if (LIN_rxInProgress == true) {
    if (LIN_timerRunning == false) {
      // Timeout
      LIN_rxState = LIN_RX_ERROR;
    }
  }

  switch (LIN_rxState) {
  case LIN_RX_IDLE:
    if (EUSART_is_rx_ready() > 0) {
      // Start Timer
      LIN_startTimer(READ_TIMEOUT);
      LIN_rxInProgress = true;
      LIN_rxState = LIN_RX_BREAK;
    }
    break;
  case LIN_RX_BREAK:
    if (EUSART_is_rx_ready() > 0) {
      if (LIN_breakCheck() == true) { // Read Break
        LIN_rxState = LIN_RX_SYNC;
      } else {
        LIN_rxState = LIN_RX_ERROR;
      }
    }
    break;
  case LIN_RX_SYNC:
    if (EUSART_is_rx_ready() > 0) {
      if (EUSART_Read() == 0x55) { // Read sync - discard
        LIN_rxState = LIN_RX_PID;
      } else {
        LIN_rxState = LIN_RX_ERROR;
      }
    }
    break;
  case LIN_RX_PID:
    if (EUSART_is_rx_ready() > 0) {
      LIN_packet.PID = EUSART_Read();

      // check LIN Parity bits
      if (LIN_checkPID(LIN_packet.PID) == false) {
        LIN_rxState = LIN_RX_ERROR;
        break;
      }
      LIN_packet.type = LIN_getFromTable(LIN_packet.PID, TYPE);
      if (LIN_packet.type == RECEIVE) {
        LIN_packet.length = LIN_getFromTable(LIN_packet.PID, LENGTH);
        LIN_rxState = LIN_RX_DATA;
      } else {
        LIN_disableRx();
        LIN_rxState = LIN_RX_TX_DATA;
      }
    }
    break;
  case LIN_RX_DATA:
    if (EUSART_is_rx_ready() > 0) {
      LIN_packet.data[rxDataIndex] = EUSART_Read();
      if (++rxDataIndex >= LIN_packet.length) {
        // received all data bytes
        rxDataIndex = 0;
        LIN_rxState = LIN_RX_CHECKSUM;
      }
    }
    break;
  case LIN_RX_CHECKSUM:
    if (EUSART_is_rx_ready() > 0) {
      LIN_packet.checksum = EUSART_Read();
      if (LIN_packet.checksum !=
          LIN_getChecksum(LIN_packet.length, LIN_packet.PID, LIN_packet.data)) {
        LIN_rxState = LIN_RX_ERROR;
      } else {
        LIN_rxState = LIN_RX_RDY;
      }
    }
    break;
  case LIN_RX_TX_DATA:
    LIN_queuePacket(LIN_packet.PID); // Send response automatically
    LIN_rxState = LIN_RX_RDY;
  case LIN_RX_RDY:
    LIN_processData();
  case LIN_RX_ERROR:
    LIN_stopTimer();
    rxDataIndex = 0;
    LIN_rxInProgress = false;
    memset(LIN_packet.rawPacket, 0,
           sizeof(LIN_packet.rawPacket)); // clear receive data
  case LIN_RX_WAIT:
    if (TXSTAbits.TRMT) {
      LIN_enableRx();
      LIN_rxState = LIN_RX_IDLE;
    } else {
      LIN_rxState = LIN_RX_WAIT;
    }
    break;
  }
  return LIN_rxState;
}

void LIN_sendPacket(uint8_t length, uint8_t pid, uint8_t *data) {

  // Write data
  for (uint8_t i = 0; i < length; i++) {
    EUSART_Write(*(data + i));
  }
  // Add Checksum
  EUSART_Write(LIN_getChecksum(length, pid, data));
}

uint8_t LIN_getPacket(uint8_t *data) {
  uint8_t cmd = LIN_packet.PID & 0x3F;

  memcpy(data, LIN_packet.data, sizeof(LIN_packet.data));
  memset(LIN_packet.rawPacket, 0, sizeof(LIN_packet.rawPacket));

  return cmd;
}

uint8_t LIN_getFromTable(uint8_t cmd, lin_sch_param_t param) {
  const lin_rx_cmd_t *rxCommand =
      LIN_rxCommand; // copy table pointer so we can modify it

  cmd &= 0x3F; // clear possible parity bits
  // check table
  for (uint8_t length = 0; length < LIN_rxCommandLength; length++) {
    if (cmd == rxCommand->cmd) {
      break;
    }
    rxCommand++; // go to next entry

    if (length == (LIN_rxCommandLength - 1)) {
      return ERROR; // command not in schedule table
    }
  }

  switch (param) {
  case CMD:
    return rxCommand->cmd;
  case TYPE:
    return rxCommand->type;
  case LENGTH:
    return rxCommand->length;
  default:
    break;
  }

  return ERROR;
}

bool LIN_checkPID(uint8_t pid) {
  if (LIN_getFromTable(pid, TYPE) == ERROR)
    return false; // PID not in schedule table

  if (pid == LIN_calcParity(pid & 0x3F))
    return true;

  return false; // Parity Error
}

uint8_t LIN_calcParity(uint8_t CMD) {
  lin_pid_t PID;
  PID.rawPID = CMD;

  // Workaround for compiler bug - CAE_MCU8-200:
  //    PID.P0 = PID.ID0 ^ PID.ID1 ^ PID.ID2 ^ PID.ID4;
  //    PID.P1 = ~(PID.ID1 ^ PID.ID3 ^ PID.ID4 ^ PID.ID5);
  PID.P0 = PID.ID0 ^ PID.ID1;
  PID.P0 = PID.P0 ^ PID.ID2;
  PID.P0 = PID.P0 ^ PID.ID4;
  PID.P1 = PID.ID1 ^ PID.ID3;
  PID.P1 = PID.P1 ^ PID.ID4;
  PID.P1 = PID.P1 ^ PID.ID5;
  PID.P1 = ~PID.P1;

  return PID.rawPID;
}

uint8_t LIN_getChecksum(uint8_t length, uint8_t pid, uint8_t *data) {
  uint16_t checksum = pid;

  for (uint8_t i = 0; i < length; i++) {
    checksum = checksum + *data++;
    if (checksum > 0xFF)
      checksum -= 0xFF;
  }
  checksum = ~checksum;

  return (uint8_t)checksum;
}

void LIN_startTimer(uint8_t timeout) {
  LIN_timeout = timeout;
  TMR0_WriteTimer(0);
  NOP();
  LIN_timerRunning = true;
}

void LIN_timerHandler(void) {

  // callback function
  if (++CountCallBack >= LIN_timeout) {
    // ticker function call
    // LIN_stopTimer();
    CountCallBack = 0;
    LIN_timerRunning = false;
  }
}

void LIN_setTimerHandler(void) { TMR0_SetInterruptHandler(LIN_timerHandler); }

void LIN_stopTimer(void) {
  NOP();
  // reset ticker counter
  CountCallBack = 0;
  LIN_timerRunning = false;
}

void LIN_enableRx(void) {
  RCSTAbits.CREN = 1;
  PIE1bits.RCIE = 1;
}

void LIN_disableRx(void) {
  RCSTAbits.CREN = 0;
  PIE1bits.RCIE = 0;
}

bool LIN_breakCheck(void) {

  if ((EUSART_Read() == 0x00) && (EUSART_get_last_status().ferr == 1)) {
    return true;
  }

  return false;
}