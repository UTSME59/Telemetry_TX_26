#ifndef RADIO_SERVICE_H
#define RADIO_SERVICE_H

#include "canbus.h"
#include <stdint.h>
#include <stdbool.h>

#define CAN_RF_BATCH_MAX 	10
#define CAN_RF_MAX_SIZE 	(CAN_RF_BATCH_MAX * 14)

void Radio_Init(void);
bool Radio_SendCAN(const CAN_Frame_t *frame);
bool Radio_SendCANBatch(const CAN_Frame_t *frames, uint8_t count);
void Radio_TxDone(void);
void Radio_TxTimeout(void);
bool Radio_IsBusy(void);

extern volatile uint32_t radioTxDoneCount;

#endif
