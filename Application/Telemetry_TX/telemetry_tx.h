/*
 * telemetry_tx.h
 *
 *  Created on: 15 Aug 2026
 *      Author: james
 */

#ifndef TELEMETRY_TX_H
#define TELEMETRY_TX_H

#define RADIO_TX_PERIOD_MS  5U   /* Check partial batches and radio completion. */
#define BATCH_MAX_AGE_MS   25U
#define CAN_QUEUE_SIZE     64U

typedef struct {
    CAN_Frame_t frame;
    uint32_t arrived;
} QueuedCAN;

static QueuedCAN queue[CAN_QUEUE_SIZE];
static uint8_t head;
static uint8_t queued;
static CAN_Frame_t batch[CAN_RF_BATCH_MAX];
static uint8_t inFlight;
static uint32_t doneBeforeSend;

/* Visible in the debugger. A full FIFO drops the newest arrival. */
static volatile uint32_t canQueueDrops;

void Telemetry_TX_Init(void);
void Telemetry_TX_Run(void);

#endif /* APPLICATION_USER_TELEMETRY_TX_TELEMETRY_TX_H_ */
