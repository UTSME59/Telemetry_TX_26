/*
 * telemetry_tx.c
 *
 *  Created on: 15 Aug 2026
 *      Author: james
 */

#include "telemetry_tx.h"

#include "canbus.h"
#include "radio_service.h"
#include "radio_driver.h"

#include "spi.h"
#include "usart.h"

#include "stm32_seq.h"
#include "stm32_timer.h"
#include "stm32wlxx_hal.h"
#include "utilities_def.h"

#include "main.h"

#include <stdio.h>

//#define TASK_LED_BLINK 		(1UL << CFG_SEQ_Task_LED_BLINK)
#define TASK_CAN_RX				(1UL << CFG_SEQ_Task_CAN_RX)
#define TASK_RADIO_TX 			(1UL << CFG_SEQ_Task_RADIO_TX)

#define LED_PERIOD_MS        	5000

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

static UTIL_TIMER_Object_t ledTimer;
static UTIL_TIMER_Object_t radioTxTimer;

static CAN_t CAN1;

static CAN_Config_t CAN1_Config =
{
	.hspi     = &hspi1,

	.cs_port  = CAN_CS_GPIO_Port,
	.cs_pin   = CAN_CS_Pin,

	.int_port = CAN_INT_GPIO_Port,
	.int_pin  = CAN_INT_Pin
};

static void LED_Blink_Task(void);
static void LED_Timer_Callback(void *context);

static void CAN_RX_Task(void);

static void RADIO_TX_Task(void);
static void RADIO_TX_Timer_Callback(void *context);

volatile CAN_Status_t canInitStatus = CAN_ERROR;

void Telemetry_TX_Init(void)
{
	//Not required due to initialisation in app_subghz_phy.c
	//UTIL_TIMER_Init();

	UTIL_SEQ_Init();

//	UTIL_SEQ_RegTask(TASK_LED_BLINK, UTIL_SEQ_RFU, LED_Blink_Task);
	UTIL_SEQ_RegTask(TASK_CAN_RX, UTIL_SEQ_RFU, CAN_RX_Task);
	UTIL_SEQ_RegTask(TASK_RADIO_TX, UTIL_SEQ_RFU, RADIO_TX_Task);

	CAN_Create(&CAN1, &CAN1_Config);

	canInitStatus = CAN1.Init(&CAN1);
	if (canInitStatus == CAN_OK) {
		const char message[] = "CAN INIT OK\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)message, sizeof(message)-1U, HAL_MAX_DELAY);
	}
	else
	{
		const char message[] = "CAN INIT FAILED\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)message, sizeof(message)-1U, HAL_MAX_DELAY);
	}

	UTIL_TIMER_Create(&ledTimer, LED_PERIOD_MS, UTIL_TIMER_ONESHOT, LED_Timer_Callback, NULL);
	UTIL_TIMER_Start(&ledTimer);

	UTIL_TIMER_Create(&radioTxTimer, RADIO_TX_PERIOD_MS, UTIL_TIMER_PERIODIC, RADIO_TX_Timer_Callback, NULL);
	UTIL_TIMER_Start(&radioTxTimer);

	if (canInitStatus == CAN_OK) {
		UTIL_SEQ_SetTask(TASK_CAN_RX, CFG_SEQ_Prio_0);
	}
}

void Telemetry_TX_Run(void)
{
	UTIL_SEQ_Run(UTIL_SEQ_DEFAULT);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == CAN_INT_Pin) {
		//HAL_GPIO_TogglePin(INT_LED_GPIO_Port, INT_LED_Pin);
		UTIL_SEQ_SetTask(TASK_CAN_RX, 0);
	}
}

static void LED_Timer_Callback(void *context)
{
	(void)context;
	//UTIL_SEQ_SetTask(TASK_LED_BLINK, 0);
}

static void LED_Blink_Task(void)
{
	//HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
	//CAN_PrintDiagnostics(&CAN1);
	UTIL_TIMER_Start(&ledTimer);
}

static void CAN_RX_Task(void)
{
    CAN_Frame_t frame;
    uint8_t reads = 0U;

    /* Yield periodically so a continuously busy CAN bus cannot hold this task. */
    while (reads < CAN_RF_BATCH_MAX && CAN1.Read(&CAN1, &frame) == CAN_OK) {
        ++reads;

        if (queued == CAN_QUEUE_SIZE) {
            ++canQueueDrops;
            continue;
        }

        uint8_t tail = (uint8_t)((head + queued) % CAN_QUEUE_SIZE);
        queue[tail].frame = frame;
        queue[tail].arrived = UTIL_TIMER_GetCurrentTime();
        ++queued;
    }

    if (queued >= CAN_RF_BATCH_MAX)
        UTIL_SEQ_SetTask(TASK_RADIO_TX, CFG_SEQ_Prio_0);

    /* MCP2515 INT can stay low while another frame is pending. */
    if (reads == CAN_RF_BATCH_MAX)
        UTIL_SEQ_SetTask(TASK_CAN_RX, CFG_SEQ_Prio_0);
}

static void RADIO_TX_Task(void)
{
    if (Radio_IsBusy())
        return;

    if (inFlight != 0U) {
        if (radioTxDoneCount != doneBeforeSend) {
            head = (uint8_t)((head + inFlight) % CAN_QUEUE_SIZE);
            queued -= inFlight;
        }
        /* On a radio timeout, the records are still at the FIFO head. */
        inFlight = 0U;
    }

    if (queued == 0U)
        return;

    if (queued < CAN_RF_BATCH_MAX &&
        UTIL_TIMER_GetElapsedTime(queue[head].arrived) < BATCH_MAX_AGE_MS)
        return;

    uint8_t count = queued < CAN_RF_BATCH_MAX ? queued : CAN_RF_BATCH_MAX;
    for (uint8_t i = 0U; i < count; ++i)
        batch[i] = queue[(head + i) % CAN_QUEUE_SIZE].frame;

    doneBeforeSend = radioTxDoneCount;
    if (Radio_SendCANBatch(batch, count))
        inFlight = count;
}

static void RADIO_TX_Timer_Callback(void *context) {
	(void)context;
	UTIL_SEQ_SetTask(TASK_RADIO_TX, CFG_SEQ_Prio_0);
}
