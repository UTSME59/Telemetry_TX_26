#include "radio_service.h"

#include "radio.h"
#include "subghz_phy_app.h"


#include "radio_driver.h"
volatile uint16_t radioIrqStatus = 0;

#include <stdio.h>
#include <string.h>

#define RADIO_TX_TIMEOUT_MS 3000U

static uint8_t txBuffer[CAN_RF_MAX_SIZE];

static volatile bool radioBusy = false;

void Radio_Init(void) {
	Radio.SetChannel(RF_FREQUENCY);

	Radio.SetTxConfig(
			MODEM_FSK, 					// GFSK
			TX_OUTPUT_POWER, 			// 14 dBm
			FSK_FDEV, 					// 25 kHz deviation
			FSK_BANDWIDTH,				// 50 kHz bandwidth
			FSK_DATARATE,  				// 50 kb/s
			0,							// Coding rate (unused)
			FSK_PREAMBLE_LENGTH,		// 5 bytes
			FSK_FIX_LENGTH_PAYLOAD_ON,  // Variable length
			true, 						// CRC enabled
			false, 						// Frequency hopping off
			0, 							// Hop period (unused)
			false, 						// IQ inversion (unused)
			RADIO_TX_TIMEOUT_MS
	);

	Radio.SetMaxPayloadLength(
			MODEM_FSK,
			CAN_RF_MAX_SIZE
	);

	printf("[RADIO] GFSK configured at 920 MHz, 50 kb/s\r\n");
}

bool Radio_SendCANBatch(const CAN_Frame_t *frames, uint8_t count)
{
    if (radioBusy || frames == NULL || count == 0U || count > CAN_RF_BATCH_MAX) {
    	return false;
    }

    uint8_t length = 0U;

    for (uint8_t i = 0U; i < count; ++i) {
        const CAN_Frame_t *f = &frames[i];

        if (f->dlc > 8U || f->id > (f->extended ? 0x1FFFFFFFUL : 0x7FFUL)) {
        	return false;
        }

        uint8_t *p = &txBuffer[length];
        p[0] = (uint8_t)f->id;
        p[1] = (uint8_t)(f->id >> 8);
        p[2] = (uint8_t)(f->id >> 16);
        p[3] = (uint8_t)(f->id >> 24);
        p[4] = f->dlc;
        p[5] = (f->extended ? 1U : 0U) | (f->rtr ? 2U : 0U);

        /* Preserve the existing 6 + DLC wire length for remote frames. */
        if (f->rtr) {
        	memset(p + 6, 0, f->dlc);
        } else {
            memcpy(p + 6, f->data, f->dlc);
    	}

        length += (uint8_t)(6U + f->dlc);
    }

    radioBusy = true;

    if (Radio.Send(txBuffer, length) != RADIO_STATUS_OK) {
        radioBusy = false;
        return false;
    }

    return true;
}

bool Radio_SendCAN(const CAN_Frame_t *frame)
{
    return Radio_SendCANBatch(frame, 1U);
}



void Radio_TxDone(void) {
	radioBusy = false;
	radioTxDoneCount++;
}

void Radio_TxTimeout(void) {
	radioBusy = false;
}

bool Radio_IsBusy(void) {
	return radioBusy;
}
