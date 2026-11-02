/*
 * radio_board_if.c
 *
 * Custom STM32WL55CCU7 telemetry board.
 * RF switch: Infineon BGS12P2L6
 *
 * PA8 LOW  -> RF1 -> RX
 * PA8 HIGH -> RF2 -> RFO_HP TX
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file is provided, it is provided AS-IS.
 */

#include "radio_board_if.h"
#include "main.h"
#include "stm32wlxx_hal.h"

#if defined(USE_BSP_DRIVER)
#error "Custom telemetry board: remove USE_BSP_DRIVER"
#endif

#define RF_SWITCH_TX_LEVEL GPIO_PIN_SET
#define RF_SWITCH_RX_LEVEL GPIO_PIN_RESET

int32_t RBI_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Set the output latch before enabling the output. */
    HAL_GPIO_WritePin(RF_SW_CTRL_GPIO_Port,
                      RF_SW_CTRL_Pin,
                      RF_SWITCH_RX_LEVEL);

    gpio.Pin = RF_SW_CTRL_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(RF_SW_CTRL_GPIO_Port, &gpio);

    return 0;
}

int32_t RBI_DeInit(void)
{
    /*
     * Keep CTRL driven low so the switch does not float.
     * This parks the antenna on RX.
     */
    HAL_GPIO_WritePin(RF_SW_CTRL_GPIO_Port,
                      RF_SW_CTRL_Pin,
                      RF_SWITCH_RX_LEVEL);

    return 0;
}

int32_t RBI_ConfigRFSwitch(RBI_Switch_TypeDef Config)
{
    switch (Config)
    {
        case RBI_SWITCH_RFO_HP:
            HAL_GPIO_WritePin(RF_SW_CTRL_GPIO_Port,
                              RF_SW_CTRL_Pin,
                              RF_SWITCH_TX_LEVEL);
            return 0;

        case RBI_SWITCH_RX:
        case RBI_SWITCH_OFF:
            /* The switch has no separate all-paths-off state. */
            HAL_GPIO_WritePin(RF_SW_CTRL_GPIO_Port,
                              RF_SW_CTRL_Pin,
                              RF_SWITCH_RX_LEVEL);
            return 0;

        case RBI_SWITCH_RFO_LP:
        default:
            /* RFO_LP is not connected on this PCB. */
            return -1;
    }
}

int32_t RBI_GetTxConfig(void)
{
    /* Always use the physically connected high-power output. */
    return RBI_CONF_RFO_HP;
}

int32_t RBI_IsTCXO(void)
{
    /* This board uses a crystal, not a TCXO. */
    return 0;
}

int32_t RBI_IsDCDC(void)
{
    /* External SMPS inductor/capacitors are fitted. */
    return 1;
}

int32_t RBI_GetRFOMaxPowerConfig(
    RBI_RFOMaxPowerConfig_TypeDef Config)
{
    /*
     * PA configuration values, not the requested TX power.
     * RFO_LP is unused because RBI_GetTxConfig() forces HP.
     */
    switch (Config)
    {
        case RBI_RFO_LP_MAXPOWER:
            return 15;

        case RBI_RFO_HP_MAXPOWER:
            return 22;

        default:
            return -1;
    }
}
