/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v4.1
processor: MIMXRT1062xxxxA
package_id: MIMXRT1062DVL6A
mcu_data: ksdk2_0
processor_version: 0.0.0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 * 
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 * 
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void) {
    BOARD_InitPins();
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: L14, peripheral: LPUART1, signal: RX, pin_signal: GPIO_AD_B0_13, software_input_on: Disable, hysteresis_enable: Disable, pull_up_down_config: Pull_Down_100K_Ohm,
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_6, slew_rate: Slow}
  - {pin_num: K14, peripheral: LPUART1, signal: TX, pin_signal: GPIO_AD_B0_12, software_input_on: Disable, hysteresis_enable: Disable, pull_up_down_config: Pull_Down_100K_Ohm,
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_6, slew_rate: Slow}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           /* iomuxc clock (iomuxc_clk_enable): 0x03u */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B0_12_LPUART1_TX,        /* GPIO_AD_B0_12 is configured as LPUART1_TX */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B0_13_LPUART1_RX,        /* GPIO_AD_B0_13 is configured as LPUART1_RX */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B0_12_LPUART1_TX,        /* GPIO_AD_B0_12 PAD functional properties : */
      0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B0_13_LPUART1_RX,        /* GPIO_AD_B0_13 PAD functional properties : */
      0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
#if 1
   IOMUXC_SetPinMux(
       IOMUXC_GPIO_SD_B0_00_LPSPI1_SCK,        /* GPIO_SD_B0_00 is configured as LPSPI1_SCK */
       0U);                                    /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
       IOMUXC_GPIO_SD_B0_01_LPSPI1_PCS0,       /* GPIO_SD_B0_01 is configured as LPSPI1_PCS0 */
       0U);                                    /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
       IOMUXC_GPIO_SD_B0_02_LPSPI1_SDO,        /* GPIO_SD_B0_02 is configured as LPSPI1_SDO */
       0U);                                    /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinMux(
       IOMUXC_GPIO_SD_B0_03_LPSPI1_SDI,        /* GPIO_SD_B0_03 is configured as LPSPI1_SDI */
       0U);                                    /* Software Input On Field: Input Path is determined by functionality */
   IOMUXC_SetPinConfig(
         IOMUXC_GPIO_SD_B0_00_LPSPI1_SCK,        /* GPIO_SD_B0_00 PAD functional properties : */
         0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                    Drive Strength Field: R0/6
                                                    Speed Field: medium(100MHz)
                                                    Open Drain Enable Field: Open Drain Disabled
                                                    Pull / Keep Enable Field: Pull/Keeper Enabled
                                                    Pull / Keep Select Field: Keeper
                                                    Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                    Hyst. Enable Field: Hysteresis Disabled */
     IOMUXC_SetPinConfig(
         IOMUXC_GPIO_SD_B0_01_LPSPI1_PCS0,       /* GPIO_SD_B0_01 PAD functional properties : */
         0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                    Drive Strength Field: R0/6
                                                    Speed Field: medium(100MHz)
                                                    Open Drain Enable Field: Open Drain Disabled
                                                    Pull / Keep Enable Field: Pull/Keeper Enabled
                                                    Pull / Keep Select Field: Keeper
                                                    Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                    Hyst. Enable Field: Hysteresis Disabled */
     IOMUXC_SetPinConfig(
         IOMUXC_GPIO_SD_B0_02_LPSPI1_SDO,        /* GPIO_SD_B0_02 PAD functional properties : */
         0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                    Drive Strength Field: R0/6
                                                    Speed Field: medium(100MHz)
                                                    Open Drain Enable Field: Open Drain Disabled
                                                    Pull / Keep Enable Field: Pull/Keeper Enabled
                                                    Pull / Keep Select Field: Keeper
                                                    Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                    Hyst. Enable Field: Hysteresis Disabled */
     IOMUXC_SetPinConfig(
         IOMUXC_GPIO_SD_B0_03_LPSPI1_SDI,        /* GPIO_SD_B0_03 PAD functional properties : */
         0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                    Drive Strength Field: R0/6
                                                    Speed Field: medium(100MHz)
                                                    Open Drain Enable Field: Open Drain Disabled
                                                    Pull / Keep Enable Field: Pull/Keeper Enabled
                                                    Pull / Keep Select Field: Keeper
                                                    Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                    Hyst. Enable Field: Hysteresis Disabled */
#endif
    // For Kalyke SPI
    IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_15_LPSPI3_SCK,        /* GPIO_SD_B0_00 is configured as LPSPI1_SCK */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B0_03_LPSPI3_PCS0,       /* GPIO_SD_B0_01 is configured as LPSPI1_PCS0 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_14_LPSPI3_SDO,        /* GPIO_SD_B0_02 is configured as LPSPI1_SDO */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_13_LPSPI3_SDI,        /* GPIO_SD_B0_03 is configured as LPSPI1_SDI */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B1_15_LPSPI3_SCK,        /* GPIO_SD_B0_00 PAD functional properties : */
      0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
    IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B0_03_LPSPI3_PCS0,       /* GPIO_SD_B0_01 PAD functional properties : */
      0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
    IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B1_14_LPSPI3_SDO,        /* GPIO_SD_B0_02 PAD functional properties : */
      0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
    IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B1_13_LPSPI3_SDI,        /* GPIO_SD_B0_03 PAD functional properties : */
      0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */


}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
