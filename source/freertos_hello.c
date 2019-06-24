/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_lpspi_freertos.h"

#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"

#include "i2c_task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
    
#define ADS8668_CMD_NO_OP     0x00
#define ADS8668_CMD_STDBY     0x82
#define ADS8668_CMD_PWR_DN    0x83
#define ADS8668_CMD_RST       0x85
#define ADS8668_CMD_AUTO_RST  0xA0
#define ADS8668_CMD_MAN_CH0   0xC0
#define ADS8668_CMD_MAN_CH1   0xC4
#define ADS8668_CMD_MAN_CH2   0xC8
#define ADS8668_CMD_MAN_CH3   0xCC
#define ADS8668_CMD_MAN_CH4   0xD0
#define ADS8668_CMD_MAN_CH5   0xD4
#define ADS8668_CMD_MAN_CH6   0xD8
#define ADS8668_CMD_MAN_CH7   0xDC
#define ADS8668_CMD_MAN_AUX   0xE0
    

#define ADS8668_PRG_AUTO_SEQ_EN  0x01
#define ADS8668_PRG_CH_PWR_DN    0x02
#define ADS8668_PRG_FEATURE_SEL  0x03
#define ADS8668_PRG_RANGE_CH0    0x05

#define AUTO_SEQ_DN_CH0_MASK    0x01
#define AUTO_SEQ_DN_CH1_MASK    0x02
#define AUTO_SEQ_DN_CH2_MASK    0x04
#define AUTO_SEQ_DN_CH3_MASK    0x08
#define AUTO_SEQ_DN_CH4_MASK    0x10
#define AUTO_SEQ_DN_CH5_MASK    0x20
#define AUTO_SEQ_DN_CH6_MASK    0x40
#define AUTO_SEQ_DN_CH7_MASK    0x80

// For Kalyke project, Vref is 4096mV
#define INPUT_RANGE_POSITIVE_NEGATIVE_2_POINT_5_MULTIPLY_VREF     0x00 //¡À10.24V
#define INPUT_RANGE_POSITIVE_NEGATIVE_1_POINT_25_MULTIPLY_VREF    0x01 //¡À5.12V
#define INPUT_RANGE_POSITIVE_NEGATIVE_0_POINT_625_MULTIPLY_VREF   0x02 //¡À2.56V
#define INPUT_RANGE_POSITIVE_NEGATIVE_0_POINT_3125_MULTIPLY_VREF  0x03 //¡À1.28V
#define INPUT_RANGE_POSITIVE_NEGATIVE_0_POINT_15625_MULTIPLY_VREF 0x0B //¡À0.64V
#define INPUT_RANGE_POSITIVE_2_POINT_5_MULTIPLY_VREF              0x05 //0 ~ 10.24V
#define INPUT_RANGE_POSITIVE_1_POINT_25_MULTIPLY_VREF             0x06 //0 ~ 5.12V
#define INPUT_RANGE_POSITIVE_0_POINT_625_MULTIPLY_VREF            0x07 //0 ~ 2.56V
#define INPUT_RANGE_POSITIVE_0_POINT_3125_MULTIPLY_VREF           0x0F //0 ~ 1.28V



// The following program register no use for Kalyke project
#if 1
/* ALARM FLAG REGISTERS (Read-Only) */
#define ADS8668_PRG_TRIP_ALL     0x10 //ALARM Overview Tripped-Flag
#define ADS8668_PRG_TRIP0        0x11 //ALARM Ch 0-3 Tripped-Flag
#define ADS8668_PRG_ACTIVE0      0x12 //ALARM Ch 0-3 Active-Flag
#define ADS8668_PRG_TRIP1        0x13 //ALARM Ch 4-7 Tripped-Flag
#define ADS8668_PRG_ACTIVE1      0x14 //ALARM Ch 4-7 Active-Flag

/* ALARM THRESHOLD REGISTERS */
#define ADS8668_PRG_HYSTERESIS_CH0         0x15 //Ch 0 Hysteresis
#define ADS8668_PRG_HIGH_THRESHOLD_MSB_CH0 0x16
#define ADS8668_PRG_HIGH_THRESHOLD_LSB_CH0 0x17
#define ADS8668_PRG_LOW_THRESHOLD_MSB_CH0  0x18
#define ADS8668_PRG_LOW_THRESHOLD_LSB_CH0  0x19
    
#define ADS8668_PRG_HYSTERESIS_CH1         0x1A //Ch 1 Hysteresis
#define ADS8668_PRG_HIGH_THRESHOLD_MSB_CH1 0x1B
#define ADS8668_PRG_HIGH_THRESHOLD_LSB_CH1 0x1C
#define ADS8668_PRG_LOW_THRESHOLD_MSB_CH1  0x1D
#define ADS8668_PRG_LOW_THRESHOLD_LSB_CH1  0x1E
    
#define ADS8668_PRG_HYSTERESIS_CH2         0x1F //Ch 2 Hysteresis
#define ADS8668_PRG_HIGH_THRESHOLD_MSB_CH2 0x20
#define ADS8668_PRG_HIGH_THRESHOLD_LSB_CH2 0x21
#define ADS8668_PRG_LOW_THRESHOLD_MSB_CH2  0x22
#define ADS8668_PRG_LOW_THRESHOLD_LSB_CH2  0x23
    
#define ADS8668_PRG_HYSTERESIS_CH3         0x24 //Ch 3 Hysteresis
#define ADS8668_PRG_HIGH_THRESHOLD_MSB_CH3 0x25
#define ADS8668_PRG_HIGH_THRESHOLD_LSB_CH3 0x26
#define ADS8668_PRG_LOW_THRESHOLD_MSB_CH3  0x27
#define ADS8668_PRG_LOW_THRESHOLD_LSB_CH3  0x28
    
#define ADS8668_PRG_HYSTERESIS_CH4         0x29 //Ch 4 Hysteresis
#define ADS8668_PRG_HIGH_THRESHOLD_MSB_CH4 0x2A
#define ADS8668_PRG_HIGH_THRESHOLD_LSB_CH4 0x2B
#define ADS8668_PRG_LOW_THRESHOLD_MSB_CH4  0x2C
#define ADS8668_PRG_LOW_THRESHOLD_LSB_CH4  0x2D
    
#define ADS8668_PRG_HYSTERESIS_CH5         0x2E //Ch 5 Hysteresis
#define ADS8668_PRG_HIGH_THRESHOLD_MSB_CH5 0x2F
#define ADS8668_PRG_HIGH_THRESHOLD_LSB_CH5 0x30
#define ADS8668_PRG_LOW_THRESHOLD_MSB_CH5  0x31
#define ADS8668_PRG_LOW_THRESHOLD_LSB_CH5  0x32
    
#define ADS8668_PRG_HYSTERESIS_CH6         0x33 //Ch 6 Hysteresis
#define ADS8668_PRG_HIGH_THRESHOLD_MSB_CH6 0x34
#define ADS8668_PRG_HIGH_THRESHOLD_LSB_CH6 0x35
#define ADS8668_PRG_LOW_THRESHOLD_MSB_CH6  0x36
#define ADS8668_PRG_LOW_THRESHOLD_LSB_CH6  0x37
    
#define ADS8668_PRG_HYSTERESIS_CH7         0x38 //Ch 7 Hysteresis
#define ADS8668_PRG_HIGH_THRESHOLD_MSB_CH7 0x39
#define ADS8668_PRG_HIGH_THRESHOLD_LSB_CH7 0x3A
#define ADS8668_PRG_LOW_THRESHOLD_MSB_CH7  0x3B
#define ADS8668_PRG_LOW_THRESHOLD_LSB_CH7  0x3C
#endif

#define KALYKE_SPI_BASE    LPSPI1
    /* Select USB1 PLL PFD0 (720 MHz) as lpspi clock source */
#define KALYKE_LPSPI_CLOCK_SOURCE_SELECT (1U)
    /* Clock divider for master lpspi clock source */
#define KALYKE_LPSPI_CLOCK_SOURCE_DIVIDER (7U)
    
#define KALYKE_LPSPI_MASTER_CLK_FREQ (CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / (KALYKE_LPSPI_CLOCK_SOURCE_DIVIDER + 1U))
#define KALYKE_LPSPI_TRANSFER_BAUDRATE 900000U
#define KALYKE_LPSPI_MASTER_PCS_FOR_INIT (kLPSPI_Pcs0)

/* Task priorities. */
#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void test_lpspi_read(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
lpspi_rtos_handle_t gSpiRtosHandle;

static uint8_t g_tx_buffer[64];
static uint8_t g_rx_buffer[64];

/*******************************************************************************
 * Code
 ******************************************************************************/

void hexdump(const void *p, size_t size)
{
#if SDK_DEBUGCONSOLE == DEBUGCONSOLE_REDIRECT_TO_SDK

    const uint8_t *c = p;

    PRINTF("Dumping %u bytes from %p:\r\n", size, p);

    while (size > 0)
    {
        unsigned i;

        for (i = 0; i < 16; i++)
        {
            if (i < size)
                PRINTF("%02X ", c[i]);
            else
                PRINTF("   ");
        }
#if 0
        for (i = 0; i < 16; i++)
        {
            if (i < size)
                PRINTF("%c", c[i] >= 32 && c[i] < 127 ? c[i] : '.');
            else
                PRINTF(" ");
        }
#endif
        PRINTF("\r\n");

        c += 16;

        if (size <= 16)
            break;

        size -= 16;
    }
    PRINTF("\r\n");
#endif
}

// W25Q Flash : Read JEDEC ID (9Fh)
void test_rtos_lpspi_read4(void)
{
    PRINTF("Read JEDEC ID (9Fh)\r\n");
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_tx_buffer, 0, sizeof(g_tx_buffer));
    g_tx_buffer[0] = 0x9F;
    PRINTF("g_tx_buffer[0] = 0x%X\r\n", g_tx_buffer[0]);
    //g_tx_buffer[0] <<= 1;
    lpspi_transfer_t spi_transfer;
    spi_transfer.txData = g_tx_buffer;
    spi_transfer.rxData = g_rx_buffer;
    spi_transfer.dataSize = 4;
    //spi_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
    //spi_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterByteSwap;
    spi_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
    
    status_t ret = LPSPI_RTOS_Transfer(&gSpiRtosHandle, &spi_transfer);
    PRINTF("LPSPI_RTOS_Transfer return : %d\r\n", ret);
    hexdump(g_rx_buffer, sizeof(g_rx_buffer));
}

// W25Q Flash : Read JEDEC ID (9Fh)
void test_lpspi_read4(void)
{
    PRINTF("Read JEDEC ID (9Fh)\r\n");
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_tx_buffer, 0, sizeof(g_tx_buffer));
    g_tx_buffer[0] = 0x9F;
    PRINTF("g_tx_buffer[0] = 0x%X\r\n", g_tx_buffer[0]);
    //g_tx_buffer[0] <<= 1;
    lpspi_transfer_t spi_transfer;
    spi_transfer.txData = g_tx_buffer;
    spi_transfer.rxData = g_rx_buffer;
    spi_transfer.dataSize = 4;
    //spi_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
    //spi_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterByteSwap;
    spi_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
    status_t ret = LPSPI_MasterTransferBlocking(KALYKE_SPI_BASE, &spi_transfer);
    PRINTF("LPSPI_MasterTransferBlocking return : %d\r\n", ret);
    hexdump(g_rx_buffer, sizeof(g_rx_buffer));
}

// W25Q Flash : Read Manufacturer / Device ID (90h)
void test_lpspi_read3(void)
{
    PRINTF("Read Manufacturer / Device ID (90h)\r\n");
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_tx_buffer, 0, sizeof(g_tx_buffer));
    g_tx_buffer[0] = 0x90;
    PRINTF("g_tx_buffer[0] = 0x%X\r\n", g_tx_buffer[0]);
    //g_tx_buffer[0] <<= 1;
    lpspi_transfer_t spi_transfer;
    spi_transfer.txData = g_tx_buffer;
    spi_transfer.rxData = g_rx_buffer;
    spi_transfer.dataSize = 6;
    //spi_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
    //spi_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterByteSwap;
    spi_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
    status_t ret = LPSPI_MasterTransferBlocking(KALYKE_SPI_BASE, &spi_transfer);
    PRINTF("LPSPI_MasterTransferBlocking return : %d\r\n", ret);
    hexdump(g_rx_buffer, sizeof(g_rx_buffer));
}


// W25Q Flash : Read Unique ID Number (4Bh)
void test_lpspi_read2(void)
{
    PRINTF("Read Unique ID Number (4Bh)\r\n");
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_tx_buffer, 0, sizeof(g_tx_buffer));
    g_tx_buffer[0] = 0x4B;
    PRINTF("g_tx_buffer[0] = 0x%X\r\n", g_tx_buffer[0]);
    //g_tx_buffer[0] <<= 1;
    lpspi_transfer_t spi_transfer;
    spi_transfer.txData = g_tx_buffer;
    spi_transfer.rxData = g_rx_buffer;
    spi_transfer.dataSize = 13;
    spi_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
    //spi_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterByteSwap;
    //spi_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
    status_t ret = LPSPI_MasterTransferBlocking(KALYKE_SPI_BASE, &spi_transfer);
    PRINTF("LPSPI_MasterTransferBlocking return : %d\r\n", ret);
    hexdump(g_rx_buffer, sizeof(g_rx_buffer));
}

void test_lpspi_write_ads8668(void)
{
    PRINTF("Enter %s()\r\n", __func__);
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_tx_buffer, 0, sizeof(g_tx_buffer));
    g_tx_buffer[0] = ADS8668_PRG_HIGH_THRESHOLD_LSB_CH0 << 1;
    g_tx_buffer[0] += 1;
    PRINTF("g_tx_buffer[0] = 0x%X\r\n", g_tx_buffer[0]);
    g_tx_buffer[1] = 0x1B;

    lpspi_transfer_t spi_transfer;
    spi_transfer.txData = g_tx_buffer;
    spi_transfer.rxData = g_rx_buffer;
    spi_transfer.dataSize = 4;
    spi_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
    status_t ret = LPSPI_MasterTransferBlocking(KALYKE_SPI_BASE, &spi_transfer);
    PRINTF("LPSPI_MasterTransferBlocking return : %d\r\n", ret);
    hexdump(g_rx_buffer, sizeof(g_rx_buffer));
}

void test_lpspi_read_ads8668(void)
{
    PRINTF("Enter %s()\r\n", __func__);
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_tx_buffer, 0, sizeof(g_tx_buffer));
    g_tx_buffer[0] = ADS8668_PRG_HIGH_THRESHOLD_LSB_CH0 << 1;
    //g_tx_buffer[0] = ADS8668_PRG_HIGH_THRESHOLD_MSB_CH0 << 1;
    PRINTF("g_tx_buffer[0] = 0x%X\r\n", g_tx_buffer[0]);

    lpspi_transfer_t spi_transfer;
    spi_transfer.txData = g_tx_buffer;
    spi_transfer.rxData = g_rx_buffer;
    spi_transfer.dataSize = 4;
    spi_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
    status_t ret = LPSPI_MasterTransferBlocking(KALYKE_SPI_BASE, &spi_transfer);
    PRINTF("LPSPI_MasterTransferBlocking return : %d\r\n", ret);
    hexdump(g_rx_buffer, sizeof(g_rx_buffer));
}

void kalyke_lpspi_init_ADS8668(void)
{
    PRINTF("Enter %s()\r\n", __func__);
    NVIC_SetPriority(LPSPI3_IRQn, 3);

    /*Set clock source for LPSPI*/
    CLOCK_SetMux(kCLOCK_LpspiMux, KALYKE_LPSPI_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_LpspiDiv, KALYKE_LPSPI_CLOCK_SOURCE_DIVIDER);

    uint32_t srcClock_Hz = KALYKE_LPSPI_MASTER_CLK_FREQ;
    PRINTF("srcClock_Hz = %u\r\n", srcClock_Hz);
    lpspi_master_config_t masterConfig;
    /*Master config*/
    masterConfig.baudRate = KALYKE_LPSPI_TRANSFER_BAUDRATE;
    masterConfig.bitsPerFrame = 8;
    
    masterConfig.cpol = kLPSPI_ClockPolarityActiveHigh;
    masterConfig.cpha = kLPSPI_ClockPhaseSecondEdge;

    masterConfig.direction = kLPSPI_MsbFirst;

    masterConfig.pcsToSckDelayInNanoSec = 1000000000 / masterConfig.baudRate;
    //masterConfig.pcsToSckDelayInNanoSec = 0;
    masterConfig.lastSckToPcsDelayInNanoSec = 1000000000 / masterConfig.baudRate;
    //masterConfig.lastSckToPcsDelayInNanoSec = 0;
    masterConfig.betweenTransferDelayInNanoSec = 1000000000 / masterConfig.baudRate;
    //masterConfig.betweenTransferDelayInNanoSec = 0;
    
    masterConfig.whichPcs = KALYKE_LPSPI_MASTER_PCS_FOR_INIT;
    masterConfig.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;

    masterConfig.pinCfg = kLPSPI_SdiInSdoOut;
    masterConfig.dataOutConfig = kLpspiDataOutTristate;

    LPSPI_MasterInit(KALYKE_SPI_BASE, &masterConfig, srcClock_Hz);

    //test_lpspi_read();
    //test_lpspi_read2();
    //test_lpspi_read3();
    //test_lpspi_read4();
    //test_lpspi_read_ads8668();
    test_lpspi_write_ads8668();
}

void kalyke_AD_convert(lpspi_transfer_t *ptf)
{
    PRINTF("Enter %s()\r\n", __func__);
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    g_tx_buffer[0] = ADS8668_CMD_NO_OP;
    g_tx_buffer[1] = 0x00;
    status_t ret = LPSPI_RTOS_Transfer(&gSpiRtosHandle, ptf);
    PRINTF("LPSPI_RTOS_Transfer return : %d, cmd=%X\r\n", ret, g_tx_buffer[0]>>1);
    hexdump(g_rx_buffer, sizeof(g_rx_buffer));

    uint16_t ad = (g_rx_buffer[2] << 4) | (g_rx_buffer[3] >> 4);
    uint8_t chAddress = g_rx_buffer[4] >> 4;
    uint8_t inputRange = ((g_rx_buffer[4] & 0x03) << 1) | (g_rx_buffer[5] >> 7);
    PRINTF("ad = %u(0x%X), chAddress = %u, inputRange = %u\r\n", ad, ad, chAddress, inputRange);
}

void kalyke_config_ADS8668(void)
{
    PRINTF("Enter %s()\r\n", __func__);
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_tx_buffer, 0, sizeof(g_tx_buffer));
    g_tx_buffer[0] = (ADS8668_PRG_AUTO_SEQ_EN << 1) | 1;
    PRINTF("g_tx_buffer[0] = 0x%X\r\n", g_tx_buffer[0]);
    g_tx_buffer[1] = AUTO_SEQ_DN_CH0_MASK | AUTO_SEQ_DN_CH1_MASK;

    lpspi_transfer_t spi_transfer;
    spi_transfer.txData = g_tx_buffer;
    spi_transfer.rxData = g_rx_buffer;
    spi_transfer.dataSize = 4;
    spi_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;

    status_t ret = LPSPI_RTOS_Transfer(&gSpiRtosHandle, &spi_transfer);
    PRINTF("LPSPI_RTOS_Transfer return : %d, cmd=%X\r\n", ret, g_tx_buffer[0]>>1);
    hexdump(g_rx_buffer, sizeof(g_rx_buffer));

    g_tx_buffer[0] = (ADS8668_PRG_CH_PWR_DN << 1) | 1;
    g_tx_buffer[1] = ~g_tx_buffer[1];
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    ret = LPSPI_RTOS_Transfer(&gSpiRtosHandle, &spi_transfer);
    PRINTF("LPSPI_RTOS_Transfer return : %d, cmd=%X\r\n", ret, g_tx_buffer[0]>>1);
    hexdump(g_rx_buffer, sizeof(g_rx_buffer));

    g_tx_buffer[0] = (ADS8668_PRG_FEATURE_SEL << 1) | 1;
    g_tx_buffer[1] = 0x03;
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    ret = LPSPI_RTOS_Transfer(&gSpiRtosHandle, &spi_transfer);
    PRINTF("LPSPI_RTOS_Transfer return : %d, cmd=%X\r\n", ret, g_tx_buffer[0]>>1);
    hexdump(g_rx_buffer, sizeof(g_rx_buffer));


    g_tx_buffer[1] = INPUT_RANGE_POSITIVE_2_POINT_5_MULTIPLY_VREF;
    for (uint8_t i = 0; i < 8; i++)
    {
        memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
        g_tx_buffer[0] = ((ADS8668_PRG_RANGE_CH0 + i) << 1) | 1;
        ret = LPSPI_RTOS_Transfer(&gSpiRtosHandle, &spi_transfer);
        PRINTF("LPSPI_RTOS_Transfer return : %d, cmd=%X\r\n", ret, g_tx_buffer[0]>>1);
        hexdump(g_rx_buffer, sizeof(g_rx_buffer));
    }

    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    g_tx_buffer[0] = ADS8668_CMD_AUTO_RST;
    g_tx_buffer[1] = 0x00;
    spi_transfer.dataSize = 6;
    ret = LPSPI_RTOS_Transfer(&gSpiRtosHandle, &spi_transfer);
    PRINTF("LPSPI_RTOS_Transfer return : %d, cmd=%X\r\n", ret, g_tx_buffer[0]>>1);
    hexdump(g_rx_buffer, sizeof(g_rx_buffer));
}

void kalyke_rtos_lpspi_init_ADS8668(void)
{
    PRINTF("Enter %s()\r\n", __func__);
    NVIC_SetPriority(LPSPI1_IRQn, 5);

    /*Set clock source for LPSPI*/
    CLOCK_SetMux(kCLOCK_LpspiMux, KALYKE_LPSPI_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_LpspiDiv, KALYKE_LPSPI_CLOCK_SOURCE_DIVIDER);

    uint32_t srcClock_Hz = KALYKE_LPSPI_MASTER_CLK_FREQ;
    PRINTF("srcClock_Hz = %u\r\n", srcClock_Hz);
    lpspi_master_config_t masterConfig;
    /*Master config*/
    masterConfig.baudRate = KALYKE_LPSPI_TRANSFER_BAUDRATE;
    masterConfig.bitsPerFrame = 8;
    masterConfig.cpol = kLPSPI_ClockPolarityActiveHigh;
    masterConfig.cpha = kLPSPI_ClockPhaseSecondEdge;
    masterConfig.direction = kLPSPI_MsbFirst;

    masterConfig.pcsToSckDelayInNanoSec = 1000000000 / masterConfig.baudRate;
    //masterConfig.pcsToSckDelayInNanoSec = 0;
    masterConfig.lastSckToPcsDelayInNanoSec = 1000000000 / masterConfig.baudRate;
    //masterConfig.lastSckToPcsDelayInNanoSec = 0;
    masterConfig.betweenTransferDelayInNanoSec = 1000000000 / masterConfig.baudRate;
    //masterConfig.betweenTransferDelayInNanoSec = 0;
    
    masterConfig.whichPcs = KALYKE_LPSPI_MASTER_PCS_FOR_INIT;
    masterConfig.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;

    masterConfig.pinCfg = kLPSPI_SdiInSdoOut;
    masterConfig.dataOutConfig = kLpspiDataOutTristate;

    status_t ret = LPSPI_RTOS_Init(&gSpiRtosHandle, KALYKE_SPI_BASE, &masterConfig, srcClock_Hz);
    PRINTF("LPSPI_RTOS_Init return : %d\r\n", ret);

    //test_rtos_lpspi_read4();
    kalyke_config_ADS8668();
}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
static void hello_task(void *pvParameters)
{
    PRINTF("hello_task RUN. Free heap size is %d bytes\r\n", xPortGetFreeHeapSize());

    vTaskDelay(4000);
    //kalyke_lpspi_init_ADS8668();
    kalyke_rtos_lpspi_init_ADS8668();

    lpspi_transfer_t spi_transfer;
    spi_transfer.txData = g_tx_buffer;
    spi_transfer.rxData = g_rx_buffer;
    spi_transfer.dataSize = 6;
    spi_transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
    for (;;)
    {
        PRINTF("Hello world.\r\n");
        vTaskDelay(4500);
        //test_lpspi_read2();
        //test_lpspi_read_ads8668();
        kalyke_AD_convert(&spi_transfer);
    }
}


#define SW_VERSION  "0.0.624.01"
/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    PRINTF("SW_VERSION = %s\r\n", SW_VERSION);
    #if 0
    if (xTaskCreate(hello_task, "Hello_task", configMINIMAL_STACK_SIZE + 10, NULL, hello_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }
    #else
    if (xTaskCreate(i2c_task, "i2c_task", configMINIMAL_STACK_SIZE + 10, NULL, hello_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("i2c_task creation failed!.\r\n");
        while (1)
            ;
    }
    #endif
    vTaskStartScheduler();
    for (;;)
        ;
}

