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
#include "fsl_lpi2c_freertos.h"

#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LPI2C_MASTER_SLAVE_ADDR_7BIT 0x50U
#define LPI2C_BAUDRATE 100000U

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (5U)
/* Get frequency of lpi2c clock */
#define LPI2C_CLOCK_FREQUENCY ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

#define LPI2C_MASTER_CLOCK_FREQUENCY    LPI2C_CLOCK_FREQUENCY

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Code
 ******************************************************************************/

static void hexdump(const void *p, size_t size)
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

lpi2c_rtos_handle_t gI2cHandle;

void kalyke_rtos_lpi2c_init(void)
{
    PRINTF("Enter %s()\r\n", __func__);
    NVIC_SetPriority(LPI2C1_IRQn, 4);
    
    /*Clock setting for LPI2C*/
    CLOCK_SetMux(kCLOCK_Lpi2cMux, LPI2C_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, LPI2C_CLOCK_SOURCE_DIVIDER);

    uint32_t srcClock_Hz = LPI2C_CLOCK_FREQUENCY;
    PRINTF("srcClock_Hz = %u\r\n", srcClock_Hz);
    
    lpi2c_master_config_t masterConfig;
    LPI2C_MasterGetDefaultConfig(&masterConfig);
    /* Change the default baudrate configuration */
    masterConfig.baudRate_Hz = LPI2C_BAUDRATE;
    status_t ret = LPI2C_RTOS_Init(&gI2cHandle, LPI2C1, &masterConfig, srcClock_Hz);
    PRINTF("LPI2C_RTOS_Init return : %d\r\n", ret);

    
}

#define AT24_ADDRESS    0x01
static void test_lpi2c_write(uint32_t addr, uint8_t val)
{
    PRINTF("Enter %s(), add = %u, val = %u\r\n", __func__, addr, val);
    uint8_t dataForWrite = val;
    
    lpi2c_master_transfer_t mt;
    mt.flags = kLPI2C_TransferDefaultFlag;
    mt.slaveAddress = LPI2C_MASTER_SLAVE_ADDR_7BIT;
    mt.direction = kLPI2C_Write;
    mt.subaddress = addr;
    mt.subaddressSize = 1;
    mt.data = &dataForWrite;
    mt.dataSize = 1;
    status_t ret = LPI2C_RTOS_Transfer(&gI2cHandle, &mt);
    PRINTF("LPI2C_RTOS_Transfer return : %d\r\n", ret);
}

static void test_lpi2c_read(uint32_t addr)
{
    PRINTF("Enter %s(), addr = %u\r\n", __func__, addr);
    uint8_t dataForRead = 0x00;
    
    lpi2c_master_transfer_t mt;
    mt.flags = kLPI2C_TransferDefaultFlag;
    mt.slaveAddress = LPI2C_MASTER_SLAVE_ADDR_7BIT;
    mt.direction = kLPI2C_Read;
    mt.subaddress = addr;
    mt.subaddressSize = 1;
    mt.data = &dataForRead;
    mt.dataSize = 1;
    status_t ret = LPI2C_RTOS_Transfer(&gI2cHandle, &mt);
    PRINTF("LPI2C_RTOS_Transfer return : %d, dataForRead = %u\r\n", ret, dataForRead);
}

void i2c_task(void *pvParameters)
{
    static uint32_t counts = 0;
    PRINTF("i2c_task RUN. Free heap size is %d bytes\r\n", xPortGetFreeHeapSize());
    vTaskDelay(4000);
    kalyke_rtos_lpi2c_init();
    for (;;)
    {
        vTaskDelay(4001);
        PRINTF("i2c task running...(%u)\r\n", counts);
        if (counts == 0)
        {
            test_lpi2c_read(counts);
            test_lpi2c_write(3, 76);
            counts++;
            continue;
        }
        if (counts == 1)
        {
            test_lpi2c_read(counts);
            test_lpi2c_write(5, 188);
            counts++;
            continue;
        }
        
        test_lpi2c_read(counts);
        counts++;
        if (counts >= 256)
        {
            counts = 0;
        }
    }
}


