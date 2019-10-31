/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "targetdev.h"
#include "i2c_transfer.h"

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS->RegLockAddr = 0x59;
    SYS->RegLockAddr = 0x16;
    SYS->RegLockAddr = 0x88;
    /* Enable internal 12MHz */
    CLK->PWRCTL |= (CLK_PWRCTL_HIRC_EN_Msk | CLK_PWRCTL_HXT_EN_Msk);

    /* Waiting for 12MHz clock ready */
    while ((!(CLK->CLKSTATUS & CLK_CLKSTATUS_HIRC_STB_Msk)));

    /* 12MHz HIRC ==> 96MHz Pll Colck Output */
    CLK->PLLCTL = 0x20220;

    while ((!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk)));

    /* 96MHz / (2 + 1) = 32MHz */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLK_N_Msk) | 2;
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLK_S_Msk) | CLK_CLKSEL0_HCLK_S_PLL;

    //SystemCoreClockUpdate();
    SystemCoreClock = 32000000;     // HCLK
    CyclesPerUs     = SystemCoreClock / 1000000;  // For SYS_SysTickDelay()

    /* Enable I2C1 clock */
    CLK->APBCLK |= CLK_APBCLK_I2C1_EN_Msk;

    /* Set I2C1 multi-function pins */
    SYS->PA_H_MFP = (SYS->PA_H_MFP & ~(SYS_PA_H_MFP_PA10_MFP_Msk | SYS_PA_H_MFP_PA11_MFP_Msk)) |
                    (SYS_PA_H_MFP_PA10_MFP_I2C1_SDA | SYS_PA_H_MFP_PA11_MFP_I2C1_SCL);

}

int main(void)
{
    uint32_t cmd_buff[16];
    SYS_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
    FMC->ISPCON |= FMC_ISPCON_ISPEN_Msk;
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    I2C_Init();
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while (1)
    {
        if (bI2cDataReady == 1)
        {
            goto _ISP;
        }

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto _APROM;
        }

    }

_ISP:

    while (1)
    {
        if (bI2cDataReady == 1)
        {
            memcpy(cmd_buff, i2c_rcvbuf, 64);
            bI2cDataReady = 0;
            ParseCmd((unsigned char *)cmd_buff, 64);
            NVIC_EnableIRQ(I2C1_IRQn);
        }
    }

_APROM:
    outpw(&SYS->RST_SRC, 3);//clear bit
    outpw(&FMC->ISPCON, inpw(&FMC->ISPCON) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

    /* Trap the CPU */
    while (1);
}
