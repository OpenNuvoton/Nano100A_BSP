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
#include "targetdev.h"
#include "uart_transfer.h"


#define PLLCON_SETTING          CLK_PLLCTL_192MHz_HIRC
#define PLL_CLOCK               192000000
#define HCLK_DIV                        1

#define nRTSPin                 (PA0)
#define RECEIVE_MODE            (0)
#define TRANSMIT_MODE           (1)

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

    //while ((!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk)));

    /* 96MHz / (2 + 1) = 32MHz */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLK_N_Msk) | 2;
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLK_S_Msk) | CLK_CLKSEL0_HCLK_S_PLL;

    //SystemCoreClockUpdate();
    SystemCoreClock = 32000000;     // HCLK
    CyclesPerUs     = SystemCoreClock / 1000000;  // For SYS_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART1_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HIRC;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    PA->PMD = (PA->PMD & ~(GP_PMD_PMD0_Msk) | (GPIO_PMD_OUTPUT << GP_PMD_PMD0_Pos));
    nRTSPin = RECEIVE_MODE;
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~(SYS_PA_L_MFP_PA2_MFP_Msk)) | SYS_PA_L_MFP_PA2_MFP_UART1_RX;
    SYS->PB_L_MFP = (SYS->PB_L_MFP & ~(SYS_PB_L_MFP_PB5_MFP_Msk)) | SYS_PB_L_MFP_PB5_MFP_UART1_TX;
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Init UART to 115200-8n1 */
    UART_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
    FMC->ISPCON |= FMC_ISPCON_ISPEN_Msk;
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;//using cpu clock

    while (1)
    {
        if ((bufhead >= 4) || (bUartDataReady == TRUE))
        {
            uint32_t lcmd;
            lcmd = inpw(uart_rcvbuf);

            if (lcmd == CMD_CONNECT)
            {
                goto _ISP;
            }
            else
            {
                bUartDataReady = FALSE;
                bufhead = 0;
            }
        }

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto _APROM;
        }
    }

_ISP:

    while (1)
    {
        if (bUartDataReady == TRUE)
        {
            bUartDataReady = FALSE;
            ParseCmd(uart_rcvbuf, 64);
            NVIC_DisableIRQ(UART1_IRQn);
            nRTSPin = TRANSMIT_MODE;
            PutString();

            while ((UART1->FSR & UART_FSR_TX_EMPTY_F_Msk) == 0);
            while ((UART1->FSR & UART_FSR_TE_F_Msk) == 0);

            nRTSPin = RECEIVE_MODE;
            NVIC_EnableIRQ(UART1_IRQn);
        }
    }

_APROM:
    outpw(&SYS->RST_SRC, 3);//clear bit
    outpw(&FMC->ISPCON, inpw(&FMC->ISPCON) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

    /* Trap the CPU */
    while (1);
}
