/**************************************************************************//**
 * @file     ld_boot.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/05/28 6:07p $
 * @brief    Show how to branch programs between LDROM, APROM start page,
 *           and APROM other page.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "map.h"


#ifdef __ARMCC_VERSION
__asm __set_SP(uint32_t _sp)
{
    MSR MSP, r0
    BX lr
}
#endif


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL &= ~CLK_PWRCTL_HXT_EN_Msk;
    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_HXT_EN_Pos); // HXT Enabled

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_HXT_STB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HXT;

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN; // UART0 Clock Enable

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART_S_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD  */
    //SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    //SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_TX | SYS_PB_L_MFP_PB1_MFP_UART0_RX);
    SYS->PB_L_MFP = ((SYS->PB_L_MFP & ~0xFF) | 0x11);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART0, 115200);
}



/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    int             u8Item;
    uint32_t        sp;
    FUNC_PTR        *func;
    volatile int    loop;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART0 for printf */
    UART0_Init();

    /* Enable FMC ISP function */
    SYS_UnlockReg();
    FMC_Open();

    do {
        printf("\n");
        printf("+----------------------------------------------+\n");
        printf("|       LD boot program running on LDROM       |\n");
        printf("+----------------------------------------------|\n");
        printf("| [0] Run ISP program (at APROM %dK)           |\n", USER_AP_MAX_SIZE/1024);
        printf("| [1] Branch and run APROM program             |\n");
        printf("| [2] Set boot mode as boot from APROM.        |\n");
        printf("+----------------------------------------------+\n");
        printf("Please select...");
        u8Item = getchar();
        printf("%c\n", u8Item);

        switch (u8Item) {
        case '0':
            sp = FMC_Read(ISP_CODE_BASE);
            func =  (FUNC_PTR *)FMC_Read(ISP_CODE_ENTRY+4);
            printf("branch_to address 0x%x\n", (int)func);
            printf("\nChange VECMAP and branch to ISP code...\n");
            while (!UART_IS_TX_EMPTY(UART0));

            FMC_SetVectorPageAddr(ISP_CODE_BASE);

            /* Switch HCLK clock source to HIRC */
            CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLK_S_Msk) | CLK_CLKSEL0_HCLK_S_HIRC;

            __set_SP(sp);
            func();
            while (1);

        case '1':
            sp = FMC_Read(USER_AP_ENTRY);
            func =  (FUNC_PTR *)FMC_Read(USER_AP_ENTRY+4);
            printf("APROM code's SP is 0x%x\n", sp);
            printf("branch_to address 0x%x\n", (int)func);
            printf("\n\nChange VECMAP and branch to user application...\n");
            while (!UART_IS_TX_EMPTY(UART0));

            FMC_SetVectorPageAddr(USER_AP_ENTRY);

            /* Switch HCLK clock source to HIRC */
            CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLK_S_Msk) | CLK_CLKSEL0_HCLK_S_HIRC;

            __set_SP(sp);
            func();
            while (1);

        case '2':
            FMC_ENABLE_CFG_UPDATE();
            FMC_Erase(FMC_CONFIG_BASE);

            // do chip reset
            SYS->IPRST_CTL1 |= SYS_IPRST_CTL1_CHIP_RST_Msk;
            break;

        default :
            continue;
        }
    } while (1);

}



