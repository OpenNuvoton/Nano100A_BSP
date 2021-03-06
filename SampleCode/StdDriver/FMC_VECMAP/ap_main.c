/**************************************************************************//**
 * @file     ap_main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/06/01 4:41p $
 * @brief    Show how to branch programs between LDROM, APROM start page,
 *           and APROM other page.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "map.h"


static int  load_image_to_flash(uint32_t image_base, uint32_t image_size, uint32_t flash_addr, uint32_t max_size);


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
    int         cbs;
    uint32_t    au32Config[2];

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART0 for printf */
    UART0_Init();

    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    printf("\n\n");
    printf("+--------------------------------------------------+\n");
    printf("|         User program running on APROM            |\n");
    printf("+--------------------------------------------------+\n");

    /*-------------------------------------------------------------
     *  Modify CBS to 11b (boot from APROM)
     *------------------------------------------------------------*/
    if (FMC_ReadConfig(au32Config, 2) < 0)
    {
        printf("\n\nFailed to read Config!\n\n");
        return -1;
    }
    cbs = (au32Config[0] >> 6) & 0x3;
    printf("Config0 = 0x%x, Config1 = 0x%x, CBS=%d\n\n", au32Config[0], au32Config[1], cbs);

    if (cbs != 0x3)
    {
        printf("\n\nChange boot setting to [Boot from APROM].\n");
        FMC_ENABLE_CFG_UPDATE();
        au32Config[0] |= 0xc0;          /* set CBS to 11b */
        FMC_WriteConfig(au32Config, 2);
    }

    printf("\n\n");
    printf("+--------------------------------------+\n");
    printf("|                                      |\n");
    printf("|     Nano100 FMC VECMAP sample main   |\n");
    printf("|      (Boot from APROM)               |\n");
    printf("|                                      |\n");
    printf("+--------------------------------------+\n");

    /*-------------------------------------------------------------
     *  Program LDROM image
     *------------------------------------------------------------*/
    printf("Writing fmc_ld_boot.bin image to LDROM...\n");
    FMC_ENABLE_LD_UPDATE();
    if (load_image_to_flash((uint32_t)&loaderImage1Base, FMC_LDROM_SIZE,
                            FMC_LDROM_BASE, FMC_LDROM_SIZE) != 0)
    {
        printf("Load image to LDROM failed!\n");
        return -1;
    }
    FMC_DISABLE_LD_UPDATE();

    /*-------------------------------------------------------------
     *  Program APROM ISP image
     *------------------------------------------------------------*/
    printf("Writing fmc_isp.bin image to APROM address 0x%x...\n", ISP_CODE_BASE);
    FMC_ENABLE_AP_UPDATE();
    if (load_image_to_flash((uint32_t)&loaderImage2Base, ISP_CODE_MAX_SIZE,
                            ISP_CODE_BASE, ISP_CODE_MAX_SIZE) != 0)
    {
        printf("Load image to APROM failed!\n");
        return -1;
    }
    FMC_DISABLE_AP_UPDATE();

    /*-------------------------------------------------------------
     *  Modify CBS to 00b (boot from LDROM with IAP)
     *------------------------------------------------------------*/
    if (FMC_ReadConfig(au32Config, 2) < 0)
    {
        printf("\n\nFailed to read Config!\n\n");
        return -1;
    }
    cbs = (au32Config[0] >> 6) & 0x3;

    if ((cbs != 0) || !(au32Config[0] & 0x1))
    {
        FMC_ENABLE_CFG_UPDATE();
        au32Config[0] &= ~0xc0;         /* set CBS to 00b */
        au32Config[0] |= 0x1;           /* disable Data Flash */
        FMC_WriteConfig(au32Config, 2);
    }

    printf("Will execute chip reset, press any key to boot from LDROM with VECMAP...\n\n");
    getchar();

    // do chip reset
    SYS->IPRST_CTL1 |= SYS_IPRST_CTL1_CHIP_RST_Msk;

    return 0;
}


static int  load_image_to_flash(uint32_t image_base, uint32_t image_size, uint32_t flash_addr, uint32_t max_size)
{
    uint32_t   i, j, u32Data, *pu32Loader;

    printf("Program image to flash address 0x%x...", flash_addr);
    pu32Loader = (uint32_t *)image_base;
    for (i = 0; i < image_size; i += FMC_FLASH_PAGE_SIZE)
    {
        if (FMC_Erase(flash_addr + i))
        {
            printf("Erase failed on 0x%x\n", flash_addr + i);
            return -1;
        }

        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            FMC_Write(flash_addr + i + j, pu32Loader[(i + j) / 4]);
        }
    }
    printf("OK.\n");

    printf("Verify ...");

    /* Verify loader */
    for (i = 0; i < image_size; i += FMC_FLASH_PAGE_SIZE)
    {
        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            u32Data = FMC_Read(flash_addr + i + j);
            if (u32Data != pu32Loader[(i+j)/4])
            {
                printf("data mismatch on 0x%x, [0x%x], [0x%x]\n", flash_addr + i + j, u32Data, pu32Loader[(i+j)/4]);
                return -1;
            }

            if (i + j >= image_size)
                break;
        }
    }
    printf("OK.\n");
    return 0;
}


