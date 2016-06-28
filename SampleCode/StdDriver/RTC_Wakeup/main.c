/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/10/20 1:13p $
 * @brief    Demonstrate how to wake up system periodically with RTC interrupt.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "Nano100Series.h"
#include "sys.h"
#include "rtc.h"

/* External functions */
void planNextRTCInterrupt(S_RTC_TIME_DATA_T *sCurTime);

void Leave_PowerDown(void);

/* Global variables */
__IO int32_t   _Wakeup_Flag = 0;    /* 1 indicates system wake up from power down mode */
__IO uint32_t  _Pin_Setting[11];    /* store Px_H_MFP and Px_L_MFP */
__IO uint32_t  _PullUp_Setting[6];  /* store GPIOx_PUEN */

/**
  * @brief  PDWU IRQHandler.
  * @param  None.
  * @return None.
  */
void PDWU_IRQHandler()
{
    printf("PDWU_IRQHandler running...\n");
    CLK->WK_IS = 1; /* clear interrupt status */
    Leave_PowerDown();
    _Wakeup_Flag = 1;
}


/**
  * @brief  RTC IRQHandler.
  * @param  None.
  * @return None.
  */
void RTC_IRQHandler()
{
    S_RTC_TIME_DATA_T sCurTime;

    printf("RTC_IRQHandler running...\n");

    /* RTC Tick interrupt */
    if ((RTC->RIER & RTC_RIER_TIER_Msk) && (RTC->RIIR & RTC_RIIR_TIF_Msk)) {
        printf("RTC Tick Interrupt.\n");
        RTC->RIIR = RTC_RIIR_TIF_Msk;
    }

    /* RTC Alarm interrupt */
    if ((RTC->RIER & RTC_RIER_AIER_Msk) && (RTC->RIIR & RTC_RIIR_AIF_Msk)) {
        printf("RTC Alarm Interrupt.\n");
        RTC->RIIR = RTC_RIIR_AIF_Msk;

        RTC_GetDateAndTime(&sCurTime);
        printf("Current Time:%d/%02d/%02d %02d:%02d:%02d\n",sCurTime.u32Year,sCurTime.u32Month,sCurTime.u32Day,sCurTime.u32Hour,sCurTime.u32Minute,sCurTime.u32Second);

        RTC_DISABLE_TICK_WAKEUP();  /* RTC tick shouldn't wake up CPU */
        planNextRTCInterrupt(&sCurTime);
    }

    if ((RTC->RIER & RTC_RIER_SNOOPIER_Msk) && (RTC->RIIR & RTC_RIIR_SNOOPIF_Msk)) { /* snooper interrupt occurred */
        RTC->RIIR = RTC_RIIR_SNOOPIF_Msk;
    }

}

/**
  * @brief  plan next RTC Interrupt time.
  * @param  sCurTime.
  * @return None.
  */
void planNextRTCInterrupt(S_RTC_TIME_DATA_T *sCurTime)
{
    // plan next interrupt timing
    if(sCurTime->u32Minute < 59)
        sCurTime->u32Minute += 1;
    else {
        if(sCurTime->u32Hour < 23)
            sCurTime->u32Hour += 1;
        else {  // next day
            sCurTime->u32Hour = 0;

            // new year first day
            if(sCurTime->u32Month==12 && sCurTime->u32Day==31) {
                sCurTime->u32Year += 1;
                sCurTime->u32Month = 1;
                sCurTime->u32Day = 1;
            } else if(sCurTime->u32Month==1 ||
                      sCurTime->u32Month==3 ||
                      sCurTime->u32Month==5 ||
                      sCurTime->u32Month==7 ||
                      sCurTime->u32Month==8 ||
                      sCurTime->u32Month==10 ||
                      sCurTime->u32Month==12) { // 1,3,5,7,8,10,12 31-day month
                if(sCurTime->u32Day < 31)
                    sCurTime->u32Day += 1;
                else {
                    sCurTime->u32Day = 1;
                    sCurTime->u32Month += 1;
                }
            } else if(sCurTime->u32Month==2) { // 2, 28 or 29-day month
                if(RTC_IS_LEAP_YEAR()) { // leap year
                    if(sCurTime->u32Day < 29)
                        sCurTime->u32Day += 1;
                    else {
                        sCurTime->u32Day = 1;
                        sCurTime->u32Month += 1;
                    }
                } else {
                    if(sCurTime->u32Day < 28)
                        sCurTime->u32Day += 1;
                    else {
                        sCurTime->u32Day = 1;
                        sCurTime->u32Month += 1;
                    }
                }
            } else if(sCurTime->u32Month==4 ||
                      sCurTime->u32Month==6 ||
                      sCurTime->u32Month==9 ||
                      sCurTime->u32Month==11) { // 4,6,9,11 30-day
                if(sCurTime->u32Day < 30)
                    sCurTime->u32Day += 1;
                else {
                    sCurTime->u32Day = 1;
                    sCurTime->u32Month += 1;
                }
            }
        }

        sCurTime->u32Month = 0;
    }
    sCurTime->u32Second = 0;

    RTC_SetAlarmDateAndTime(sCurTime);
    RTC_EnableInt(RTC_RIER_AIER_Msk);
    NVIC_EnableIRQ(RTC_IRQn);

}

/**
  * @brief  Store original setting of multi-function pin selection.
  * @param  None.
  * @return None.
  */
void SavePinSetting()
{
    /* Save Pin selection setting */
    _Pin_Setting[0] = SYS->PA_L_MFP;
    _Pin_Setting[1] = SYS->PA_H_MFP;
    _Pin_Setting[2] = SYS->PB_L_MFP;
    _Pin_Setting[3] = SYS->PB_H_MFP;
    _Pin_Setting[4] = SYS->PC_L_MFP;
    _Pin_Setting[5] = SYS->PC_H_MFP;
    _Pin_Setting[6] = SYS->PD_L_MFP;
    _Pin_Setting[7] = SYS->PD_H_MFP;
    _Pin_Setting[8] = SYS->PE_L_MFP;
    _Pin_Setting[9] = SYS->PE_H_MFP;
    _Pin_Setting[10] = SYS->PF_L_MFP;

    /* Save Pull-up setting */
    _PullUp_Setting[0] =  PA->PUEN;
    _PullUp_Setting[1] =  PB->PUEN;
    _PullUp_Setting[2] =  PC->PUEN;
    _PullUp_Setting[3] =  PD->PUEN;
    _PullUp_Setting[4] =  PE->PUEN;
    _PullUp_Setting[5] =  PF->PUEN;
}

/**
  * @brief  Restore original setting of multi-function pin selection.
  * @param  None.
  * @return None.
  */
void RestorePinSetting()
{
    /* Restore Pin selection setting */
    SYS->PA_L_MFP = _Pin_Setting[0];
    SYS->PA_H_MFP = _Pin_Setting[1];
    SYS->PB_L_MFP = _Pin_Setting[2];
    SYS->PB_H_MFP = _Pin_Setting[3];
    SYS->PC_L_MFP = _Pin_Setting[4];
    SYS->PC_H_MFP = _Pin_Setting[5];
    SYS->PD_L_MFP = _Pin_Setting[6];
    SYS->PD_H_MFP = _Pin_Setting[7];
    SYS->PE_L_MFP = _Pin_Setting[8];
    SYS->PE_H_MFP = _Pin_Setting[9];
    SYS->PF_L_MFP = _Pin_Setting[10];

    /* Restore Pull-up setting */
    PA->PUEN = _PullUp_Setting[0];
    PB->PUEN = _PullUp_Setting[1];
    PC->PUEN = _PullUp_Setting[2];
    PD->PUEN = _PullUp_Setting[3];
    PE->PUEN = _PullUp_Setting[4];
    PF->PUEN = _PullUp_Setting[5];
}


/**
  * @brief  Save multi-function pin setting and then go to power down.
  * @param  None.
  * @return None.
  */
void Enter_PowerDown()
{
    /* Back up original setting */
    SavePinSetting();

    /* Set function pin to GPIO mode */
    SYS->PA_L_MFP = 0;
    SYS->PA_H_MFP = 0;
    SYS->PB_L_MFP = 0;
    SYS->PB_H_MFP = 0;
    SYS->PC_L_MFP = 0;
    SYS->PC_H_MFP = 0;
    SYS->PD_L_MFP = 0;
    SYS->PD_H_MFP = 0;
    SYS->PE_L_MFP = 0;
    SYS->PE_H_MFP = 0;
    SYS->PF_L_MFP = 0x00007700;

    /* Enable GPIO pull up */
    PA->PUEN = 0xFFFF;
    PB->PUEN = 0xFFFF;
    PC->PUEN = 0xFFFF;
    PD->PUEN = 0xFFFF;
    PE->PUEN = 0xFFFF;
    PF->PUEN = 0x0033;      /* exclude GPF2 and GPF3 which are HXT OUT/IN */

    CLK->PWRCTL |= CLK_PWRCTL_LXT_EN_Msk; /* enable LXT - 32Khz */
    CLK->PWRCTL |= CLK_PWRCTL_PD_WK_IE_Msk;  /* Enable wake up interrupt source */
    NVIC_EnableIRQ(PDWU_IRQn);             /* Enable IRQ request for PDWU interrupt */
    CLK_PowerDown();

}


/**
  * @brief  This function is called by PDWU_IRQHandler to restore original setting of multi-function pin selection.
  * @param  None.
  * @return None.
  */
void Leave_PowerDown()
{
    /* Restore pin setting */
    RestorePinSetting();

    /* Set PF.0 and PF.1 to ICE Data and Clock */
    SYS->PF_L_MFP |= 0x00000077;

}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_HXT_EN_Pos); // HXT Enabled

    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_LXT_EN_Pos); // LXT Enable

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_HXT_STB_Msk);
    /* Waiting for 32KHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_LXT_STB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HXT;

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART_S_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN; // UART0 Clock Enable
    CLK->APBCLK |= CLK_APBCLK_RTC_EN;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD  */
    SYS->PA_H_MFP &= ~(SYS_PA_H_MFP_PA14_MFP_Msk|SYS_PA_H_MFP_PA15_MFP_Msk);
    SYS->PA_H_MFP |=  (SYS_PA_H_MFP_PA14_MFP_UART0_RX|SYS_PA_H_MFP_PA15_MFP_UART0_TX);

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


/**
  * @brief  Main routine.
  * @param  None.
  * @return None.
  */
int32_t main(void)
{
    S_RTC_TIME_DATA_T sCurTime;

    SYS_Init();
    UART0_Init();

    /* Unlock protected registers */
    SYS_UnlockReg();

    printf("\nNANO130 RTC Wakeup Demo \n");

    /* Time Setting */
    sCurTime.u32Year       = 2014;
    sCurTime.u32Month      = 10;
    sCurTime.u32Day        = 1;
    sCurTime.u32Hour       = 0;
    sCurTime.u32Minute     = 0;
    sCurTime.u32Second     = 0;
    sCurTime.u32DayOfWeek  = RTC_TUESDAY;
    sCurTime.u32TimeScale  = RTC_CLOCK_24;

    RTC_Open(&sCurTime);

    /* Read curent RTC time */
    RTC_GetDateAndTime(&sCurTime);
    printf("Current Time:%d/%02d/%02d %02d:%02d:%02d\n",sCurTime.u32Year,sCurTime.u32Month,sCurTime.u32Day,sCurTime.u32Hour,sCurTime.u32Minute,sCurTime.u32Second);

    /* Enable RTC alarm for 1 minute to update RTC time */
    RTC_DISABLE_TICK_WAKEUP();  /* RTC tick shouldn't wake up CPU */
    planNextRTCInterrupt(&sCurTime);

    _Wakeup_Flag = 0;
    while(1) {
        printf("\nGoing to Power Down...\n");

        while(!(UART0->FSR & UART_FSR_TE_F_Msk)) ;  /* waits for message send out */

        /* Enter power down mode */
        Enter_PowerDown();

        printf("Program resume...\n");

        if (_Wakeup_Flag == 1) {
            _Wakeup_Flag = 0;

            printf("System Wakeup success!! \n");
        }
    }
}


#ifdef USE_ASSERT
/**
  * @brief  The function prints the source file name and line number where the assert_param() error
  *         occurs, and then stops in an infinite loop. User can add his own codes here if necessary.
  * @param[in] file Source file name
  * @param[in] line Line number
  * @return None
  */
void assert_error(uint8_t * file, uint32_t line)
{
    MFP_UART0_TO_PORTA();                  /* UART0 TX/RX to PA14/PA15*/
    CLK->APBCLK |= CLK_APBCLK_UART0_EN;    /* Enable UART0 clock */
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_UART_MASK;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART_MASK) | CLK_CLKSEL1_UART_HXT;  /* Select 12 Mhz XTAL */


    /* Set UART to 115200,n,8,1,none */
    UART0->BAUD = 0x67;             /* Baud Rate:115200 for 12MHz */
    UART0->TLCTL = 0x03;            /* Word len is 8 bits */

    printf("[%s] line %d : wrong parameters.\r\n", file, line);

    /* Infinite loop */
    while(1) ;

}
#endif


/*** (C) COPYRIGHT 2012 Nuvoton Technology Corp. ***/



