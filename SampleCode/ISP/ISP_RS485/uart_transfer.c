/**************************************************************************//**
 * @file     uart_transfer.c
 * @version  V1.00
 * $Date: 14/11/17 5:36p $
 * @brief    General UART ISP slave Sample file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "targetdev.h"
#include "uart_transfer.h"

__align(4) uint8_t  uart_rcvbuf[MAX_PKT_SIZE] = {0};

uint8_t volatile bUartDataReady = 0;
uint8_t volatile bufhead = 0;


/* please check "targetdev.h" for chip specific define option */

/*---------------------------------------------------------------------------------------------------------*/
/* INTSTS to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    /*----- Determine interrupt source -----*/
    uint32_t u32IntSrc = UART1->ISR;

    if (u32IntSrc & 0x11)   //RDA FIFO interrupt & RDA timeout interrupt
    {
        while (((UART1->FSR & UART_FSR_RX_EMPTY_F_Msk) == 0) && (bufhead < MAX_PKT_SIZE))      //RX fifo not empty
        {
            uart_rcvbuf[bufhead++] = UART1->RBR;
        }
    }

    if (bufhead == MAX_PKT_SIZE)
    {
        bUartDataReady = TRUE;
        bufhead = 0;
    }
    else if (u32IntSrc & 0x10)
    {
        bufhead = 0;
    }
}

extern __align(4) uint8_t response_buff[64];
void PutString(void)
{
    uint32_t i;

    while (!(UART1->FSR & UART_FSR_TX_EMPTY_F_Msk));

    for (i = 0; i < MAX_PKT_SIZE; i++)
    {
        UART1->THR = response_buff[i];
        while (!(UART1->FSR & UART_FSR_TX_EMPTY_F_Msk));
    }
}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select UART function */
    UART1->FUN_SEL = UART_FUNC_SEL_UART;
    /* Set UART line configuration */
    UART1->TLCTL = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1 |
                   UART_TLCTL_RFITL_14BYTES | UART_TLCTL_RTS_TRI_LEV_14BYTES;

    /* Set UART baud rate */
    UART1->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(__HIRC, 115200));
    /* Set time-out interrupt comparator */
    UART1->TMCTL = 0x80;
    NVIC_SetPriority(UART1_IRQn, 2);
    NVIC_EnableIRQ(UART1_IRQn);
    /* 0x0811 */
    UART1->IER = (UART_IER_RTO_IE_Msk | UART_IER_RDA_IE_Msk);

}

