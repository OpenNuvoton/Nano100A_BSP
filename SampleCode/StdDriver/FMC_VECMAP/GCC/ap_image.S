;/**************************************************************************//**
; * @file     ap_images.S
; * @version  V1.00
; * $Revision: 1 $
; * $Date: 18/06/21 4:41p $
; * @brief    Embedded fmc_ld_boot.bin and fmc_isp.bin image into ap_main.bin.
; *
; * @note
; * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/

.syntax unified
.arch   armv7-m

.globl  loaderImage1Base
.globl  loaderImage1Limit

.globl  loaderImage2Base
.globl  loaderImage2Limit

.align  4

.text

loaderImage1Base:
.incbin  "..\\..\\fmc_ld_boot.bin"
loaderImage1Limit:
.space   4

loaderImage2Base:
.incbin  "..\\..\\fmc_isp.bin"
loaderImage2Limit:
.space   4

.end

