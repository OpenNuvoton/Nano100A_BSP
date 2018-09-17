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


	AREA _image, DATA, READONLY

	EXPORT  loaderImage1Base
	EXPORT  loaderImage1Limit
	EXPORT  loaderImage2Base
	EXPORT  loaderImage2Limit

	ALIGN   4

loaderImage1Base
	INCBIN ./obj/fmc_ld_boot.bin
loaderImage1Limit

loaderImage2Base
	INCBIN ./obj/fmc_isp.bin
loaderImage2Limit


	END
