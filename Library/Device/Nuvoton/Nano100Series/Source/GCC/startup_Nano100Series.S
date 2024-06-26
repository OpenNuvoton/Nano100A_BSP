/****************************************************************************//**
 * @file     startup_Nano100Series.S
 * @version  V1.00
 * @brief    CMSIS Cortex-M0 Core Device Startup File for Nano100
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/



	.syntax	unified
	.arch	armv6-m

	.section .stack
	.align	3
#ifdef __STACK_SIZE
	.equ	Stack_Size, __STACK_SIZE
#else
	.equ	Stack_Size, 0x00000400
#endif
	.globl	__StackTop
	.globl	__StackLimit
__StackLimit:
	.space	Stack_Size
	.size	__StackLimit, . - __StackLimit
__StackTop:
	.size	__StackTop, . - __StackTop

	.section .heap
	.align	3
#ifdef __HEAP_SIZE
	.equ	Heap_Size, __HEAP_SIZE
#else
	.equ	Heap_Size, 0x00000100
#endif
	.globl	__HeapBase
	.globl	__HeapLimit
__HeapBase:
	.if	Heap_Size
	.space	Heap_Size
	.endif
	.size	__HeapBase, . - __HeapBase
__HeapLimit:
	.size	__HeapLimit, . - __HeapLimit

	.section .vectors
	.align	2
	.globl	__Vectors
__Vectors:
	.long	__StackTop            /* Top of Stack */
	.long	Reset_Handler         /* Reset Handler */
	.long	NMI_Handler           /* NMI Handler */
	.long	HardFault_Handler     /* Hard Fault Handler */
	.long	MemManage_Handler     /* MPU Fault Handler */
	.long	BusFault_Handler      /* Bus Fault Handler */
	.long	UsageFault_Handler    /* Usage Fault Handler */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	SVC_Handler           /* SVCall Handler */
	.long	DebugMon_Handler      /* Debug Monitor Handler */
	.long	0                     /* Reserved */
	.long	PendSV_Handler        /* PendSV Handler */
	.long	SysTick_Handler       /* SysTick Handler */

	/* External interrupts */
	.long BOD_IRQHandler        /* Brownout low voltage detected interrupt           */
	.long WDT_IRQHandler        /* Watch Dog Timer interrupt                         */
	.long EINT0_IRQHandler      /* External signal interrupt from PB.14 pin          */
	.long EINT1_IRQHandler      /* External signal interrupt from PB.15 pin          */
 	.long GPABC_IRQHandler      /* External interrupt from PA[15:0]/PB[15:0]/PC[15:0]*/
 	.long GPDEF_IRQHandler      /* External interrupt from PD[15:0]/PE[15:0]/PF[7:0] */
 	.long PWM0_IRQHandler       /* PWM 0 interrupt                                   */
 	.long PWM1_IRQHandler       /* PWM 1 interrupt                                   */
 	.long TMR0_IRQHandler       /* Timer 0 interrupt                                 */
 	.long TMR1_IRQHandler       /* Timer 1 interrupt                                 */
 	.long TMR2_IRQHandler       /* Timer 2 interrupt                                 */
 	.long TMR3_IRQHandler       /* Timer 3 interrupt                                 */
 	.long UART0_IRQHandler      /* UART0 interrupt                                   */
 	.long UART1_IRQHandler      /* UART1 interrupt                                   */
 	.long SPI0_IRQHandler       /* SPI0 interrupt                                    */
 	.long SPI1_IRQHandler       /* SPI1 interrupt                                    */  
 	.long SPI2_IRQHandler       /* SPI2 interrupt                                    */
 	.long HIRC_IRQHandler       /* HIRC interrupt                                    */
 	.long I2C0_IRQHandler       /* I2C0 interrupt                                    */
 	.long I2C1_IRQHandler       /* I2C1 interrupt                                    */
 	.long 0                     /* Reserved                                          */
 	.long SC0_IRQHandler        /* SC0 interrupt                                     */
 	.long SC1_IRQHandler        /* SC1 interrupt                                     */
 	.long USBD_IRQHandler       /* USB FS Device interrupt                           */
 	.long 0                     /* Reserved                                          */
 	.long 0                     /* Reserved                                          */
 	.long PDMA_IRQHandler       /* PDMA interrupt                                    */
 	.long I2S_IRQHandler        /* I2S interrupt                                     */
 	.long PDWU_IRQHandler       /* Power Down Wake up interrupt                      */
	.long ADC_IRQHandler        /* ADC interrupt                                     */
	.long 0                     /* Reserved                                          */
	.long RTC_IRQHandler        /* Real time clock interrupt                         */
	.size	__Vectors, . - __Vectors

	.text
	.thumb
	.thumb_func
	.align	2
	.globl	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:
/*  Firstly it copies data from read only memory to RAM. There are two schemes
 *  to copy. One can copy more than one sections. Another can only copy
 *  one section.  The former scheme needs more instructions and read-only
 *  data to implement than the latter.
 *  Macro __STARTUP_COPY_MULTIPLE is used to choose between two schemes.  */

#ifdef __STARTUP_COPY_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of triplets, each of which specify:
 *    offset 0: LMA of start of a section to copy from
 *    offset 4: VMA of start of a section to copy to
 *    offset 8: size of the section to copy. Must be multiply of 4
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
	ldr	r4, =__copy_table_start__
	ldr	r5, =__copy_table_end__

.L_loop0:
	cmp	r4, r5
	bge	.L_loop0_done
	ldr	r1, [r4]
	ldr	r2, [r4, #4]
	ldr	r3, [r4, #8]

.L_loop0_0:
	subs	r3, #4
	ittt	ge
	ldrge	r0, [r1, r3]
	strge	r0, [r2, r3]
	bge	.L_loop0_0

	adds	r4, #12
	b	.L_loop0

.L_loop0_done:
#else
/*  Single section scheme.
 *
 *  The ranges of copy from/to are specified by following symbols
 *    __etext: LMA of start of the section to copy from. Usually end of text
 *    __data_start__: VMA of start of the section to copy to
 *    __data_end__: VMA of end of the section to copy to
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
	ldr	r1, =__etext
	ldr	r2, =__data_start__
	ldr	r3, =__data_end__

	subs	r3, r2
	ble	.L_loop1_done

.L_loop1:
	subs	r3, #4
	ldr	r0, [r1,r3]
	str	r0, [r2,r3]
	bgt	.L_loop1

.L_loop1_done:
#endif /*__STARTUP_COPY_MULTIPLE */

/*  This part of work usually is done in C library startup code. Otherwise,
 *  define this macro to enable it in this startup.
 *
 *  There are two schemes too. One can clear multiple BSS sections. Another
 *  can only clear one section. The former is more size expensive than the
 *  latter.
 *
 *  Define macro __STARTUP_CLEAR_BSS_MULTIPLE to choose the former.
 *  Otherwise efine macro __STARTUP_CLEAR_BSS to choose the later.
 */
#ifdef __STARTUP_CLEAR_BSS_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of tuples specifying:
 *    offset 0: Start of a BSS section
 *    offset 4: Size of this BSS section. Must be multiply of 4
 */
	ldr	r3, =__zero_table_start__
	ldr	r4, =__zero_table_end__

.L_loop2:
	cmp	r3, r4
	bge	.L_loop2_done
	ldr	r1, [r3]
	ldr	r2, [r3, #4]
	movs	r0, 0

.L_loop2_0:
	subs	r2, #4
	itt	ge
	strge	r0, [r1, r2]
	bge	.L_loop2_0

	adds	r3, #8
	b	.L_loop2
.L_loop2_done:
#elif defined (__STARTUP_CLEAR_BSS)
/*  Single BSS section scheme.
 *
 *  The BSS section is specified by following symbols
 *    __bss_start__: start of the BSS section.
 *    __bss_end__: end of the BSS section.
 *
 *  Both addresses must be aligned to 4 bytes boundary.
 */
	ldr	r1, =__bss_start__
	ldr	r2, =__bss_end__

	movs	r0, 0
.L_loop3:
	cmp	r1, r2
	itt	lt
	strlt	r0, [r1], #4
	blt	.L_loop3
#endif /* __STARTUP_CLEAR_BSS_MULTIPLE || __STARTUP_CLEAR_BSS */

/*  Unlock Register */
	ldr	r0, =0x50000100
	ldr	r1, =0x59
	str	r1, [r0]
	ldr	r1, =0x16
	str	r1, [r0]
	ldr	r1, =0x88
	str	r1, [r0]

/* Init POR */
	ldr	r0, =0x50000060
	ldr	r1, =0x00005AA5
	str	r1, [r0]
Lock:

/* Lock register */
	ldr	r0, =0x50000100
	ldr	r1, =0
	str	r1, [r0]

#ifndef __START
#define __START _start
#endif
	bl	__START

	.pool
	.size	Reset_Handler, . - Reset_Handler

	.align	1
	.thumb_func
	.weak	Default_Handler
	.type	Default_Handler, %function
Default_Handler:
	b	.
	.size	Default_Handler, . - Default_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
	.macro	def_irq_handler	handler_name
	.weak	\handler_name
	.set	\handler_name, Default_Handler
	.endm

	def_irq_handler	NMI_Handler
	def_irq_handler	HardFault_Handler
	def_irq_handler	MemManage_Handler
	def_irq_handler	BusFault_Handler
	def_irq_handler	UsageFault_Handler
	def_irq_handler	SVC_Handler
	def_irq_handler	DebugMon_Handler
	def_irq_handler	PendSV_Handler
	def_irq_handler	SysTick_Handler

 	def_irq_handler BOD_IRQHandler
 	def_irq_handler WDT_IRQHandler
	def_irq_handler EINT0_IRQHandler
	def_irq_handler EINT1_IRQHandler
	def_irq_handler GPABC_IRQHandler
	def_irq_handler GPDEF_IRQHandler
	def_irq_handler PWM0_IRQHandler
	def_irq_handler PWM1_IRQHandler
	def_irq_handler TMR0_IRQHandler
	def_irq_handler TMR1_IRQHandler
	def_irq_handler TMR2_IRQHandler
	def_irq_handler TMR3_IRQHandler
	def_irq_handler UART0_IRQHandler
	def_irq_handler UART1_IRQHandler
	def_irq_handler SPI0_IRQHandler
	def_irq_handler SPI1_IRQHandler
	def_irq_handler SPI2_IRQHandler
	def_irq_handler HIRC_IRQHandler
	def_irq_handler I2C0_IRQHandler
	def_irq_handler I2C1_IRQHandler
	def_irq_handler SC0_IRQHandler
	def_irq_handler SC1_IRQHandler
	def_irq_handler USBD_IRQHandler
	def_irq_handler PDMA_IRQHandler
	def_irq_handler I2S_IRQHandler
	def_irq_handler PDWU_IRQHandler
	def_irq_handler ADC_IRQHandler
	def_irq_handler RTC_IRQHandler

	.end
