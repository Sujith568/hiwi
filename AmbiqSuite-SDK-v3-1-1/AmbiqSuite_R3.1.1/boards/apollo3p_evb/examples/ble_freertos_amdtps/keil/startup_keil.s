;******************************************************************************
;
;! @file startup_keil.s
;!
;! @brief Definitions for Apollo3 Blue Plus interrupt handlers, the vector
;! table, and the stack.
;
;******************************************************************************

;******************************************************************************
;
; Copyright (c) 2023, Ambiq Micro, Inc.
; All rights reserved.
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions are met:
;
; 1. Redistributions of source code must retain the above copyright notice,
; this list of conditions and the following disclaimer.
;
; 2. Redistributions in binary form must reproduce the above copyright
; notice, this list of conditions and the following disclaimer in the
; documentation and/or other materials provided with the distribution.
;
; 3. Neither the name of the copyright holder nor the names of its
; contributors may be used to endorse or promote products derived from this
; software without specific prior written permission.
;
; Third party software included in this distribution is subject to the
; additional license terms as defined in the /docs/licenses directory.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
; POSSIBILITY OF SUCH DAMAGE.
;
; This is part of revision release_sdk_3_1_1-10cda4b5e0 of the AmbiqSuite Development Package.
;
;******************************************************************************

;******************************************************************************
;
; <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
;************************************************************************
Stack   EQU     0x00003000

;******************************************************************************
;
; <o> Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
;
;******************************************************************************
Heap    EQU     0x00000000

;******************************************************************************
;
; Allocate space for the stack.
;
;******************************************************************************
        AREA    STACK, NOINIT, READWRITE, ALIGN=3
StackMem
        SPACE   Stack
__initial_sp

;******************************************************************************
;
; Allocate space for the heap.
;
;******************************************************************************
        AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
HeapMem
        SPACE   Heap
__heap_limit

;******************************************************************************
;
; Indicate that the code in this file preserves 8-byte alignment of the stack.
;
;******************************************************************************
        PRESERVE8

;******************************************************************************
;
; Place code into the reset code section.
;
;******************************************************************************
        AREA    RESET, CODE, READONLY
        THUMB

;******************************************************************************
;
; The vector table.
;
;******************************************************************************
;
; Note: Aliasing and weakly exporting am_mpufault_isr, am_busfault_isr, and
; am_usagefault_isr does not work if am_fault_isr is defined externally.
; Therefore, we'll explicitly use am_fault_isr in the table for those vectors.
;

        EXPORT  __Vectors
__Vectors
        DCD     StackMem + Stack            ; Top of Stack
        DCD     Reset_Handler               ; Reset Handler
        DCD     NMI_Handler                 ; NMI Handler
        DCD     HardFault_Handler           ; Hard Fault Handler
        DCD     MemManage_Handler           ; The MPU fault handler
        DCD     BusFault_Handler            ; The bus fault handler
        DCD     UsageFault_Handler          ; The usage fault handler
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     SVC_Handler                 ; SVCall handler
        DCD     DebugMon_Handler            ; Debug monitor handler
        DCD     0                           ; Reserved
        DCD     PendSV_Handler              ; The PendSV handler
        DCD     SysTick_Handler             ; The SysTick handler

        ;
        ; Peripheral Interrupts
        ;
        DCD     am_brownout_isr             ;  0: Reserved
        DCD     am_watchdog_isr             ;  1: Reserved
        DCD     am_rtc_isr                  ;  2: RTC
        DCD     am_vcomp_isr                ;  3: Voltage Comparator
        DCD     am_ioslave_ios_isr          ;  4: I/O Slave general
        DCD     am_ioslave_acc_isr          ;  5: I/O Slave access
        DCD     am_iomaster0_isr            ;  6: I/O Master 0
        DCD     am_iomaster1_isr            ;  7: I/O Master 1
        DCD     am_iomaster2_isr            ;  8: I/O Master 2
        DCD     am_iomaster3_isr            ;  9: I/O Master 3
        DCD     am_iomaster4_isr            ; 10: I/O Master 4
        DCD     am_iomaster5_isr            ; 11: I/O Master 5
        DCD     am_ble_isr                  ; 12: BLEIF
        DCD     am_gpio_isr                 ; 13: GPIO
        DCD     am_ctimer_isr               ; 14: CTIMER
        DCD     am_uart_isr                 ; 15: UART0
        DCD     am_uart1_isr                ; 16: UART1
        DCD     am_scard_isr                ; 17: SCARD
        DCD     am_adc_isr                  ; 18: ADC
        DCD     am_pdm0_isr                 ; 19: PDM
        DCD     am_mspi0_isr                ; 20: MSPI0
        DCD     am_software0_isr            ; 21: SOFTWARE0
        DCD     am_stimer_isr               ; 22: SYSTEM TIMER
        DCD     am_stimer_cmpr0_isr         ; 23: SYSTEM TIMER COMPARE0
        DCD     am_stimer_cmpr1_isr         ; 24: SYSTEM TIMER COMPARE1
        DCD     am_stimer_cmpr2_isr         ; 25: SYSTEM TIMER COMPARE2
        DCD     am_stimer_cmpr3_isr         ; 26: SYSTEM TIMER COMPARE3
        DCD     am_stimer_cmpr4_isr         ; 27: SYSTEM TIMER COMPARE4
        DCD     am_stimer_cmpr5_isr         ; 28: SYSTEM TIMER COMPARE5
        DCD     am_stimer_cmpr6_isr         ; 29: SYSTEM TIMER COMPARE6
        DCD     am_stimer_cmpr7_isr         ; 30: SYSTEM TIMER COMPARE7
        DCD     am_clkgen_isr               ; 31: CLKGEN
        DCD     am_mspi1_isr                ; 32: MSPI1
        DCD     am_mspi2_isr                ; 33: MSPI2

__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

;******************************************************************************
;
; Place code immediately following vector table.
;
;******************************************************************************
;******************************************************************************
;
; The Patch table.
;
; The patch table should pad the vector table size to a total of 64 entries
; (16 core + 48 periph) such that code begins at offset 0x100.
;
;******************************************************************************
        EXPORT  __Patchable
__Patchable
        DCD     0                           ; 34
        DCD     0                           ; 35
        DCD     0                           ; 36
        DCD     0                           ; 37
        DCD     0                           ; 38
        DCD     0                           ; 39
        DCD     0                           ; 40
        DCD     0                           ; 41
        DCD     0                           ; 42
        DCD     0                           ; 43
        DCD     0                           ; 44
        DCD     0                           ; 45
        DCD     0                           ; 46
        DCD     0                           ; 47

;******************************************************************************
;
; This is the code that gets called when the processor first starts execution
; following a reset event.
;
;******************************************************************************
Reset_Handler   PROC
                EXPORT  Reset_Handler               [WEAK]
                IMPORT  __main

                ;
                ; Enable the FPU.
                ;
                MOVW    R0, #0xED88
                MOVT    R0, #0xE000
                LDR     R1, [R0]
                ORR     R1, #0x00F00000
                STR     R1, [R0]
                DSB
                ISB

                ;
                ; Branch to main.
                ;
                LDR     R0, =__main
                BX      R0

                ENDP

;******************************************************************************
;
; Weak Exception Handlers.
;
;******************************************************************************

HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler           [WEAK]
                B       .
                ENDP
NMI_Handler     PROC
                EXPORT  NMI_Handler                 [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler           [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler            [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler          [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler                 [WEAK]
                B       .
                ENDP
DebugMon_Handler  PROC
                EXPORT  DebugMon_Handler            [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler              [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler             [WEAK]
                B       .
                ENDP

am_default_isr\
                PROC
                EXPORT  am_brownout_isr             [WEAK]
                EXPORT  am_watchdog_isr             [WEAK]
                EXPORT  am_rtc_isr                  [WEAK]
                EXPORT  am_vcomp_isr                [WEAK]
                EXPORT  am_ioslave_ios_isr          [WEAK]
                EXPORT  am_ioslave_acc_isr          [WEAK]
                EXPORT  am_iomaster0_isr            [WEAK]
                EXPORT  am_iomaster1_isr            [WEAK]
                EXPORT  am_iomaster2_isr            [WEAK]
                EXPORT  am_iomaster3_isr            [WEAK]
                EXPORT  am_iomaster4_isr            [WEAK]
                EXPORT  am_iomaster5_isr            [WEAK]
                EXPORT  am_ble_isr                  [WEAK]
                EXPORT  am_gpio_isr                 [WEAK]
                EXPORT  am_ctimer_isr               [WEAK]
                EXPORT  am_uart_isr                 [WEAK]
                EXPORT  am_uart0_isr                [WEAK]
                EXPORT  am_uart1_isr                [WEAK]
                EXPORT  am_scard_isr                [WEAK]
                EXPORT  am_adc_isr                  [WEAK]
                EXPORT  am_pdm0_isr                 [WEAK]
                EXPORT  am_mspi0_isr                [WEAK]
                EXPORT  am_software0_isr            [WEAK]
                EXPORT  am_stimer_isr               [WEAK]
                EXPORT  am_stimer_cmpr0_isr         [WEAK]
                EXPORT  am_stimer_cmpr1_isr         [WEAK]
                EXPORT  am_stimer_cmpr2_isr         [WEAK]
                EXPORT  am_stimer_cmpr3_isr         [WEAK]
                EXPORT  am_stimer_cmpr4_isr         [WEAK]
                EXPORT  am_stimer_cmpr5_isr         [WEAK]
                EXPORT  am_stimer_cmpr6_isr         [WEAK]
                EXPORT  am_stimer_cmpr7_isr         [WEAK]
                EXPORT  am_clkgen_isr               [WEAK]
                EXPORT  am_mspi1_isr                [WEAK]
                EXPORT  am_mspi2_isr                [WEAK]

am_brownout_isr
am_watchdog_isr
am_rtc_isr
am_vcomp_isr
am_ioslave_ios_isr
am_ioslave_acc_isr
am_iomaster0_isr
am_iomaster1_isr
am_iomaster2_isr
am_iomaster3_isr
am_iomaster4_isr
am_iomaster5_isr
am_ble_isr
am_gpio_isr
am_ctimer_isr
am_uart_isr
am_uart0_isr
am_uart1_isr
am_scard_isr
am_adc_isr
am_pdm0_isr
am_mspi0_isr
am_software0_isr
am_stimer_isr
am_stimer_cmpr0_isr
am_stimer_cmpr1_isr
am_stimer_cmpr2_isr
am_stimer_cmpr3_isr
am_stimer_cmpr4_isr
am_stimer_cmpr5_isr
am_stimer_cmpr6_isr
am_stimer_cmpr7_isr
am_clkgen_isr
am_mspi1_isr
am_mspi2_isr

                ; all device interrupts go here unless the weak label is over
                ; ridden in the linker hard spin so the debugger will know it
                ; was an unhandled interrupt request a come-from-buffer or
                ; instruction trace hardware would sure be nice if you get here
                B       .

                ENDP

;******************************************************************************
;
; Align the end of the section.
;
;******************************************************************************
                ALIGN

;******************************************************************************
;
; Initialization of the heap and stack.
;
;******************************************************************************
                AREA    |.text|, CODE, READONLY

;******************************************************************************
;
; User Initial Stack & Heap.
;
;******************************************************************************
    IF :DEF: __MICROLIB
        EXPORT  __initial_sp
        EXPORT  __heap_base
        EXPORT  __heap_limit
    ELSE
        IMPORT  __use_two_region_memory
        EXPORT  __user_initial_stackheap
__user_initial_stackheap PROC
        LDR     R0, =HeapMem
        LDR     R1, =(StackMem + Stack)
        LDR     R2, =(HeapMem + Heap)
        LDR     R3, =StackMem
        BX      LR

        ENDP

    ENDIF

;******************************************************************************
;
; Align the end of the section.
;
;******************************************************************************
                ALIGN

;******************************************************************************
;
; All Done
;
;******************************************************************************
                END


