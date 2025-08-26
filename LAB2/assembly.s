/*
 * assembly.s
 *
 */
 
 @ DO NOT EDIT
	.syntax unified
    .text @code starts from here
    .global ASM_Main @ASM_Main tells the compliter that this is where the first instruction starts. first label
    .thumb_func @enables thumb instructions

@ DO NOT EDIT
vectors:
	.word 0x20002000 @allocates 32 bit of memory and puts stuff in there that uses .word
	.word ASM_Main + 1 

@ DO NOT EDIT label ASM_Main
ASM_Main:

	@ Some code is given below for you to start with
	LDR R0, RCC_BASE  		@ Enable clock for GPIOA and B by setting bit 17 and 18 in RCC_AHBENR
	LDR R1, [R0, #0x14]
	LDR R2, AHBENR_GPIOAB	@ AHBENR_GPIOAB is defined under LITERALS at the end of the code
	ORRS R1, R1, R2
	STR R1, [R0, #0x14]

	LDR R0, GPIOA_BASE		@ Enable pull-up resistors for pushbuttons
	MOVS R1, #0b01010101
	STR R1, [R0, #0x0C]
	LDR R1, GPIOB_BASE  	@ Set pins connected to LEDs to outputs
	LDR R2, MODER_OUTPUT
	STR R2, [R1, #0]
	MOVS R2, #0         	@ NOTE: R2 will be dedicated to holding the value on the LEDs

@ TODO: Add code, labels and logic for button checks and LED patterns

main_loop:

    @  Read switches 
    LDR   R0, GPIOA_BASE
    LDR   R3, [R0, #0x10]   @ GPIOA IDR

    @ Check both pressed 
    MOVS  R5, #3			@ sw0 + sw1 mask
    TST   R5, R3
    BEQ   both_pressed      @ move if both pressed

    MOVS  R5, #1			@ sw0 mask
    TST   R5, R3
    BEQ   sw0_only			@ move if sw0 pressed

    MOVS  R5, #2			@ sw1 mask
    TST   R5, R3
    BEQ   sw1_only             @ move if sw1 pressed
    B     default			@ do default otherwises

check_sw1_only:
    CMP   R6, #0
    BEQ   write_leds           @ neither pressed
    B     sw1_only

both_pressed:
    MOVS  R4, #2
    BL    short_delay
    B     write_leds

sw0_only:
    MOVS  R4, #2
    BL    long_delay
    B     write_leds

sw1_only:
    MOVS  R4, #1
    BL    short_delay
    B     write_leds

default:
    @ Defaults: increment=1, long delay
    MOVS  R4, #1
    BL    long_delay
    B 	  write_leds

write_leds:
	ADDS  R2, R2, R4
    STR   R2, [R1, #0x14]     @ Write to LEDs
    B     main_loop

@ Delay routines 
long_delay:
    LDR   R5, LONG_DELAY_CNT
long_loop:
    SUBS  R5, R5, #1
    BNE   long_loop
    BX    LR

short_delay:
    LDR   R5, SHORT_DELAY_CNT
short_loop:
    SUBS  R5, R5, #1
    BNE   short_loop
    BX    LR


@ LITERALS; DO NOT EDIT
	.align
RCC_BASE: 			.word 0x40021000
AHBENR_GPIOAB: 		.word 0b1100000000000000000
GPIOA_BASE:  		.word 0x48000000
GPIOB_BASE:  		.word 0x48000400
MODER_OUTPUT: 		.word 0x5555

@ TODO: Add your own values for these delays
LONG_DELAY_CNT: 	.word 1400000 @frequency/cycles*delay
SHORT_DELAY_CNT: 	.word 300000
