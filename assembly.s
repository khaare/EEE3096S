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
    @ Read what switches are pressed
    LDR   R0, GPIOA_BASE
    LDR   R3, [R0, #0x10]   @ GPIOA IDR

    @ Task 5: Check if SW3 is pressed to freeze everything
    MOVS  R5, R3
    LSRS  R5, R5, #3         @ Shift SW3 bit to position 0
    MOVS  R7, #1
    ANDS  R5, R5, R7         @ Isolate just that bit
    CMP   R5, #0
    BEQ   freeze_pattern     @ If pressed, freeze the current pattern


    @ Task 4: Check if SW2 is pressed to set pattern to 0xAA
    MOVS  R5, R3
    LSRS  R5, R5, #2         @ Shift SW2 bit to position 0
    MOVS  R7, #1
    ANDS  R5, R5, R7         @ Isolate just that bit
    CMP   R5, #0
    BEQ   set_to_AA          @ If pressed, jump to 0xAA pattern


    @ Extract SW0 and SW1 bits for normal operation
    MOVS  R5, R3             @ Get SW0 status
    LSRS  R5, R5, #0
    MOVS  R7, #1
    ANDS  R5, R5, R7
    MOVS  R6, R3             @ Get SW1 status
    LSRS  R6, R6, #1
    ANDS  R6, R6, R7


    @ Figure out which switches are pressed
    CMP   R5, #0
    BNE   check_sw1_only     @ SW0 not pressed, check SW1
    CMP   R6, #0
    BNE   sw0_only           @ Only SW0 is pressed
    B     both_pressed       @ Both switches pressed


check_sw1_only:
    CMP   R6, #0
    BNE   default            @ No switches pressed
    B     sw1_only           @ Only SW1 pressed


@ Task 5: Freeze - don't change the LED pattern at all
freeze_pattern:
    MOVS  R4, #0             @ Set increment to zero
    BL    long_delay
    B     write_leds


@ Task 4: Set LEDs to 0xAA pattern but still respond to other switches
set_to_AA:
    MOVS  R2, #0xAA          @ Set the special pattern

    @ Even with 0xAA, SW0/SW1 still control timing and increment
    MOVS  R5, R3
    LSRS  R5, R5, #0
    MOVS  R7, #1
    ANDS  R5, R5, R7
    MOVS  R6, R3
    LSRS  R6, R6, #1
    ANDS  R6, R6, R7

    CMP   R5, #0
    BNE   aa_check_sw1_only
    CMP   R6, #0
    BNE   aa_sw0_only
    B     aa_both_pressed


aa_check_sw1_only:
    CMP   R6, #0
    BNE   aa_default
    B     aa_sw1_only


@ 0xAA pattern behaviors with different switch combinations
aa_both_pressed:
    MOVS  R4, #2             @ Increment by 2, fast timing
    BL    short_delay
    B     write_leds

aa_sw0_only:
    MOVS  R4, #2             @ Increment by 2, slow timing
    BL    long_delay
    B     write_leds

aa_sw1_only:
    MOVS  R4, #1             @ Increment by 1, fast timing
    BL    short_delay
    B     write_leds

aa_default:
    MOVS  R4, #1             @ Increment by 1, slow timing
    BL    long_delay
    B     write_leds


@ Normal switch behaviors
both_pressed:
    MOVS  R4, #2             @ SW0+SW1: count by 2, fast
    BL    short_delay
    B     write_leds

sw0_only:
    MOVS  R4, #2             @ SW0 only: count by 2, slow
    BL    long_delay
    B     write_leds

sw1_only:
    MOVS  R4, #1             @ SW1 only: count by 1, fast
    BL    short_delay
    B     write_leds

default:
    MOVS  R4, #1             @ No switches: count by 1, slow
    BL    long_delay
    B     write_leds


@ Update the LEDs and loop back
write_leds:
    STR   R2, [R1, #0x14]    @ Show current pattern on LEDs
    ADDS  R2, R2, R4         @ Add the increment for next time
    B     main_loop          @ Do it all again

@ Timing delays
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
SHORT_DELAY_CNT: 	.word 600000
