; THIS PROGRAM IS PROVIDED ”AS IS”. TI MAKES NO WARRANTIES OR
; REPRESENTATIONS, EITHER EXPRESS, IMPLIED OR STATUTORY,
; INCLUDING ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
; FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR
; COMPLETENESS OF RESPONSES, RESULTS AND LACK OF NEGLIGENCE.
; TI DISCLAIMS ANY WARRANTY OF TITLE, QUIET ENJOYMENT, QUIET
; POSSESSION, AND NON–INFRINGEMENT OF ANY THIRD PARTY
; INTELLECTUAL PROPERTY RIGHTS WITH REGARD TO THE PROGRAM OR
; YOUR USE OF THE PROGRAM.
;
; IN NO EVENT SHALL TI BE LIABLE FOR ANY SPECIAL, INCIDENTAL,
; CONSEQUENTIAL OR INDIRECT DAMAGES, HOWEVER CAUSED, ON ANY
; THEORY OF LIABILITY AND WHETHER OR NOT TI HAS BEEN ADVISED
; OF THE POSSIBILITY OF SUCH DAMAGES, ARISING IN ANY WAY OUT
; OF THIS AGREEMENT, THE PROGRAM, OR YOUR USE OF THE PROGRAM.
; EXCLUDED DAMAGES INCLUDE, BUT ARE NOT LIMITED TO, COST OF
; REMOVAL OR REINSTALLATION, COMPUTER TIME, LABOR COSTS, LOSS
; OF GOODWILL, LOSS OF PROFITS, LOSS OF SAVINGS, OR LOSS OF
; USE OR INTERRUPTION OF BUSINESS. IN NO EVENT WILL TI’S
; AGGREGATE LIABILITY UNDER THIS AGREEMENT OR ARISING OUT OF
; YOUR USE OF THE PROGRAM EXCEED FIVE HUNDRED DOLLARS
; (U.S.$500).
;
; Unless otherwise stated, the Program written and copyrighted
; by Texas Instruments is distributed as ”freeware”. You may,
; only under TI’s copyright in the Program, use and modify the
; Program without any charge or restriction. You may
; distribute to third parties, provided that you transfer a
; copy of this license to the third party and the third party
; agrees to these terms by its first use of the Program. You
; must reproduce the copyright notice and any other legend of
; ownership on each copy or partial copy, of the Program.
;
; You acknowledge and agree that the Program contains
; copyrighted material, trade secrets and other TI proprietary
; information and is protected by copyright laws,
; international copyright treaties, and trade secret laws, as
; well as other intellectual property laws. To protect TI’s
; rights in the Program, you agree not to decompile, reverse
; engineer, disassemble or otherwise translate any object code
; versions of the Program to a human–readable form. You agree
; that in no event will you alter, remove or destroy any
; copyright notice included in the Program. TI reserves all
; rights not specifically granted under this license. Except
; as specifically provided herein, nothing in this agreement
; shall be construed as conferring by implication, estoppel,
; or otherwise, upon you, any license or other right under any
; TI patents, copyrights or trade secrets.
;
; You may not use the Program in non–TI devices.
;
;******************************************************************************
NAME ULTRASONIC_DISTANCE_MEASUREMENT
;AUTHOR Murugavel Raju
; MSP430 Applications
; Texas Instruments Inc.
; Feb 2001
#include ”msp430x41x.h” ; Standard Equations
;
;******************************************************************************
; MSP430F413 Ultrasonic Distance Measurement Demonstration Program
;
;******************************************************************************
;Register definitions
;******************************************************************************
#define DIGITS R11
#define Result R10
#define IRBT R9
#define IROP1 R4
#define IROP2L R5
#define IROP2M R6
#define IRACL R7
#define IRACM R8
;******************************************************************************
;Variables definition
;******************************************************************************
RSEG UDATA0
htX100_msw: DS 2 					; word variable stored in RAM 200h & 201h
htX100_lsw: DS 2 					; 202h & 203h
;******************************************************************************
RSEG CSTACK 						; Directive to begin stack segment
DS 0
RSEG CODE 							; Directive to begin code segment
RESET mov.w #SFE(CSTACK),SP 		; Define stack pointer
call #Init_Device 					; Initialize device
mov.w #0,DIGITS 					; Initialize DIGITS to ’0’
Mainloop
bic.b #CAON,&CACTL1 				; Comparator_A OFF
call #Display 						; Display Data on LCD
bis.w #LPM3,SR 						; Wait in LPM3
;***************Start Ultrasonic Bursts and take measurements *****************
clr.w &CCTL1 						; Disable CCTL1
clr.w &TACTL 						; Disable timer_A
bis.b #BIT0,&P1OUT 					; LED ON
SetupTimerA mov.w #TASSEL0+TACLR+MC1,&TACTL
; TACLK = ACLK, 16 bit up mode
bis.b #BIT5,&P1SEL 					; ACLK o/p on P1.5
mov.w #12,&CCR1 					; 12 cycle 40KHz burst
mov.w #CCIE,&CCTL1 					; Compare mode,interrupt
bis.w #LPM0,SR 						; Wait for CCR1 interrupt
bic.b #BIT5,&P1SEL 					; ACLK o/p on P1.5 OFF
TimerCLR bis.w #TACLR,&TACTL
mov.w #36,&CCR1 					; Delay for transducer to settle
mov.w #CCIE,&CCTL1 					; Compare mode,interrupt
bis.w #LPM0,SR 						; Wait for CCR1 interrupt
bis.b #CAON,&CACTL1 				; Comparator_A ON
bic.b #CAIFG,&CACTL1 				; Enable Comparator_A interrupt flag
mov.w #CM0+CCIS0+SCS+CAP+CCIE,&CCTL1
; Pos edge, CCIB,Cap,interrupt
push &TAR ; TOS = TAR at Start of measurement
bis.w #LPM0,SR 						; Wait for CCR1 interrupt (Echo)
clr.w &CCTL1 						; Disable CCTL1
bic.b #BIT0,&P1OUT 					; LED OFF
bit.b #CAIFG,&CACTL1 				; Check for Echo not received
jz Next 							; ’out of range’ condition
mov.w &CCR1,Result 					; Result = TAR (CCR1) at EOC
sub.w @SP+,Result 					; Result = time taken
add.w #48,Result 					; compensate 12Clks for the burst
									; transmission time + 36Clks delay
;****************** Measurement Done ******************************************
call #Math_calc 					; Call Math subroutine
swpb DIGITS 						; Shift left by two digits for /100
jmp Mainloop 						; next measurement cycle
Next mov.w #0beh,DIGITS 			; No echo received display ’E’ error
jmp Mainloop
;******************************************************************************
Init_Device 						; Initialize MSP430x41x
;******************************************************************************
mov.w #WDTPW+WDTHOLD,&WDTCTL 		; Stop WDT
bis.b #030h,&FLL_CTL0 				; Turn on internal load capacitors
									; for the XTAL to start oscillation
call #Delay 						; Delay for oscillator to stabilize
mov.b #03fh,&SCFQCTL 				; MCLK = 40KhzX64 = 2.56Mhz
call #Delay 						; Delay for FLL to stabilize
SetupP1 mov.b #000h,&P1OUT 			; Clear P1 output register
bis.b #0bfh,&P1DIR 					; Unused pins as o/p’s
bis.b #040h,&P1SEL 					; Comp_A + i/p function
SetupP2 mov.b #000h,&P2OUT 			; Clear P2 output register
bis.b #0ffh,&P2DIR 					; Unused pins as o/p’s
SetupP6 mov.b #000h,&P6OUT 			; Clear P6 output register
bis.b #0ffh,&P6DIR 					; Unused pins as o/p’s
SetupBT mov.b #BTFRFQ0+BTFRFQ1+BTIP2+BTDIV,&BTCTL
									; Enable BT with 150Hz LCD freq.
									; and 205 millisecond interrupt
SetupCA mov.b #CAPD6,&CAPD 			; o/p buffer disable for comp i/p
mov.b #P2CA0,&CACTL2 				; P1.6 to Comp + input
mov.b #CARSEL+CAREF1+CAON,&CACTL1
									; Comp_A ON, 0.5Vcc int. reference
SetupLCD bis.b #LCDON+LCDSON+LCDSG0_7,LCDCTL
									; LCD module ON and in static mode
ClearLCD mov #15,R15 ; 15 LCD mem locations to clear
mov.b #LCDMEM,R14
Clear1 mov.b #0,0(R14) 				; Write zeros in LCD RAM locations
inc.b R14
dec R15 							; All LCD mem clear?
jnz Clear1 							; More LCD mem to clear go
bis.b #BTIE,&IE2 					; Enable Basic Timer interrupt
eint 								; Enable interrupts
ret
;******************************************************************************
BT_ISR 								; Basic Timer ISR, CPU returns
									; to active mode on RETI
;******************************************************************************
bic #LPM3,0(SP) 					; Clear LPM3 bits on TOS
reti 								; on return from interrupt
;******************************************************************************
TAX_ISR								; Common ISR for CCR1–4 and overflow
;******************************************************************************
add.w &TAIV,PC 						; Add TA interrupt offset to PC
reti 								; CCR0 – no source
jmp CCR1_ISR 						; CCR1
reti 								; CCR2
reti 								; CCR3
reti 								; CCR4
TA_over reti 						; Timer_A overflow
CCR1_ISR bic.w #CCIFG,&CCTL1
bic.w #LPM0,0(SP) 					; Exit LPM0 on reti
reti 								;
;******************************************************************************
Display 					;Subroutine to Display values DIGIT1 & DIGIT2
							;CPU Registers used R15, R14, R13 and R12, not saved
;******************************************************************************
mov.w #LCDM1,R15 					; R15 points to first LCD location
mov.b DIGITS,R14 					; LSD value moved to R14
OutLCD mov.b R14,R13 				; Copy value in R14 to R13
rra.b R13 							; Right Shift
rra.b R13 							; four times to
rra.b R13 							; swap
rra.b R13 							; nibbles
and.b #0Fh,R14 						; low nibble now in R14
and.b #0Fh,R13 						; high nibble now in R13
mov.b LCD_Tab(R14),R12 				; Low nibble to LCD digit 1
mov.b R12,0(R15) 					; Low nibble segments a & b to LCD
rra.w R12
inc.b R15
mov.b R12,0(R15) 					; Low nibble segments c & d to LCD
rra.w R12
inc.b R15
mov.b R12,0(R15) 					; Low nibble segments e & f to LCD
rra.w R12
inc.b R15
mov.b R12,0(R15) 					; Low nibble segments g & h to LCD
rra.w R12
inc.b R15
mov.b LCD_Tab(R13),R12 				; High nibble to LCD digit 2
mov.b R12,0(R15) 					; High nibble segments a & b to LCD
rra.w R12
inc.b R15
mov.b R12,0(R15) 					; High nibble segments c & d to LCD
rra.w R12
inc.b R15
mov.b R12,0(R15) 					; High nibble segments e & f to LCD
rra.w R12
inc.b R15
mov.b R12,0(R15) 					; High nibble segments g & h to LCD
rra.w R12
ret
;******************************************************************************
; LCD Type Definition
;******************************************************************************
;Segments definition
a equ 001h
b equ 010h
c equ 002h
d equ 020h
e equ 004h
f equ 040h
g equ 008h
h equ 080h
Blank equ 000h
LCD_Tab db a+b+c+d+e+f 				; Displays ”0”
db b+c 								; Displays ”1”
db a+b+d+e+g 						; Displays ”2”
db a+b+c+d+g 						; Displays ”3”
db b+c+f+g 							; Displays ”4”
db a+c+d+f+g 						; Displays ”5”
db a+c+d+e+f+g 						; Displays ”6”
db a+b+c 							; Displays ”7”
db a+b+c+d+e+f+g 					; Displays ”8”
db a+b+c+d+f+g 						; Displays ”9”
db a+b+c+e+f+g 						; Displays ”A”
db Blank 							; Displays Blank
db a+d+e+f 							; Displays ”C”
db b+c+d+e+g 						; Displays ”D” d
db a+d+e+f+g 						; Displays ”E”
db a+e+f+g 							; Displays ”F”
;******************************************************************************
Delay; Software delay
;******************************************************************************
push #0FFFFh 						; Delay to TOS
DL1 dec.w 0(SP) 					; Decrement TOS
jnz DL1 							; Delay over?
incd SP 							; Clean TOS
ret 								; Return from subroutine
;******************************************************************************
Math_calc; calculation subroutine
;******************************************************************************
mov.w #0h, DIGITS 					; Initialize DIGIT to 0
cmp.w #0h, Result 					; Check if Result count=0
jeq calc_over 						; Exit if 0
call #Mul100 						; Multiply Result count by 100
call #Divide 						; Divide the result with #06d
call #Hex2bcd 						; Convert 16bit binary to BCD number
									; Result xx.xx
calc_over ret 						; Return from subroutine
;******************************************************************************
Mul100 						;subroutine for multiplying Result with 100d
							;inputs Result 16bit and constant 64h (100d) 16bit
							;output 32bit htX100_msw & htX100_lsw
;******************************************************************************
mov.w #100,IROP1 					; Load IROP1 with 100 (multiplier)
mpyu clr.w htX100_lsw 				; Clear buffer for least
									; Significant word
clr.w htX100_msw 					; Clear buffer for most
									; Significant word
macu clr.w IROP2M 					; Clear multiplier high word
L$002 bit.w #1,IROP1 				; Test actual bit
jz L$01 							; If 0: do nothing
add.w Result,htX100_lsw 			; If 1: Add multiplier to Result
addc.w IROP2M,htX100_msw 			;
L$01 rla.w Result 					; Multiplier X 2
rlc.w IROP2M 						;
rrc.w IROP1 						; Next bit to test
jnz L$002 							; If bit in carry : finished
ret
;******************************************************************************
Divide 		;Subroutine for 32/16 bits division
			;inputs 32bit htX100_msw & htX100_lsw and #06 16bit, output DIGIT
16bit
;******************************************************************************
clr.w DIGITS 						; Clear buffer to hold new Result
mov.w #17,IRBT 						; Initialize loop counter
div1 cmp.w #06,htX100_msw 			; Compare divisor with dividend high
word
jlo div2 							; If less : jump to div2
sub.w #06,htX100_msw 				; Subtract 6 from high word
div2 rlc.w DIGITS 					; Rotate result left through carry 1
bit
jc div4 							; If carry set: finished
dec.w IRBT 							; Decrement bit counter
jz div3 							; If counter = 0 : finished
rla.w htX100_lsw 					; Dividend X 2
rlc.w htX100_msw 					;
jnc div1 							; If carry not set jump to step div1
sub.w #06,htX100_msw 				; Subtract 6 from high word
setc 								; Set carry
jmp div2 							; Jump to repeat
div3 clrc 							; Clear carry
div4 ret 							; Return from subroutine
;******************************************************************************
Hex2bcd 	;Subroutine for converting 16bit hexadecimal value to BCD value
			;input in DIGITS 16bit hexadecimal, output in DIGITS 16bit BCD
;******************************************************************************
mov #16,r9 							; R9 no of bits
clr r8 								; Clear R8
clr r7 								; Clear R7
L$1 rla DIGITS 						; Rotate left arithmetic DIGITS
dadd r7,r7 							; Add source and carry decimally
dadd r8,r8 							; to destination
dec r9 								; Decrement bit counter
jnz L$1 							; Is 16 bits over ?
mov r7,DIGITS 						; Result in DIGITS
ret 								; Return from subroutine
;******************************************************************************
COMMON INTVEC 						; MSP430x41x Interrupt vectors
;******************************************************************************
ORG BASICTIMER_VECTOR
BT_VEC DW BT_ISR 					; Basic Timer Vector
ORG TIMERA1_VECTOR 					; Timer_AX Vector
TIMA_VEC DW TAX_ISR 				;
ORG RESET_VECTOR
RESET_VEC DW RESET 					; POR, ext. Reset, Watchdog
END
Title