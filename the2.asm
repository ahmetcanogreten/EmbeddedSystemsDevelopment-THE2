LIST P=18F4620
    
#include <P18F4620.INC>

config OSC = HSPLL      ; Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
config FCMEN = OFF      ; Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
config IESO = OFF       ; Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

; CONFIG2L
config PWRT = ON        ; Power-up Timer Enable bit (PWRT enabled)
config BOREN = OFF      ; Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
config BORV = 3         ; Brown Out Reset Voltage bits (Minimum setting)

; CONFIG2H
config WDT = OFF        ; Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
config WDTPS = 32768    ; Watchdog Timer Postscale Select bits (1:32768)

; CONFIG3H
config CCP2MX = PORTC   ; CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
config PBADEN = OFF     ; PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
config LPT1OSC = OFF    ; Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
config MCLRE = ON       ; MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

; CONFIG4L
config STVREN = OFF     ; Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
config LVP = OFF        ; Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
config XINST = OFF      ; Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

; CONFIG5L
config CP0 = OFF        ; Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
config CP1 = OFF        ; Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
config CP2 = OFF        ; Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
config CP3 = OFF        ; Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

; CONFIG5H
config CPB = OFF        ; Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
config CPD = OFF        ; Data EEPROM Code Protection bit (Data EEPROM not code-protected)

; CONFIG6L
config WRT0 = OFF       ; Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
config WRT1 = OFF       ; Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
config WRT2 = OFF       ; Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
config WRT3 = OFF       ; Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

; CONFIG6H
config WRTC = OFF       ; Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
config WRTB = OFF       ; Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
config WRTD = OFF       ; Data EEPROM Write Protection bit (Data EEPROM not write-protected)

; CONFIG7L
config EBTR0 = OFF      ; Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
config EBTR1 = OFF      ; Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
config EBTR2 = OFF      ; Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
config EBTR3 = OFF      ; Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

; CONFIG7H
config EBTRB = OFF      ; Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

    
    
    
;********************************************************************************************************************************

    
myVariables udata_acs
outerState	res 1   ; bit0 -> initialState, bit1 -> writeState, bit2 -> reviewState, bit3 -> readState

count_n	    res 1   ;	    These three counters are used inside "wait_for_n_times_10ms" procedure.
count_256_1 res 1   ;	"count_256_1" and "count_256_2" are set in this procedure in a way that with 40MHz
count_256_2 res 1   ;	clock speed, waiting time is approximately "count_n" * 10ms.
 
rb4ButtonState	res 1	; bit0 -> definetlyReleased, bit1 -> mightBePressed, bit2 -> definetlyPressed, bit3 -> mightBeReleased
		
countDownCounterOnes	res 1	;
countDownCounterTens	res 1	;
	
whichScreen	res 1	    ; This register is the state of the nextScreen to be drawn.
			    ; Drawing is achieved by --ms timer interrupt.
			    ; Screens from left to right correspond to
			    ; bit0 -> screen1, bit1 -> screen2, bit2 -> screen3, bit3 -> screen4
			    
;******* Letters are aligned from left to right *******
letter_1	res 1
letter_2	res 1
letter_3	res 1
letter_4	res 1
letter_5	res 1	
letter_6	res 1	
currentPushedButton	    res 1	; will store current button
					; 0x0 -> no button is pushed
					; 0x2 -> button2 is pushed
					; 0x3 -> button3 is pushed
					; 0x4 -> button4 is pushed
					; ....
					; 0x9 -> button9 is pushed				
currentModulo		    res 1	; will store modulo for cycles 0, 1, 2, 0, ...
lastWrittenLetterPosition   res 1	; 0x0 -> no character is written yet
					; 0x1 -> letter_1 is the last character written
					; ...
					; 0x6 -> letter_6 is the last character written
	
; 0x0 -> empty
; 0x1 -> a
; 0x2 -> b
; ...
; 0x18 = 24 -> space
; every character is equal to char = 3 * currentButton + currentModulo
;*********************************************

attentionRequired   res 1   ; bit0 : 0 -> not detected	, 1 -> rb4 press or release action detected
			    ; bit1 : 0 -> not set	, 1 -> debouincing timer is set for rb4
			    ; bit2 : 0 -> not set	, 1 -> debouincing timer is set for keypad
			    ; bit3 : 0 -> not detected	, 1 -> debouncing timer is up (10ms is up)
			    
shouldUpdateScreen  res 1   ; At every 5ms bit0 will be set to 1 to signal main that display must be updated.
			    ; That is, next 7-seg must bu drawn. 1, 2, 3, 4, 1, 2, ...
			    

rbPressedCoordinate res 1   ; will be used to poll the specific button
rdPressedCoordinate res 1   ; will be used to poll the specific button
			    
isAnyKeyPressed	res 1	    ; bit0 -> definetlyReleased, bit1 -> mightBePressed, bit2 -> definetlyPressed, bit3 -> mightBeReleased
			    ; in definetlyReleased state, all keys are polled
			    ; in other states, specific key is polled
			    
			    
commitCounter	res 1	    ; Decreased by Timer3 from 100 at every 10ms.
			    ; Therefore, when it is 0, it means 1 second elapsed and
			    ; current letter must be committed.

commitCurrentLetter res 1   ; Used to signal main to commit current letter after 1 second
			    ; bit0 = 1 -> commit current letter
			    
tempTRISD   res 1   ; to save and restore display
tempLATD    res 1   ; while polling for keypad
tempLATA    res 1   
    
    
pollCounter res 1   ;
 
reviewStateLeftMostLetter   res 1   ; 0x1, 0x2, 0x3, ..., 0x6
   
readStateDirection	res 1
readStateLeftMostLetter	res 1
			    
org     0x00
goto	initialState

org     0x08
bra    isr

isr:
    btfsc   INTCON, 0	    ; Is it because of RB4, check RBIF
	bra isr_rb4
    btfsc   PIR1, 0	    ; Is it because of Timer1 (10ms debouncing of RB4), check TMR1IF
	bra isr_timer1
    btfsc   INTCON, 2	    ; Is it because of Timer0 (1 second passed), check TMR0IF
	bra isr_timer0
    btfsc   PIR1, 1	    ; Is it because of Timer2 (updating screen), check TMR1IF
	bra isr_timer2
    btfsc   PIR2, 1	    ; Is it because of Timer3 (Commit and Scroll counter)
	bra isr_timer3
	
isr_timer3:
    movf    PIR2	; must be read to have last-read value
    bcf	    PIR2, 1	; clear TMR3IF
    
    decfsz  commitCounter   ; Decrease, then is 0 ?
	retfie FAST	    ; No
    
    bsf	    commitCurrentLetter, 0  ; Yes, then commit current letter
    retfie FAST
    
	
isr_timer2:
    movf    PIR1	; must be read to have last-read value
    bcf	    PIR1, 1	; clear TMR2IF
    
    bsf	    shouldUpdateScreen, 0   ; Signal main that next screen must be drawn
    retfie  FAST
	

isr_timer0:    
    movf    INTCON	; Must be read to have last-read value
    bcf	    INTCON, 2	; so that TMR0IF can be cleared
    
    movlw   0x67
    movwf   TMR0H
    movlw   0x69
    movwf   TMR0L	; Initial value of 0x6769 for 1
    
    movlw   0x00
    cpfsgt  countDownCounterOnes    ; Is there a number greater than 0 in Ones Place
	bra borrow		    ; No, then borrow from Tens Place
    
    decf    countDownCounterOnes    ; Decrease by 1 second
    
    cpfseq  countDownCounterOnes    ; Is there any more seconds in Ones Place
	retfie	FAST		    ; Yes, then return from interrupt (We have time)
    
    cpfseq  countDownCounterTens    ; No, then check if there are any in Tens Place
	retfie	FAST		    ; Yes, then return from interrupt (We have time)
    
    ; Time Left = 0x00 -> Time is up
    ;bcf	    INTCON, 7	    ; No more interrupts, (GIE cleared)
    
    bcf	    INTCON, 3	; Disable RB interrupt
    bcf	    INTCON, 5	; Disable TMR0 interrupt
    bcf	    PIE1, 0	; Disable TMR1 Interrupt
    

    
    
    clrf    outerState	
    bsf	    outerState, 3   ; Go into readState
    
    retfie FAST
    
    borrow:
	decf	countDownCounterTens	; Decrease by one Tens Place
	movlw	0x09
	movwf	countDownCounterOnes	; Set Ones Place to 9	
	retfie	FAST
	

isr_timer1:
    
    movf    PIR1	; must be read to have last-read value
    bcf	    PIR1, 0	; clear TMR1F
    bcf	    T1CON, 0	; stop TMR1 (TMR1ON = 0)
    bcf	    PIE1, 0	; disable TMR1 Interrupt
    
    bsf	    attentionRequired, 3	; debouncing Timer is up, main must deal with it
    
    retfie FAST
    
    
	    
isr_rb4:
    movf    PORTB	    ; must be read to have last-read value
    bcf	    INTCON, 0	    ; clear RB4 Interrupt flag
    
    bsf	    attentionRequired, 0	; An action is detected, main must deal with it
    retfie  FAST
    
    
    
    
    
 

wait_for_n_times_10ms:  ;n * 10ms (9.98 ms in fact)
    while_count_n:
	movlw 0x82
	movwf count_256_1
	while_count_256_1:
	    setf count_256_2
	    while_count_256_2:
		decf count_256_2
		bnz while_count_256_2
	    decf count_256_1
	    bnz while_count_256_1
	decf count_n
	bnz while_count_n
    return   


set_letters:
    movlw   0x2
    mullw   0x3
    movf    PRODL, 0
    addlw   0x2
    movwf   letter_1
    
    movlw   0x3
    mullw   0x3
    movf    PRODL, 0
    addlw   0x1
    movwf   letter_2
    
    movlw   0x6
    mullw   0x3
    movf    PRODL, 0
    addlw   0x1
    movwf   letter_3    
    
    movlw   0x4
    mullw   0x3
    movf    PRODL, 0
    addlw   0x0
    movwf   letter_4
    
    movlw   0x8
    mullw   0x3
    movf    PRODL, 0
    addlw   0x0
    movwf   letter_5
    
    movlw   0x8
    mullw   0x3
    movf    PRODL, 0
    addlw   0x0
    movwf   letter_6
    
    clrf    outerState
    bsf	    outerState, 2
    bsf	    reviewStateLeftMostLetter, 0
    return
    
main:
    btfsc   attentionRequired, 0    ; Did rb4 changed state ?
	call rb4_action_detected    ; Yes, 
    btfsc   attentionRequired, 3    ; Did timer is up for debouincing
	call whose_timer_is_up	    ; Yes, let's see if it is RB4 or KEYPAD
    dcfsnz  pollCounter
	call poll_keypad
	
	
    btfsc   outerState, 1   ; Is in writeState ?
	goto writeState	    ; Yes, go to writeState
    btfsc   outerState, 2   ; Is in reviewState ?
	goto reviewState	    ; Yes, go to reviewState
    btfsc   outerState, 3   ; Is in readState ?
	goto readState	    ; Yes, go to readState
    
set_timer1_debouncing:
    ;***** TIMER1 SETUP (for debouncing of RB4 and KEYPAD, 10ms is set in ISR)******
	clrf	    T1CON
	bsf	    T1CON, 4		; 1:2 Prescale set
	bsf	    INTCON, 6		; Peripheral Interrupts Enabled
	
;	movlw	0x3C
;	movwf	TMR1H
;	movlw	0xB0
;	movwf	TMR1L		; 0x3CB0 = 15536 is the initial value of Timer1
;	bsf	T1CON, 0		; TMR1ON is set
;	bsf	PIE1, 0			; TMR1IE is set
	;************************
	
    movlw	0x3C
    movwf	TMR1H
    movlw	0xB0
    movwf	TMR1L		; 0x3CB0 = 15536 is the initial value of Timer1
    bsf	T1CON, 0	; TMR1ON is set
    bsf	PIE1, 0		; TMR1IE is set
    return
    
initialState:
    
    ;***** RB3 (initialState -> writeState button) Setup ******
    clrf    LATB
    clrf    TRISB		;
    bsf	    TRISB, 3		; Configure RB3 as input
    ;************************   
    
    ;***** Start Main Loop ******
    bra from_initial_to_write;main
    ;************************
    rb3_Released:
	btfsc	PORTB, 3	    ; Is Pressed ? (LOW ?)
	    bra rb3_Released	    ; No, then wait
	
	movlw	0x1		    ; Yes, RB3 is pressed
	movwf	count_n	    ; 
	call wait_for_n_times_10ms  ; will wait for (count_256 * 10ms)
	
	btfsc	PORTB, 3	    ; Is still Pressed ?
	    bra rb3_Released	    ; No, then go back waiting
	    
	bra	rb3_Pressed	    ; Yes, go to Pressed    
	
    rb3_Pressed:
	btfss	PORTB, 3	    ; Is Released ? (LOW ?)
	    bra rb3_Pressed	    ; No, then wait
	
	movlw	0x1		    ; Yes, RB3 is pressed
	movwf	count_n	    
	call wait_for_n_times_10ms  ; will wait for (count_256 * 10ms)
	
	btfss	PORTB, 3	    ; Is still Released?
	    bra rb3_Pressed	    ; No, then wait     
    from_initial_to_write:
	bcf	outerState, 0	    ; Yes,
	bsf	outerState, 1	    ; then go into writeState
	
	
	;***** RB4 (writeState-reviewState Change Interrupt) Setup******
	clrf	    rb4ButtonState
	bsf	    rb4ButtonState, 0	; Start from definetlyReleased state
	bsf	    TRISB, 4		; Configure RB4 as input    
	bcf	    INTCON2, 7		; PORTB pull-ups are enabled
	bsf	    INTCON, 3		; Enable RBs InterruptOnChange so that RB4 interrupts

	;************************
	
	;***** TIMER0 SETUP for 1 second (will be used for 20 seconds countdown) ************
	movlw   b'00000111'	; 1:256 scale is set
	movwf   T0CON

	movlw   0x67
	movwf   TMR0H
	movlw   0x69
	movwf   TMR0L		; Initial value of 0x6769

	bsf	    T0CON, 7	; TMR0 is started
	bsf	    INTCON, 5	; TMR0 interrupt enabled
	movf	    INTCON	; Must have last-read value
	bcf	    INTCON, 2	; so that TMR0IF can be cleared
	;******************************************	
	
	;***** 20 seconds Count Down register are set******        
	movlw   0x02
	movwf   countDownCounterTens
	movlw   0x00
	movwf   countDownCounterOnes	
	;************************

	;***** TIMER1 SETUP (for debouncing of RB4 and KEYPAD, 10ms is set in ISR)******
	clrf	    T1CON
	;bsf	    T1CON, 4		; 1:2 Prescale set
	bsf	    INTCON, 6		; Peripheral Interrupts Enabled
	
;	movlw	0x3C
;	movwf	TMR1H
;	movlw	0xB0
;	movwf	TMR1L		; 0x3CB0 = 15536 is the initial value of Timer1
;	bsf	T1CON, 0		; TMR1ON is set
;	bsf	PIE1, 0			; TMR1IE is set
	;************************
	
	;******* RB4 and KEYPAD Debouincing Register SETUP*********
	clrf	attentionRequired
	;******************

	;***** TIMER2 Setup (for Screen Updating, at every --ms one of 7-seg will be set) ******
	movlw   b'011110101'		; 1:16 Prescale and 1:16 Postscale
	movwf   T2CON
	
	movlw	    0xC3
	movwf	    PR2

	bsf	    T2CON, 2	; TMR2 is started
	bsf	    PIE1, 1	; TMR2 interrupt enabled
	movf	    PIR1	; Must have last-read value
	bcf	    PIR1, 1	; so that TMR2IF can be cleared
	;******************************************	
	
	;****** TIMER3 SETUP for letter commit time (1s) **********
	bsf	T3CON, 4	; 1:2 Prescale is set
	
	movlw	0x3C
	movwf	TMR3H
	movlw	0xB0
	movwf	TMR3L		; Initial value for 10ms
	
	
	movf	PIR2		; Must have last-read value
	bcf	PIR2, 1		; so that TMR3IF can be cleared
	
	;bsf	PIE2, 1		; TMR3 interrupt enabled
	;bsf	T3CON, 0	; TMR3 is started
	;*****************************

	;***** 7-Segment setup ******
	clrf    LATD		
	clrf    TRISD		; PORTD all bits are set as output
	setf    ADCON1		;   Configure PORTA as digital
	clrf    LATA
	movlw   b'11000011'		; PORTA<2,3,4,5> bits are set as output
	movwf   TRISA

	clrf    whichScreen	
	bsf	whichScreen, 0	; Next screen to be drawn is Screen1
	bsf	shouldUpdateScreen, 0
	;************************	
	
	;****** KEYPAD SETUP *********
	bcf	TRISB, 0
	bsf	LATB, 0		; RB0 set as output HIGH
	
	bcf	TRISB, 1	
	bsf	LATB, 1		; RB1 set as output HIGH
	
	bcf	TRISB, 2
	bsf	LATB, 2		; RB2 set as output HIGH
	
	;bsf	TRISD, 0	; RD0 set as INPUT
	;bsf	TRISD, 1	; RD1 set as INPUT
	;bsf	TRISD, 2	; RD2 set as INPUT
	;bsf	TRISD, 3	; RD3 set as INPUT
	
	clrf	rbPressedCoordinate	; coordinates are not important when not pressed
	clrf	rdPressedCoordinate	; coordinates are not important when not pressed
	
	clrf	isAnyKeyPressed		; currently no key is pressed
	bsf	isAnyKeyPressed, 0	; wait in definitelyReleased state
	
	movlw	0x1
	movwf	pollCounter
	;****************************
	
	;******* Current Button Registers SETUP *********
	clrf	currentModulo
	clrf	currentPushedButton
	;************************************************
	
	;***** General Interrupt Setup ******
	bcf	    RCON, 7	    ; Disable Priorities for Interrupts
	movf    PORTB	    ; Must have last-read value of RB
	bcf	    INTCON, 0	    ; so that RBIF can be cleared
	bsf	    INTCON, 7	    ; Enable GIE
	;************************	
	
	call set_letters
	bra	main	
	
        
writeState:
    btfsc   shouldUpdateScreen, 0   ; Is 5ms up ?
	call update_screen_in_writeState	    ; Yes, then draw next 7-seg       
    
    btfsc   commitCurrentLetter, 0  ; Should current letter be committed ?
	call commit_current_letter  ; Yes, commit
    goto main
    
    commit_current_letter:
	clrf	commitCurrentLetter
	
	movf    currentPushedButton, 0
	mullw   0x03
	movf    PRODL, 0
	addwf   currentModulo, 0
	
	
	dcfsnz	lastWrittenLetterPosition   ; Is last written letter_1 ?
	    bra	commit_letter_2		    ; Then, current must be 2
	dcfsnz	lastWrittenLetterPosition
	    bra	commit_letter_3
	dcfsnz	lastWrittenLetterPosition
	    bra	commit_letter_4
	dcfsnz	lastWrittenLetterPosition
	    bra	commit_letter_5
	dcfsnz	lastWrittenLetterPosition
	    bra	commit_letter_6
	bra commit_letter_1		    ; Then, current must be first
	
	commit_letter_1:	    
	    movwf   letter_1
	    movlw   0x1
	    bra	    common
	commit_letter_2:
	    movwf   letter_2
	    movlw   0x2
	    bra	    common    
	commit_letter_3:
	    movwf   letter_3
	    movlw   0x3
	    bra	    common	    
	commit_letter_4:
	    movwf   letter_4
	    movlw   0x4
	    bra	    common  
	commit_letter_5:
	    movwf   letter_5
	    movlw   0x5
	    bra	    common  
	commit_letter_6:
	    movwf   letter_6
	    movlw   0x6
	    
	    ;bcf	    INTCON, 7	    ; No more interrupts, (GIE cleared)
	    
	    bcf	    INTCON, 3	; Disable RB interrupt
	    bcf	    INTCON, 5	; Disable TMR0 interrupt
	    bcf	    PIE1, 0	; Disable TMR1 Interrupt
    
    
	    clrf    outerState	
	    bsf	    outerState, 3   ; Go into readState
	    
	    bra	    common  
	common:
	    movwf   lastWrittenLetterPosition
	    clrf    currentPushedButton
	    clrf    currentModulo
	    return
    
	
    save_current_display:
	movff	LATA, tempLATA	    ; save current selection of display
	movff	LATD, tempLATD
	movff	TRISD, tempTRISD    ; save current content of display
	
	clrf	LATA		    ; do not select any
	movlw	b'00001111'	    ; prepare for polling rd<0-3> configured as input
	movwf	TRISD
	clrf	LATD
	return
    restore_current_display:
	movff	tempTRISD, TRISD
	movff	tempLATD, LATD
	movff	tempLATA, LATA
	return	 
    
    
    rb4_action_detected:
	bcf	attentionRequired, 0	; Attention is given, clear flag
	
	btfsc   rb4ButtonState, 0	    ; Is in definetlyReleased ?
	    bra rb4_definetlyReleased   ; Yes, 
	btfsc   rb4ButtonState, 2	    ; Is in definetlyPressed ?
	    bra rb4_definetlyPressed    ; Yes,

	rb4_definetlyReleased:
	    btfsc	PORTB, 4	; Is RB4 Pressed ?
		return	; No, then nothing happens

	    bcf rb4ButtonState, 0	; Yes, then change button state
	    bsf rb4ButtonState, 1	; into mightBePressed
	    bra rb4_action_common
	    
	rb4_definetlyPressed:
	    btfss	PORTB, 4	; Is RB4 Released?
		return	; No, then nothing happens

	    bcf rb4ButtonState, 2	; Yes, then change button state
	    bsf rb4ButtonState, 3	; into mightBeReleased
	    bra rb4_action_common
	    
	rb4_action_common:
	    bsf		attentionRequired, 1	; Timer is set for RB4)
						; this bit says that timer is for
						; debouncing effect of rb4
	    
	    call set_timer1_debouncing
	    
	    return
	    
    whose_timer_is_up:    
	bcf	attentionRequired, 3	; debouincing flag is cleared
	
	btfsc	attentionRequired, 1	; RB4 debouncing timer is up
	    call rb4_timer_detected
	btfsc	attentionRequired, 2	; KEYPAD debouincing timer is up
	    call is_it_in_write_or_review
	return
	
	is_it_in_write_or_review:
	    btfsc   outerState, 1   ; Is it in writeState ?
		call keypad_timer_in_writeState
	    btfsc   outerState, 2   ; Is it in reviewState ?
		call keypad_timer_in_reviewState	
	    return
	
	keypad_timer_in_reviewState:
	    call save_current_display
	    
	    bcf	attentionRequired, 2	; Attention is given, clear flag	    

	    btfsc   isAnyKeyPressed, 1	    ; Is in mightBePressed
		bra keypad_mightBePressed_in_reviewState	    ; Yes, 
	    btfsc   isAnyKeyPressed, 3	    ; Is in mightBeReleased
		bra mightBeReleased_in_reviewState	    ; Yes,

	    keypad_mightBePressed_in_reviewState:
		bsf	LATB, 0
		bsf	LATB, 1
		bsf	LATB, 2	
		
		movf	rbPressedCoordinate, 0
		incf	WREG
		
		dcfsnz	WREG			    ; Is it LEFT (COL0)
		    bra	check_left_pressed 
		decf	WREG
		dcfsnz	WREG			    ; Is it RIGHT (COL2)
		    bra check_right_pressed
		    
		goto    back_definetlyReleased_review
		
		
		
		check_right_pressed:
		    bcf	LATB, 2
		    btfsc	    PORTD, 0
			goto    back_definetlyReleased_review

		    bcf	isAnyKeyPressed, 1
		    bsf	isAnyKeyPressed, 2
		    goto restore_current_display		    
		
		check_left_pressed:
		    bcf	LATB, 0
		    
		    btfsc	    PORTD, 0
			goto    back_definetlyReleased_review

		    bcf	isAnyKeyPressed, 1
		    bsf	isAnyKeyPressed, 2
		    goto restore_current_display
		back_definetlyReleased_review:
			bcf	isAnyKeyPressed, 1
			bsf	isAnyKeyPressed, 0
			goto restore_current_display		
	    mightBeReleased_in_reviewState:
		bsf	LATB, 0
		bsf	LATB, 1
		bsf	LATB, 2
		
		movf	rbPressedCoordinate, 0
		incf	WREG
		
		dcfsnz	WREG			    ; Is it LEFT (COL0)
		    bra	check_left_released 	
		decf	WREG
		dcfsnz	WREG			    ; Is it RIGHT (COL2)
		    bra check_right_released		
		
		goto    back_definetlyPressed_review
		check_right_released:
		    bcf	LATB, 2
		    btfss	    PORTD, 0
			goto    back_definetlyPressed_review

		    movlw	0x3
		    cpfseq	reviewStateLeftMostLetter
			incf    reviewStateLeftMostLetter

		    goto mightBeReleased_com_reviewState					
		check_left_released:
		    bcf	LATB, 0
		    btfss	    PORTD, 0
			goto    back_definetlyPressed_review

		    movlw	0x1
		    cpfseq	reviewStateLeftMostLetter
			decf    reviewStateLeftMostLetter

		    goto mightBeReleased_com_reviewState	
		back_definetlyPressed_review:
		    bcf	isAnyKeyPressed, 3
		    bsf	isAnyKeyPressed, 2
		    goto restore_current_display
		mightBeReleased_com_reviewState:
		    bcf	isAnyKeyPressed, 3
		    bsf	isAnyKeyPressed, 0
		    
		    goto restore_current_display
			
	
	poll_keypad:
	    movlw   0x4
	    movwf   pollCounter
	    
	    btfsc   isAnyKeyPressed, 0	    ; Is in definetlyReleased
		call poll_all_keys	    ; Yes, poll all keys
	    btfsc   isAnyKeyPressed, 2	    ; Is in definetlyPressed
		call poll_specific_key	    ; Yes, poll the specific key
	    return
	    poll_all_keys:    
		call save_current_display
		
		btfss   outerState, 1			; Is in reviewState
			bra poll_col0_in_reviewState	; Yes, 		
			
		;********* POLL COL0 *******
		poll_col0_in_writeState:		; No,
		    bcf	LATB, 0
		    
		    btfss	PORTD, 1
			bra coor_0_2
		    btfss	PORTD, 2
			bra coor_0_1
		    ;btfss	PORTD, 3
		    ;	bra coor_0_0

		    bsf	LATB, 0
		    bra poll_col1_in_writeState
		poll_col0_in_reviewState:
		    bcf	    LATB, 0
		    
		    btfss   PORTD, 0
		    	bra coor_0_3
		    
		    bsf	LATB, 0
		    bra poll_col2_in_reviewState

		    coor_0_0:
			; shouldn't be here
			movlw   0x0
			movwf   rbPressedCoordinate
			movwf   rdPressedCoordinate
			bra poll_all_keys_common	    
		    coor_0_1:
			movlw   0x0
			movwf   rbPressedCoordinate
			movlw   0x1
			movwf   rdPressedCoordinate
			bra poll_all_keys_common	    
		    coor_0_2:
			movlw   0x0
			movwf   rbPressedCoordinate
			movlw   0x2
			movwf   rdPressedCoordinate
			bra poll_all_keys_common	    
		    coor_0_3:
			; shouldn't be here
			movlw   0x0
			movwf   rbPressedCoordinate
			movlw   0x3
			movwf   rdPressedCoordinate
			bra poll_all_keys_common       
		;***************************
		;********* POLL COL1 *******
		poll_col1_in_writeState:
		    bcf	LATB, 1
		    ;btfss	PORTD, 0
		    ;	bra coor_1_3
		    btfss	PORTD, 1
			bra coor_1_2
		    btfss	PORTD, 2
			bra coor_1_1
		    btfss	PORTD, 3
		    	bra coor_1_0
		    
		    bsf	LATB, 1
		    bra poll_col2_in_writeState

		    coor_1_0:
			movlw   0x1
			movwf   rbPressedCoordinate
			movlw   0x0
			movwf   rdPressedCoordinate
			bra poll_all_keys_common	    
		    coor_1_1:
			movlw   0x1
			movwf   rbPressedCoordinate
			movlw   0x1
			movwf   rdPressedCoordinate
			bra poll_all_keys_common	    
		    coor_1_2:
			movlw   0x1
			movwf   rbPressedCoordinate
			movlw   0x2
			movwf   rdPressedCoordinate
			bra poll_all_keys_common	    
		    coor_1_3:
			; shouldn't be here
			movlw   0x1
			movwf   rbPressedCoordinate
			movlw   0x3
			movwf   rdPressedCoordinate
			bra poll_all_keys_common       
		;***************************
		;********* POLL COL2 *******
		poll_col2_in_writeState:
		    bcf	LATB, 2		    
		    btfss	PORTD, 1
			bra coor_2_2
		    btfss	PORTD, 2
			bra coor_2_1
		    btfss	PORTD, 3
			bra coor_2_0
		    
		    bsf	LATB, 2
		    goto restore_current_display
		    
		poll_col2_in_reviewState:
		    bcf	LATB, 2
		    btfss	PORTD, 0
		    	bra coor_2_3
		    
		    bsf	LATB, 2
		    goto restore_current_display

		    coor_2_0:
			movlw   0x2
			movwf   rbPressedCoordinate
			movlw   0x0
			movwf   rdPressedCoordinate
			bra poll_all_keys_common	    
		    coor_2_1:
			movlw   0x2
			movwf   rbPressedCoordinate
			movlw   0x1
			movwf   rdPressedCoordinate
			bra poll_all_keys_common	    
		    coor_2_2:
			movlw   0x2
			movwf   rbPressedCoordinate
			movlw   0x2
			movwf   rdPressedCoordinate
			bra poll_all_keys_common	    
		    coor_2_3:
			; shouldn't be here
			movlw   0x2
			movwf   rbPressedCoordinate
			movlw   0x3
			movwf   rdPressedCoordinate
			bra poll_all_keys_common       
		;***************************	
		poll_all_keys_common:    
		    bcf	    isAnyKeyPressed, 0	; Change state into
		    bsf	    isAnyKeyPressed, 1	; mightBePressed

		    bsf		attentionRequired, 2	; Timer is set for KEYPAD
		    bra poll_all_keys_set_timer1_again
		    
		poll_all_keys_set_timer1_again:
    
		    call set_timer1_debouncing
		    
		    goto restore_current_display
		    
	    poll_specific_key:
		call save_current_display

		bsf	LATB, 0
		bsf	LATB, 1
		bsf	LATB, 2
		
		btfsc	outerState, 2			    ; Is it in reviewState
		    bra poll_specific_key_in_reviewState    ; Yes, 
		    
						    ; No,
		movf	rbPressedCoordinate, 0
		incf	WREG
		
		dcfsnz	WREG	; Is it on COL0
		    bra check_col0_pressed_def 
		dcfsnz	WREG	; Is it on COL1
		    bra check_col1_pressed_def
		dcfsnz	WREG	; Is it on COL2
		    bra check_col2_pressed_def
		
		goto restore_current_display
		
		poll_specific_key_in_reviewState:
		    btfsc   rbPressedCoordinate, 1	; Is it RIGHT button
			bra col2_row3_pressed_def	; Yes
		    bra col0_row3_pressed_def		; No
		    
		    
		check_col2_pressed_def:
		    bcf	LATB, 2
		    
		    movf    rdPressedCoordinate, 0
		    incf    WREG
			
		    dcfsnz  WREG		    ; Is it ROW0
			bra col2_row0_pressed_def	     
		    dcfsnz  WREG		    ; Is it ROW1
			bra col2_row1_pressed_def    
		    dcfsnz  WREG		    ; Is it ROW2
			bra col2_row2_pressed_def
		    
		    goto restore_current_display

		    col2_row0_pressed_def:
			movf	    PORTD, 0
			btfss	    PORTD, 3
			;btfss	    PORTD, 3
			    goto restore_current_display
			nop
			goto poll_specific_key_common
		    col2_row1_pressed_def:
			btfss	    PORTD, 2
			    goto restore_current_display
			goto poll_specific_key_common
		    col2_row2_pressed_def:
			btfss	    PORTD, 1
			    goto restore_current_display
			goto poll_specific_key_common
		    col2_row3_pressed_def:		
							    ; Yes, It is RIGHT button
			btfss	    PORTD, 0		    
			    goto restore_current_display
			goto poll_specific_key_common		    
		check_col1_pressed_def:
		    bcf	LATB, 1
		    
		    movf    rdPressedCoordinate, 0
		    incf    WREG
		    
		    dcfsnz  WREG		    ; Is it ROW0
			bra col1_row0_pressed_def   
		    dcfsnz  WREG		    ; Is it ROW1
			bra col1_row1_pressed_def    
		    dcfsnz  WREG		    ; Is it ROW2
			bra col1_row2_pressed_def
		    
		    goto restore_current_display
			

		    col1_row0_pressed_def:
			btfss	    PORTD, 3
			    goto restore_current_display
			goto poll_specific_key_common
		    col1_row1_pressed_def:
			btfss	    PORTD, 2
			    goto restore_current_display
			goto poll_specific_key_common
		    col1_row2_pressed_def:
			btfss	    PORTD, 1
			    goto restore_current_display
			goto poll_specific_key_common
		    col1_row3_pressed_def:
			; shouldn't be here
			goto restore_current_display
		check_col0_pressed_def:
		    bcf	LATB, 0
		    
		    movf    rdPressedCoordinate, 0
		    incf    WREG
		    
		    dcfsnz  WREG		    ; Is it ROW0
			bra col0_row0_pressed_def   
		    dcfsnz  WREG		    ; Is it ROW1
			bra col0_row1_pressed_def
		    dcfsnz  WREG		    ; Is it ROW2
			bra col0_row2_pressed_def
		    dcfsnz  WREG		    ; Is it ROW3
			bra col0_row3_pressed_def
			
		    goto restore_current_display

		    col0_row0_pressed_def:
			; shouldn't be here
			goto restore_current_display
		    col0_row1_pressed_def:
			btfss	    PORTD, 2
			    goto restore_current_display
			goto poll_specific_key_common
		    col0_row2_pressed_def:
			btfss	    PORTD, 1
			    goto restore_current_display
			goto poll_specific_key_common
		    col0_row3_pressed_def:
							    ; Yes, It is LEFT button
			btfss	    PORTD, 0
			    goto restore_current_display
			goto poll_specific_key_common
		poll_specific_key_common:    
		    bcf	isAnyKeyPressed, 2	; Change state into
		    bsf	isAnyKeyPressed, 3	; mightBeReleased	   

		    bsf		attentionRequired, 2	; Timer is set for KEYPAD
		    
		    call set_timer1_debouncing

		    goto restore_current_display
	keypad_timer_in_writeState:
	    call save_current_display
	    
	    bcf	attentionRequired, 2	; Attention is given, clear flag	    

	    btfsc   isAnyKeyPressed, 1	    ; Is in mightBePressed
		bra keypad_mightBePressed	    ; Yes, 
	    btfsc   isAnyKeyPressed, 3	    ; Is in mightBeReleased
		bra keypad_mightBeReleased	    ; Yes,

	    keypad_mightBePressed:
		bsf	LATB, 0
		bsf	LATB, 1
		bsf	LATB, 2
		
		movf	rbPressedCoordinate, 0
		incf	WREG
		
		dcfsnz	WREG	    ; is it COL0
		    bra check_col0_pressed
		dcfsnz	WREG	    ; is it COL1
		    bra check_col1_pressed
		dcfsnz	WREG	    ; is it COL2
		    bra check_col2_pressed
		
		goto goBack_definetlyReleased_keypad
		
		check_col2_pressed:
		    bcf	LATB, 2
		    
		    movf    rdPressedCoordinate, 0
		    incf    WREG
		    
		    dcfsnz  WREG		    ; Is it ROW0
			bra col2_row0_pressed	  
		    dcfsnz  WREG		    ; Is it ROW1
			bra col2_row1_pressed    
		    dcfsnz  WREG		    ; Is it ROW2
			bra col2_row2_pressed
		    dcfsnz  WREG		    ; Is it ROW3
			bra col2_row3_pressed
		    
		    goto    goBack_definetlyReleased_keypad
		    col2_row0_pressed:
			btfsc	    PORTD, 3
			    goto    goBack_definetlyReleased_keypad
			
			bcf	isAnyKeyPressed, 1
			bsf	isAnyKeyPressed, 2
			goto restore_current_display
		    col2_row1_pressed:
			btfsc	    PORTD, 2
			    goto    goBack_definetlyReleased_keypad
			
			bcf	isAnyKeyPressed, 1
			bsf	isAnyKeyPressed, 2
			goto restore_current_display
		    col2_row2_pressed:
			btfsc	    PORTD, 1
			    goto    goBack_definetlyReleased_keypad
			
			bcf	isAnyKeyPressed, 1
			bsf	isAnyKeyPressed, 2
			goto restore_current_display
		    col2_row3_pressed:
			; shouldn't be here
			goto goBack_definetlyReleased_keypad	    
		check_col1_pressed:
		    bcf	LATB, 1
		    
		    movf    rdPressedCoordinate, 0
		    incf    WREG
		    
		    dcfsnz  WREG		    ; Is it ROW0
			bra col1_row0_pressed	    
		    dcfsnz  WREG		    ; Is it ROW1
			bra col1_row1_pressed    
		    dcfsnz  WREG		    ; Is it ROW2
			bra col1_row2_pressed
		    dcfsnz  WREG		    ; Is it ROW3
			bra col1_row3_pressed
		    
		    goto    goBack_definetlyReleased_keypad		    
		    
		    col1_row0_pressed:
			btfsc	    PORTD, 3
			    goto    goBack_definetlyReleased_keypad
			
			bcf	isAnyKeyPressed, 1
			bsf	isAnyKeyPressed, 2
			goto restore_current_display
		    col1_row1_pressed:
			btfsc	    PORTD, 2
			    goto    goBack_definetlyReleased_keypad
			
			bcf	isAnyKeyPressed, 1
			bsf	isAnyKeyPressed, 2
			goto restore_current_display
		    col1_row2_pressed:
			btfsc	    PORTD, 1
			    goto    goBack_definetlyReleased_keypad
			
			bcf	isAnyKeyPressed, 1
			bsf	isAnyKeyPressed, 2
			goto restore_current_display
		    col1_row3_pressed:
			; shouldn't be here
			goto goBack_definetlyReleased_keypad
		check_col0_pressed:
		    bcf	LATB, 0
		    
		    movf    rdPressedCoordinate, 0
		    incf    WREG
			
		    dcfsnz  WREG		    ; Is it ROW0
			bra col0_row0_pressed	
		    dcfsnz  WREG		    ; Is it ROW1
			bra col0_row1_pressed    
		    dcfsnz  WREG		    ; Is it ROW2
			bra col0_row2_pressed
		    dcfsnz  WREG		    ; Is it ROW3
			bra col0_row3_pressed
		    
		    goto    goBack_definetlyReleased_keypad
		    col0_row0_pressed:
			; shouldn't be here
			goto goBack_definetlyReleased_keypad
		    col0_row1_pressed:
			btfsc	    PORTD, 2
			    goto    goBack_definetlyReleased_keypad
			
			bcf	isAnyKeyPressed, 1
			bsf	isAnyKeyPressed, 2
			goto restore_current_display
		    col0_row2_pressed:
			btfsc	    PORTD, 1
			    goto    goBack_definetlyReleased_keypad
			
			bcf	isAnyKeyPressed, 1
			bsf	isAnyKeyPressed, 2
			goto restore_current_display
		    col0_row3_pressed:
			; shouldn't be here
			goto goBack_definetlyReleased_keypad
		goBack_definetlyReleased_keypad:
			bcf	isAnyKeyPressed, 1
			bsf	isAnyKeyPressed, 0
			goto restore_current_display		
	    keypad_mightBeReleased:
		bsf	LATB, 0
		bsf	LATB, 1
		bsf	LATB, 2
		
		movf	rbPressedCoordinate, 0
		incf	WREG
		
		dcfsnz	WREG		    ; Is it on COL0
		    bra	check_col0_released   	    ; then, must be on COL0 
		dcfsnz	WREG		    ; Is it on COL1
		    bra check_col1_released
		dcfsnz	WREG		    ; Is it on COL2
		    bra check_col2_released
		    
		goto    goBack_definetlyPressed_keypad	
		
		
		check_col2_released:
		    bcf	LATB, 2
		    
		    
		    movf    rdPressedCoordinate, 0
		    incf    WREG
		    
		    dcfsnz  WREG		    ; Is it ROW0
			bra col2_row0_released
		    dcfsnz  WREG		    ; Is it ROW1
			bra col2_row1_released
		    dcfsnz  WREG		    ; Is it ROW2
			bra col2_row2_released
		    dcfsnz  WREG		    ; Is it ROW3
			bra col2_row3_released
		    
		    goto    goBack_definetlyPressed_keypad
		    
		    col2_row0_released:
			btfss	    PORTD, 3
			    goto    goBack_definetlyPressed_keypad	
			    
			    
			movlw	0x03
			cpfseq	currentPushedButton	; Is it the same button
			    bra	col2_row0_released_diff	; No, it is not
			bra col2_row0_released_same	; Yes, it is
			
			col2_row0_released_same:
			    incf    currentModulo	; Same button then increment
			    
			    movlw   0x03
			    cpfslt  currentModulo	; Is it 3
				clrf	currentModulo	; Yes, then cycle back to 0
			    goto keypad_mightBeReleased_common
			    
			col2_row0_released_diff:
			    movlw   0x00
			    cpfsgt  currentPushedButton		; Is another button was pushed ?
				bra col2_row0_released_diff_new	; No, then current place
			    bra	col2_row0_released_diff_another	; Yes, then first commit
			    
			    col2_row0_released_diff_new:
				movlw	0x03
				movwf	currentPushedButton	; Change pushed button as this button
				clrf	currentModulo		; First press, modulo is 0
				
				goto keypad_mightBeReleased_common		
				
			    col2_row0_released_diff_another:
				call commit_current_letter
				
				movlw	0x03
				movwf	currentPushedButton	; Change pushed button as this button
				clrf	currentModulo		; First press, modulo is 0
				
				goto keypad_mightBeReleased_common
		    col2_row1_released:
			btfss	    PORTD, 0
			    goto    goBack_definetlyPressed_keypad
			    
			movlw	0x06
			cpfseq	currentPushedButton	; Is it the same button
			    bra	col2_row1_released_diff	; No, it is not
			bra col2_row1_released_same	; Yes, it is
			
			col2_row1_released_same:
			    incf    currentModulo	; Same button then increment
			    
			    movlw   0x03
			    cpfslt  currentModulo	; Is it 3
				clrf	currentModulo	; Yes, then cycle back to 0
			    goto keypad_mightBeReleased_common
			    
			col2_row1_released_diff:
			    movlw   0x00
			    cpfsgt  currentPushedButton		; Is another button was pushed ?
				bra col2_row1_released_diff_new	; No, then current place
			    bra	col2_row1_released_diff_another	; Yes, then first commit
			    
			    col2_row1_released_diff_new:
				movlw	0x06
				movwf	currentPushedButton	; Change pushed button as this button
				clrf	currentModulo		; First press, modulo is 0
				
				goto keypad_mightBeReleased_common		
				
			    col2_row1_released_diff_another:
				call commit_current_letter
				
				movlw	0x06
				movwf	currentPushedButton	; Change pushed button as this button
				clrf	currentModulo		; First press, modulo is 0
				
				goto keypad_mightBeReleased_common
		    col2_row2_released:
			btfss	    PORTD, 1
			    goto    goBack_definetlyPressed_keypad
			
			movlw	0x09
			cpfseq	currentPushedButton	; Is it the same button
			    bra	col2_row2_released_diff	; No, it is not
			bra col2_row2_released_same	; Yes, it is
			
			col2_row2_released_same:
			    incf    currentModulo	; Same button then increment
			    
			    movlw   0x03
			    cpfslt  currentModulo	; Is it 3
				clrf	currentModulo	; Yes, then cycle back to 0
			    goto keypad_mightBeReleased_common
			    
			col2_row2_released_diff:
			    movlw   0x00
			    cpfsgt  currentPushedButton		; Is another button was pushed ?
				bra col2_row2_released_diff_new	; No, then current place
			    bra	col2_row2_released_diff_another	; Yes, then first commit
			    
			    col2_row2_released_diff_new:
				movlw	0x09
				movwf	currentPushedButton	; Change pushed button as this button
				clrf	currentModulo		; First press, modulo is 0
				
				goto keypad_mightBeReleased_common		
				
			    col2_row2_released_diff_another:
				call commit_current_letter
				
				movlw	0x09
				movwf	currentPushedButton	; Change pushed button as this button
				clrf	currentModulo		; First press, modulo is 0
				
				goto keypad_mightBeReleased_common
		    col2_row3_released:
			; shouldn't be here	
		check_col1_released:
		    bcf	LATB, 1
		    
		    movf    rdPressedCoordinate, 0
		    incf    WREG
		    
		    dcfsnz  WREG		    ; Is it ROW0
			bra col1_row0_released	    
		    dcfsnz  WREG		    ; Is it ROW1
			bra col1_row1_released  
		    dcfsnz  WREG		    ; Is it ROW2
			bra col1_row2_released
		    dcfsnz  WREG		    ; Is it ROW3
			bra col1_row3_released
			
		    goto    goBack_definetlyPressed_keypad

			
		    col1_row0_released:
			movf	PORTD, 0
			btfss	    PORTD, 3
			    goto    goBack_definetlyPressed_keypad
			    
			movlw	0x02
			cpfseq	currentPushedButton	; Is it the same button
			    bra	col1_row0_released_diff	; No, it is not
			bra col1_row0_released_same	; Yes, it is
			
			col1_row0_released_same:
			    incf    currentModulo	; Same button then increment
			    
			    movlw   0x03
			    cpfslt  currentModulo	; Is it 3
				clrf	currentModulo	; Yes, then cycle back to 0
			    goto keypad_mightBeReleased_common
			    
			col1_row0_released_diff:
			    movlw   0x00
			    cpfsgt  currentPushedButton		; Is another button was pushed ?
				bra col1_row0_released_diff_new	; No, then current place
			    bra	col1_row0_released_diff_another	; Yes, then first commit
			    
			    col1_row0_released_diff_new:
				movlw	0x02
				movwf	currentPushedButton	; Change pushed button as this button
				clrf	currentModulo		; First press, modulo is 0
				
				goto keypad_mightBeReleased_common		
				
			    col1_row0_released_diff_another:
				call commit_current_letter
				
				movlw	0x02
				movwf	currentPushedButton	; Change pushed button as this button
				clrf	currentModulo		; First press, modulo is 0
				
				goto keypad_mightBeReleased_common
		    col1_row1_released:
			btfss	    PORTD, 2
			    goto    goBack_definetlyPressed_keypad
			
			movlw	0x05
			cpfseq	currentPushedButton	; Is it the same button
			    bra	col1_row1_released_diff	; No, it is not
			bra col1_row1_released_same	; Yes, it is
			
			col1_row1_released_same:
			    incf    currentModulo	; Same button then increment
			    
			    movlw   0x03
			    cpfslt  currentModulo	; Is it 3
				clrf	currentModulo	; Yes, then cycle back to 0
			    goto keypad_mightBeReleased_common
			    
			col1_row1_released_diff:
			    movlw   0x00
			    cpfsgt  currentPushedButton		; Is another button was pushed ?
				bra col1_row1_released_diff_new	; No, then current place
			    bra	col1_row1_released_diff_another	; Yes, then first commit
			    
			    col1_row1_released_diff_new:
				movlw	0x05
				movwf	currentPushedButton	; Change pushed button as this button
				clrf	currentModulo		; First press, modulo is 0
				
				goto keypad_mightBeReleased_common		
				
			    col1_row1_released_diff_another:
				call commit_current_letter
				
				movlw	0x05
				movwf	currentPushedButton	; Change pushed button as this button
				clrf	currentModulo		; First press, modulo is 0
				
				goto keypad_mightBeReleased_common
		    col1_row2_released:
			btfss	    PORTD, 1
			    goto    goBack_definetlyPressed_keypad
			
			movlw	0x08
			cpfseq	currentPushedButton	; Is it the same button
			    bra	col1_row2_released_diff	; No, it is not
			bra col1_row2_released_same	; Yes, it is
			
			col1_row2_released_same:
			    incf    currentModulo	; Same button then increment
			    
			    movlw   0x03
			    cpfslt  currentModulo	; Is it 3
				clrf	currentModulo	; Yes, then cycle back to 0
			    goto keypad_mightBeReleased_common
			    
			col1_row2_released_diff:
			    movlw   0x00
			    cpfsgt  currentPushedButton		; Is another button was pushed ?
				bra col1_row2_released_diff_new	; No, then current place
			    bra	col1_row2_released_diff_another	; Yes, then first commit
			    
			    col1_row2_released_diff_new:
				movlw	0x08
				movwf	currentPushedButton	; Change pushed button as this button
				clrf	currentModulo		; First press, modulo is 0
				
				goto keypad_mightBeReleased_common		
				
			    col1_row2_released_diff_another:
				call commit_current_letter
				
				movlw	0x08
				movwf	currentPushedButton	; Change pushed button as this button
				clrf	currentModulo		; First press, modulo is 0
				
				goto keypad_mightBeReleased_common
		    col1_row3_released:
			; shouldn't be here
		check_col0_released:
		    bcf	LATB, 0
		    
		    movf    rdPressedCoordinate, 0
		    incf    WREG
		    
		    dcfsnz  WREG		    ; Is it ROW0
			bra col0_row0_released
		    dcfsnz  WREG		    ; Is it ROW1
			bra col0_row1_released    
		    dcfsnz  WREG		    ; Is it ROW2
			bra col0_row2_released
		    dcfsnz  WREG		    ; Is it ROW3
			bra col0_row3_released
		    
		    goto    goBack_definetlyPressed_keypad
		    
		    col0_row0_released:
			; shouldn't be here
			goto    goBack_definetlyPressed_keypad
		    col0_row1_released:
			btfss	    PORTD, 2
			    goto    goBack_definetlyPressed_keypad
			    
			movlw	0x04
			cpfseq	currentPushedButton	; Is it the same button
			    bra	col0_row1_released_diff	; No, it is not
			bra col0_row1_released_same	; Yes, it is
			
			col0_row1_released_same:
			    incf    currentModulo	; Same button then increment
			    
			    movlw   0x03
			    cpfslt  currentModulo	; Is it 3
				clrf	currentModulo	; Yes, then cycle back to 0
			    goto keypad_mightBeReleased_common
			    
			col0_row1_released_diff:
			    movlw   0x00
			    cpfsgt  currentPushedButton		; Is another button was pushed ?
				bra col0_row1_released_diff_new	; No, then current place
			    bra	col0_row1_released_diff_another	; Yes, then first commit
			    
			    col0_row1_released_diff_new:
				movlw	0x04
				movwf	currentPushedButton	; Change pushed button as this button
				clrf	currentModulo		; First press, modulo is 0
				
				goto keypad_mightBeReleased_common		
				
			    col0_row1_released_diff_another:
				call commit_current_letter
				
				movlw	0x04
				movwf	currentPushedButton	; Change pushed button as this button
				clrf	currentModulo		; First press, modulo is 0
				
				goto keypad_mightBeReleased_common
		    col0_row2_released:
			btfss	    PORTD, 1
			    goto    goBack_definetlyPressed_keypad
			
			movlw	0x07
			cpfseq	currentPushedButton	; Is it the same button
			    bra	col0_row2_released_diff	; No, it is not
			bra col0_row2_released_same	; Yes, it is
			
			col0_row2_released_same:
			    incf    currentModulo	; Same button then increment
			    
			    movlw   0x03
			    cpfslt  currentModulo	; Is it 3
				clrf	currentModulo	; Yes, then cycle back to 0
			    goto keypad_mightBeReleased_common
			    
			col0_row2_released_diff:
			    movlw   0x00
			    cpfsgt  currentPushedButton		; Is another button was pushed ?
				bra col0_row2_released_diff_new	; No, then current place
			    bra	col0_row2_released_diff_another	; Yes, then first commit
			    
			    col0_row2_released_diff_new:
				movlw	0x07
				movwf	currentPushedButton	; Change pushed button as this button
				clrf	currentModulo		; First press, modulo is 0
				
				goto keypad_mightBeReleased_common		
				
			    col0_row2_released_diff_another:
				call commit_current_letter
				
				movlw	0x07
				movwf	currentPushedButton	; Change pushed button as this button
				clrf	currentModulo		; First press, modulo is 0
				
				goto keypad_mightBeReleased_common
		    col0_row3_released:
			; shouldn't be here
			goto    goBack_definetlyPressed_keypad
		goBack_definetlyPressed_keypad:
		    bcf	isAnyKeyPressed, 3
		    bsf	isAnyKeyPressed, 2
		    goto restore_current_display
		keypad_mightBeReleased_common:
		    bcf	isAnyKeyPressed, 3
		    bsf	isAnyKeyPressed, 0    
		    
		    ;****** TIMER3 SETUP for letter commit time (1s) **********
		    bsf	T3CON, 4	; 1:2 Prescale is set
		    
		    movlw	0x4D
		    movwf	commitCounter

		    movlw	0x3C
		    movwf	TMR3H
		    movlw	0xB0
		    movwf	TMR3L		; Initial value for 10ms


		    movf	PIR2		; Must have last-read value
		    bcf	PIR2, 1		; so that TMR3IF can be cleared

		    bsf	PIE2, 1		; TMR3 interrupt enabled
		    bsf	T3CON, 0	; TMR3 is started
		    ;*****************************
		    
		    goto restore_current_display
			
	rb4_timer_detected:
	    bcf	attentionRequired, 1	; Attention is given, clear flag

	    btfsc   rb4ButtonState, 1	    ; Is in mightBePressed
		bra rb4_mightBePressed	    ; Yes, 
	    btfsc   rb4ButtonState, 3	    ; Is in mightBeReleased
		bra rb4_mightBeReleased	    ; Yes,

	    rb4_mightBePressed:
		btfsc	PORTB, 4		    ; Is RB4 still Pressed ?
		    bra goBack_definetlyReleased    ; No, then go back into definetlyReleased

		bcf rb4ButtonState, 1	; Yes, then change button state
		bsf rb4ButtonState, 2	; into definetlyPressed
		return

		goBack_definetlyReleased:
		    bcf rb4ButtonState, 1
		    bsf	rb4ButtonState, 0
		    return

	    rb4_mightBeReleased:
		btfss	PORTB, 4		    ; Is RB4 still Released ?
		    bra goBack_definetlyPressed	    ; No, then go back into definetlyPressed

		bcf rb4ButtonState, 3	; Yes, then change button state
		bsf rb4ButtonState, 0	; into definetlyReleased


		btfsc	outerState, 1	; Is in writeState ?
		    bra to_reviewState	; Yes, then change to reviewState


		to_writeState:
		    bcf	outerState, 2
		    bsf	outerState, 1
		    return
		to_reviewState:
		    bcf	outerState, 1
		    bsf outerState, 2
		    
		    clrf    reviewStateLeftMostLetter	; Default is 0x1
		    bsf	    reviewStateLeftMostLetter, 0
		    
		    clrf    currentPushedButton	    ; Current letter is discarded
		    clrf    currentModulo
		    
		    bcf	PIE2, 1		; TMR3 interrupt disabled
		    bcf	T3CON, 0	; TMR3 is stopped		    
		    return

		goBack_definetlyPressed:
		    bcf rb4ButtonState, 2
		    bsf	rb4ButtonState, 1
		    return
    
    update_screen_in_writeState:
	bcf shouldUpdateScreen, 0
	btfsc   whichScreen, 0  ; Is it Screen1 which must be drawn
	    bra draw_screen_1   ; Yes, then draw Screen1
	btfsc   whichScreen, 1  ; Is it Screen2 which must be drawn
	    bra draw_screen_2   ; Yes, then draw Screen2
	btfsc   whichScreen, 2  ; Is it Screen3 which must be drawn
	    bra draw_screen_3   ; Yes, then draw Screen3
	btfsc   whichScreen, 3  ; Is it Screen4 which must be drawn
	    bra draw_screen_4   ; Yes, then draw Screen4

	draw_screen_1:
	    clrf    LATA	    
	    	    
	    
	    movlw   HIGH load_digit_for_display_into_w
	    movwf   PCLATH
	    movf    countDownCounterTens, 0	; Move Tens Place into WREG
	    rlncf   WREG, f			; multiply by 2 since each instruction is 2 byte
	    call    load_digit_for_display_into_w		; Load corresponding 7-seg output into WREG	    
	    movwf   LATD			; Draw digit to the screen
	    
	    clrf	whichScreen
	    bsf	whichScreen, 1	; The next time, Screen2 will be drawn
	    bsf	    LATA, 2	    ; Select first (left-most) screen
	    
	    return	  
	draw_screen_2:
	    clrf    LATA	    
	    
	    
	    movlw   HIGH load_digit_for_display_into_w
	    movwf   PCLATH
	    movf    countDownCounterOnes, 0
	    rlncf   WREG, f
	    call    load_digit_for_display_into_w
	    movwf   LATD

	    clrf	whichScreen
	    bsf	whichScreen, 2	; Next, Screen3 will be drawn
	    ;bsf		whichScreen, 0
	    bsf	    LATA, 3	    ; Select second screen

	    return	   
	draw_screen_3:
	    clrf    LATA	    
	    
	    
	    movlw   HIGH load_letter_for_display_into_w
	    movwf   PCLATH
	    movf    lastWrittenLetterPosition, 0
	    rlncf   WREG, f
	    rlncf   WREG, f
	    call    load_last_letter_into_w
	    rlncf   WREG, f
	    call    load_letter_for_display_into_w
	    movwf   LATD
	    bsf	    LATA, 4	    ; Select third screen

	    clrf	whichScreen
	    bsf	whichScreen, 3	; Next, Screen3 will be drawn

	    return
	draw_screen_4:
	    clrf    LATA	    
	    
	    
	    movlw   HIGH load_letter_for_display_into_w
	    movwf   PCLATH
	    
	    movf    currentPushedButton, 0
	    mullw   0x03
	    movf    PRODL, 0
	    addwf   currentModulo, 0
	    
	    rlncf   WREG, f
	    call    load_letter_for_display_into_w
	    movwf   LATD
	    bsf	    LATA, 5	    ; Select fourth screen

	    clrf	whichScreen
	    bsf	whichScreen, 0	; Next, Screen3 will be drawn

	    return
reviewState:
    btfsc   shouldUpdateScreen, 0	    ; Is 5ms up ?
	call update_screen_in_reviewState	    ; Yes, then draw next 7-seg    
    goto main
    
    update_screen_in_reviewState:
	bcf shouldUpdateScreen, 0
	btfsc   whichScreen, 0  ; Is it Screen1 which must be drawn
	    bra draw_screen_1_in_reviewState   ; Yes, then draw Screen1
	btfsc   whichScreen, 1  ; Is it Screen2 which must be drawn
	    bra draw_screen_2_in_reviewState   ; Yes, then draw Screen2
	btfsc   whichScreen, 2  ; Is it Screen3 which must be drawn
	    bra draw_screen_3_in_reviewState   ; Yes, then draw Screen3
	btfsc   whichScreen, 3  ; Is it Screen4 which must be drawn
	    bra draw_screen_4_in_reviewState   ; Yes, then draw Screen4
	    
	
	draw_screen_1_in_reviewState:
	    clrf    LATA	    
	    
	    
	    movlw   HIGH load_letter_for_display_into_w
	    movwf   PCLATH
	    movf    reviewStateLeftMostLetter, 0
	    rlncf   WREG, f
	    rlncf   WREG, f
	    call    load_last_letter_into_w
	    rlncf   WREG, f
	    call    load_letter_for_display_into_w
	    movwf   LATD
	    bsf	    LATA, 2	    ; Select first screen

	    clrf	whichScreen
	    bsf	whichScreen, 1	; Next, Screen2 will be drawn

	    return
	    
	draw_screen_2_in_reviewState:
	    clrf    LATA	    
	    
	    
	    movlw   HIGH load_letter_for_display_into_w
	    movwf   PCLATH
	    movf    reviewStateLeftMostLetter, 0
	    addlw   0x1
	    rlncf   WREG, f
	    rlncf   WREG, f
	    call    load_last_letter_into_w
	    rlncf   WREG, f
	    call    load_letter_for_display_into_w
	    movwf   LATD
	    bsf	    LATA, 3	    ; Select second screen

	    clrf	whichScreen
	    bsf	whichScreen, 2	; Next, Screen3 will be drawn

	    return    
	   	   
	draw_screen_3_in_reviewState:
	    clrf    LATA	    
	    
	    
	    movlw   HIGH load_letter_for_display_into_w
	    movwf   PCLATH
	    movf    reviewStateLeftMostLetter, 0
	    addlw   0x2
	    rlncf   WREG, f
	    rlncf   WREG, f
	    call    load_last_letter_into_w
	    rlncf   WREG, f
	    call    load_letter_for_display_into_w
	    movwf   LATD
	    bsf	    LATA, 4	    ; Select third screen

	    clrf	whichScreen
	    bsf	whichScreen, 3	; Next, Screen4 will be drawn

	    return
	draw_screen_4_in_reviewState:
	    clrf    LATA	    
	    
	    
	    movlw   HIGH load_letter_for_display_into_w
	    movwf   PCLATH
	    movf    reviewStateLeftMostLetter, 0
	    addlw   0x3
	    rlncf   WREG, f
	    rlncf   WREG, f
	    call    load_last_letter_into_w
	    rlncf   WREG, f
	    call    load_letter_for_display_into_w
	    movwf   LATD
	    bsf	    LATA, 5	    ; Select fourth screen

	    clrf	whichScreen
	    bsf	whichScreen, 0	; Next, Screen0 will be drawn

	    return
	    
    
    
readState:
    init:
	clrf	readStateDirection  ; Default direction is to right
	
	movlw	0x1
	movwf	readStateLeftMostLetter	; Default left most is letter_1
	
	;****** TIMER3 SETUP for scrolling (1s) (Actually used for commit but the same logic) **********
	bsf	T3CON, 4	; 1:2 Prescale is set

	movlw	0x4D
	movwf	commitCounter

	movlw	0x3C
	movwf	TMR3H
	movlw	0xB0
	movwf	TMR3L		; Initial value for 10ms
	
	movf	PIR2		; Must have last-read value
	bcf	PIR2, 1		; so that TMR3IF can be cleared

	bsf	PIE2, 1		; TMR3 interrupt enabled
	bsf	T3CON, 0	; TMR3 is started
	;*****************************	
	
    
    infinite:
    btfsc   shouldUpdateScreen, 0	    ; Is 5ms up ?
	call update_screen_in_readState	    ; Yes, then draw next 7-seg    
    btfsc   commitCurrentLetter, 0	    ; Should letters be scrolled ?
	call scroll_letters	    ; Yes, scroll
    goto infinite
    
    scroll_letters:
	clrf	commitCurrentLetter
	
	;****** TIMER3 SETUP for scrolling (1s) (Actually used for commit but the same logic) **********
	bsf	T3CON, 4	; 1:2 Prescale is set

	movlw	0x4D
	movwf	commitCounter

	movlw	0x3C
	movwf	TMR3H
	movlw	0xB0
	movwf	TMR3L		; Initial value for 10ms
	
	movf	PIR2		; Must have last-read value
	bcf	PIR2, 1		; so that TMR3IF can be cleared

	bsf	PIE2, 1		; TMR3 interrupt enabled
	bsf	T3CON, 0	; TMR3 is started
	;*****************************
	
	btfss	readStateDirection, 0	; Should they be scrolled to the right ?
	    bra scroll_right		; Yes, 
	bra scroll_left		; No, 
	
	scroll_right:
	    movlw   0x3
	    cpfseq  readStateLeftMostLetter
		bra scroll_right_by_1
	    bra	change_dir_to_left
	    
	    scroll_right_by_1:
		incf	readStateLeftMostLetter
		return
	    change_dir_to_left:
		bsf	readStateDirection, 0
		decf	readStateLeftMostLetter
		return
		
	scroll_left:
	    movlw   0x1
	    cpfseq  readStateLeftMostLetter
		bra scroll_left_by_1
	    bra	change_dir_to_right
	    
	    scroll_left_by_1:
		decf	readStateLeftMostLetter
		return
	    change_dir_to_right:
		bcf	readStateDirection, 0
		incf	readStateLeftMostLetter
		return
    
    update_screen_in_readState:
	bcf shouldUpdateScreen, 0
	btfsc   whichScreen, 0  ; Is it Screen1 which must be drawn
	    bra draw_screen_1_in_readState	; Yes, then draw Screen1
	btfsc   whichScreen, 1  ; Is it Screen2 which must be drawn
	    bra draw_screen_2_in_readState	; Yes, then draw Screen2
	btfsc   whichScreen, 2  ; Is it Screen3 which must be drawn
	    bra draw_screen_3_in_readState	; Yes, then draw Screen3
	btfsc   whichScreen, 3  ; Is it Screen4 which must be drawn
	    bra draw_screen_4_in_readState	; Yes, then draw Screen4
	    
	
	draw_screen_1_in_readState:
	    clrf    LATA	    
	    
	    
	    movlw   HIGH load_letter_for_display_into_w
	    movwf   PCLATH
	    movf    readStateLeftMostLetter, 0
	    rlncf   WREG, f
	    rlncf   WREG, f
	    call    load_last_letter_into_w
	    rlncf   WREG, f
	    call    load_letter_for_display_into_w
	    movwf   LATD
	    bsf	    LATA, 2	    ; Select first screen

	    clrf	whichScreen
	    bsf	whichScreen, 1	; Next, Screen2 will be drawn

	    return
	    
	draw_screen_2_in_readState:
	    clrf    LATA	    
	    
	    
	    movlw   HIGH load_letter_for_display_into_w
	    movwf   PCLATH
	    movf    readStateLeftMostLetter, 0
	    addlw   0x1
	    rlncf   WREG, f
	    rlncf   WREG, f
	    call    load_last_letter_into_w
	    rlncf   WREG, f
	    call    load_letter_for_display_into_w
	    movwf   LATD
	    bsf	    LATA, 3	    ; Select second screen

	    clrf	whichScreen
	    bsf	whichScreen, 2	; Next, Screen3 will be drawn

	    return    
	   	   
	draw_screen_3_in_readState:
	    clrf    LATA	    
	    
	    
	    movlw   HIGH load_letter_for_display_into_w
	    movwf   PCLATH
	    movf    readStateLeftMostLetter, 0
	    addlw   0x2
	    rlncf   WREG, f
	    rlncf   WREG, f
	    call    load_last_letter_into_w
	    rlncf   WREG, f
	    call    load_letter_for_display_into_w
	    movwf   LATD
	    bsf	    LATA, 4	    ; Select third screen

	    clrf	whichScreen
	    bsf	whichScreen, 3	; Next, Screen4 will be drawn

	    return
	draw_screen_4_in_readState:
	    clrf    LATA	    
	    
	    
	    movlw   HIGH load_letter_for_display_into_w
	    movwf   PCLATH
	    movf    readStateLeftMostLetter, 0
	    addlw   0x3
	    rlncf   WREG, f
	    rlncf   WREG, f
	    call    load_last_letter_into_w
	    rlncf   WREG, f
	    call    load_letter_for_display_into_w
	    movwf   LATD
	    bsf	    LATA, 5	    ; Select fourth screen

	    clrf	whichScreen
	    bsf	whichScreen, 0	; Next, Screen0 will be drawn

	    return
    
org	0x1000
load_letter_for_display_into_w:
    addwf   PCL, f	    ; Advance by WREG
    draw_underscore:
	retlw	b'00001000'
    nop
    nop
    
    nop
    nop
    nop
    draw_a:
	retlw   b'01011111'
    draw_b:
	retlw   b'01111100'
    draw_c:
	retlw   b'01011000'
    draw_d:
	retlw   b'01011110'
    draw_e:
	retlw   b'01111011'
    draw_f:
	retlw   b'01110001'
    draw_g:
	retlw   b'01101111'
    draw_h:
	retlw   b'01110100'
    draw_i:
	retlw   b'00000100'
    draw_j:
	retlw   b'00001110'
    draw_k:
	retlw   b'01110101'
    draw_l:
	retlw   b'00111000'
    draw_m:
	retlw   b'01010101'
    draw_n:
	retlw   b'01010100'
    draw_o:
	retlw   b'01011100'
    draw_p:
	retlw   b'01110011'
    draw_r:
	retlw   b'01010000'
    draw_s:
	retlw   b'01100100'
    draw_t:
	retlw   b'01111000'
    draw_u:
	retlw   b'00011100'
    draw_v:
	retlw   b'00101010'
    draw_y:
	retlw   b'01101110'
    draw_z:
	retlw   b'01011011'
    draw_space:
	retlw	b'00000000'
load_digit_for_display_into_w:
    addwf   PCL, f	    ; Advance by WREG
    draw_0:
	retlw   b'00111111'
    draw_1:
	retlw   b'00000110'
    draw_2:
	retlw   b'01011011'
    draw_3:
	retlw   b'01001111'
    draw_4:
	retlw   b'01100110'
    draw_5:
	retlw   b'01101101'
    draw_6:
	retlw   b'01111100'
    draw_7:
	retlw   b'00000111'
    draw_8:
	retlw   b'01111111'
    draw_9:
	retlw   b'01100111'
load_last_letter_into_w:
    addwf   PCL, f   
    no_character_yet:
	movlw	b'00000000'
	return
    load_letter_1:
	movf	letter_1, 0
	return
    load_letter_2:
	movf	letter_2, 0
	return
    load_letter_3:
	movf	letter_3, 0
	return
    load_letter_4:
	movf	letter_4, 0
	return
    load_letter_5:
	movf	letter_5, 0
	return
    load_letter_6:
	movf	letter_6, 0
	return

	
org 0x2000


end