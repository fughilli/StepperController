#include <msp430.h>
#include "simple_io.h"
/*
 *	TODO:
 *	Save stepper state index to flash when driver shuts off
 *	Reload stepper state index when driver starts back up
 */

/*
 * Speed, in cm/s
 *
 * 1.8 deg/step
 * 200 steps/turn
 * 20 turns/in
 * 4000 steps/in
 *
 * 6.35*10^-4 cm/step
 *
 * 250000 TimerA cycles/sec
 *
 * Period/250000 seconds/step
 * 250000/Period steps/second
 *
 * 158.75/Period cm/second
 *
 */
/*-----Stepper state variables-----*/
#define STEPPER_ENABLE_PIN_MASK			0x0021			// These pins enable the L298N
#define STEPPER_ENABLE_A_PIN			0				// This pin is connected to the L298N's channel A enable pin
#define STEPPER_ENABLE_B_PIN			1				// This pin is connected to the L298N's channel B enable pin

#define STEPPER_ENDSTOP_PIN_MASK		0x0300			// These pins are connected to the endstops
#define STEPPER_ENSTOP_A_PIN			15				// This pin is connected to the low endstop
#define STEPPER_ENSTOP_B_PIN			14				// This pin is connected to the high endstop

/* 4 stepper states: AB, A'B, A'B', AB'
 *
 * A = Coil A, powered forward
 * A' = Coil A, powered in reverse
 * B = Coil B, powered forward
 * B' = Coil B, powered in reverse
 */
#define STEPPER_STEP_PINS				0x001E
#define STEPPER_STEP_IN1				2
#define STEPPER_STEP_IN2				3
#define STEPPER_STEP_IN3				4
#define STEPPER_STEP_IN4				5

#define STEPPER_NUMBER_STEPPER_MODES	4

const unsigned char STEPPER_MODE_MASKS[] = {
		0x14,											// A B 			0b00010100
		0x12,											// A'B 			0b00010010
		0x0A,											// A'B'			0b00001010
		0x0C											// A B'			0b00001100
};
/*-----I2C state variables-----*/
#define I2C_STEPPER_ENABLE				0xA1
#define I2C_STEPPER_DISABLE				0xA2
#define I2C_STEPPER_BEGIN				0xA3
#define I2C_STEPPER_PAUSE				0xA4
#define I2C_STEPPER_RESUME				0xA5
#define I2C_STEPPER_STOP				0xA6
#define I2C_READ_PERIOD					0xB1
#define I2C_READ_STEPS					0xB2
#define I2C_READ_SETTINGS				0xB3
volatile unsigned short I2C_State =		0;
volatile unsigned char R_I2C_Expected = 0;

volatile unsigned int SLV_Addr =		0x2D;			//0x2A (42) << 1

volatile unsigned int BRDCST_Addr =		0x55;			//0x55 (85) << 1
volatile unsigned int MST_Data =		0x01;
volatile unsigned short RW_I2C_Mode =	0;				//write
/*-----Data variables-----*/
volatile unsigned char RXIndex =		0;
volatile unsigned char TXIndex =		0;
volatile unsigned char RXData =			0;
#define MAX_CTL_POINTER 				8
#define MIN_CTL_POINTER 				0
volatile unsigned char STEPPER_CTL_BUFFER_POINTER = 0;
volatile unsigned char STEPPER_CTL_POINTER = 0;
volatile unsigned char STEPPER_CTL[10];
volatile unsigned char STEPPER_CTL_BUFFER[10];

/*
 * P = STEP PERIOD
 *
 * +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 * | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P |
 * +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 */
#define MAX_TIMER_VALUE 65535
#define STEPPER_CTL_PERIOD_POINTER_VALUE 0
volatile unsigned short STEPPER_PERIOD_OVERFLOW_COMPENSATED = 0;
#define STEPPER_PERIOD (*((unsigned int *)(STEPPER_CTL + STEPPER_CTL_PERIOD_POINTER_VALUE)))
#define STEPPER_PERIOD_BUFFER (*((unsigned int *)(STEPPER_CTL_BUFFER + STEPPER_CTL_PERIOD_POINTER_VALUE)))
/*
 * S = STEP COUNTER
 *
 * +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 * | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S | S |
 * +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 */
#define STEPPER_CTL_STEP_COUNTER_POINTER_VALUE 4
#define STEPPER_STEP_COUNTER (*((unsigned int *)(STEPPER_CTL + STEPPER_CTL_STEP_COUNTER_POINTER_VALUE)))
#define STEPPER_STEP_COUNTER_BUFFER (*((unsigned int *)(STEPPER_CTL_BUFFER + STEPPER_CTL_STEP_COUNTER_POINTER_VALUE)))
/*
 * I = STEPPER MODE INDEX
 * D = STEP DIRECTION
 * H = HOLD POSITION
 *
 * +---+---+---+---+---+---+---+---+
 * |   |   |   |   | D | H | I | I |
 * +---+---+---+---+---+---+---+---+
 */
#define STEPPER_CTL_CONTROL_POINTER_VALUE 8
#define STEPPER_CONTROL (*((unsigned int *)(STEPPER_CTL + STEPPER_CTL_CONTROL_POINTER_VALUE)))
#define STEPPER_CONTROL_BUFFER (*((unsigned int *)(STEPPER_CTL_BUFFER + STEPPER_CTL_CONTROL_POINTER_VALUE)))

#define STEPPER_CONTROL_MODE_INDEX_MASK 			0x0003
#define STEPPER_CONTROL_DIRECTION_MASK				0x0008
#define STEPPER_CONTROL_HOLD_POSITION_MASK			0x0004


#define STEPPER_INDEX (STEPPER_CONTROL & STEPPER_CONTROL_MODE_INDEX_MASK)

int main(void) {

	P1DIR = 0x3F;
	P1OUT = 0;

	WDTCTL = WDTPW | WDTHOLD;							// Stop watchdog timer

	/*SET UP CLOCKS*/
	BCSCTL1 = CALBC1_16MHZ;								// Set range
	DCOCTL = CALDCO_16MHZ;								// Set DCO frequency

	BCSCTL2 = DIVS_3;									// Divide SMCLK by 8 from DCO

	/*SET UP USCI (I2C)*/
	USICTL0 = USIPE6 + USIPE7 + USISWRST;				//enable pins 1.6 and 1.7 for use with the USI module (I2C), MSB first
	USICTL1 = USII2C + USISTTIE + USIIE;				//set the USI module to I2C mode and enable the start condition interrupt
	USICKCTL = USICKPL;									//SCL is inactive high
	USICNT |= USIIFGCC;                  				// Disable automatic clear control

	/*SET UP TIMERA*/
	TACTL = TASSEL_2 + ID_3 + MC_0;						// Timer A, Source SMCLK, /8 (Total /64 from DCO), Stopped
	TACCR0 = 1;											// Next interrupt immediately
	TACCTL0 = CCIE;										// Enable CC0 interrupt

	USICTL0 &= ~USISWRST;                				// Enable USI
	USICTL1 &= ~USIIFG;                  				// Clear pending flag

	_BIS_SR(GIE | LPM0_bits);							// Enable interrupts, shut the CPU off

	while(1);											// CPU should never get here because of LPM0

	return 0;
}

#pragma vector=PORT2_VECTOR,PORT1_VECTOR,ADC10_VECTOR,TIMERA1_VECTOR,WDT_VECTOR,NMI_VECTOR
__interrupt void ISR_trap(void)
{
	// the following will cause an access violation which results in a PUC reset
	WDTCTL = 0;
}

#pragma vector = TIMERA0_VECTOR
__interrupt void Stepper_SR (void) {
	/*
	 * Stop TimerA
	 * Adjust period
	 * Set pin levels
	 * Decrement step counter
	 */
	if(STEPPER_PERIOD_OVERFLOW_COMPENSATED > MAX_TIMER_VALUE) {
		TACCR0 = MAX_TIMER_VALUE;						// Set TimerA period
		STEPPER_PERIOD_OVERFLOW_COMPENSATED -= MAX_TIMER_VALUE;
	}else{
		TACCR0 = STEPPER_PERIOD_OVERFLOW_COMPENSATED;
		STEPPER_PERIOD_OVERFLOW_COMPENSATED = STEPPER_PERIOD;
	}

	P1OUT &= ~STEPPER_STEP_PINS;					// Clear out the levels on the pins connected to IN1-IN4 on the driver board
	P1OUT |= STEPPER_MODE_MASKS[STEPPER_INDEX];			// Set the IO levels according to the current stepper mode

	if(STEPPER_CONTROL & STEPPER_CONTROL_DIRECTION_MASK) {			// If CCW
		if(STEPPER_INDEX == 0x0003) {				// If the stepper mode index reached 3 this time
			STEPPER_CTL2 &= ~0x0003;				// Set stepper mode index back to 0
		} else {
			STEPPER_CTL2++;							// Increment stepper mode index
		}
	} else {										// If CW
		if(STEPPER_INDEX == 0x0000) {				// If the stepper mode index reached 0 last time
			STEPPER_CTL2 |= 0x0003;					// Set stepper mode index back to 3
		} else {
			STEPPER_CTL2--;							// Decrement stepper mode index
		}
	}
	STEPPER_STEP_COUNTER--;							// Decrement the step counter

	if(STEPPER_CTL1 == 0) {							// If there are no more steps to take
		TACTL &= ~MC_3;								// Stop the timer
		if(!(STEPPER_CTL2 & STEPPER_HOLD_POSITION)) {
			P1OUT &= ~STEPPER_ENABLE_PINS;			// Disable stepper driver
		}
	}

}

#pragma vector=USI_VECTOR
__interrupt void I2C_SR(){							// Process I2C events
	if (USICTL1 & USISTTIFG)             			//Start entry?
	{
		I2C_State = 2;                     			//Enter 1st state on start
	}
	switch(I2C_State)
	{
	case 0: {									// Idle, should not get here
		break;
	}

	case 2: {									// RX Address
		USICNT = (USICNT & 0xE0) + 0x08; 			// Bit counter = 8, RX address
		USICTL1 &= ~USISTTIFG;   					// Clear start flag
		I2C_State = 4;           					// Go to next state: check address
		break;
	}

	case 4: {									// Process Address and send (N)Acknowledge
		if (((USISRL & 0x01) == 1) && R_I2C_Expected)
		{											// If read...
			RW_I2C_Mode = 1;						// Save R/W bit
			R_I2C_Expected = 0;
		}else{
			RW_I2C_Mode = 0;
			R_I2C_Expected = 0;
		}
		USICTL0 |= USIOE;        					// SDA = output
		if (((USISRL & ~0x01) == (SLV_Addr<<1)) |
				(USISRL == (BRDCST_Addr<<1)))  		// Address match?
		{
			USISRL = 0x00;        					// Send Acknowledge
			I2C_State = 8 + RW_I2C_Mode;			// Go to next state: RX data
		}
		else
		{
			USISRL = 0xFF;      					// Send NAck
			I2C_State = 6;         					// Go to next state: prepare for next Start
		}
		USICNT |= 0x01;       						// Bit counter = 1, send (N)Acknowledge bit
		break;
	}

	case 6: {									// Prepare for Start condition
		USICTL0 &= ~USIOE;       					// SDA = input
		RW_I2C_Mode = 0;         					// Reset slave address
		I2C_State = 0;           					// Reset state machine
		break;
	}

	case 8: { 									// Receive data byte
		USICTL0 &= ~USIOE;       					// SDA = input
		USICNT |=  0x08;         					// Bit counter = 8, RX data
		I2C_State = 10;			          			// Go to next state: Test data and (N)Acknowledge
		break;
	}

	case 10: {									// Check Data & TX (N)Acknowledge
		USICTL0 |= USIOE;        					// SDA = output
		RXData = USISRL;							// Store data to RXData
		switch(RXIndex){
		case 0:{
			if(RXData == I2C_STEPPER_BEGIN) {
				if(STEPPER_CTL1_buffer > 0){		// If there are any steps commanded to execute
					STEPPER_CTL0 = STEPPER_CTL0_buffer;	// Unload register buffers
					STEPPER_CTL1 = STEPPER_CTL1_buffer;
					STEPPER_CTL2 = ((STEPPER_CTL2_buffer & 0xFFFC) | (STEPPER_CTL2 & 0x0003));	// Preserve bottom 2 bits (stepper state index)
					STEPPER_CTL0_buffer = STEPPER_CTL1_buffer = STEPPER_CTL2_buffer = 0;		// Clear buffers
					P1OUT |= STEPPER_ENABLE_PINS;	// Enable stepper driver
					TACCR0 = 1;						// Interrupt occurs immediately
					TACTL |= MC_1;					// Start TimerA
					I2C_State = 6;					// Done receiving
					RXIndex = 0;
				} else {							// No steps commanded, "We're done here!"
					I2C_State = 6;					// Done receiving
					RXIndex = 0;
				}
			} else if(RXData == I2C_STEPPER_PAUSE) {
				TACCR0 = 0;							// Stop count
				TACTL &= ~MC_3;						// Stop count
				I2C_State = 6;						// Done receiving
				RXIndex = 0;
			} else if(RXData == I2C_STEPPER_RESUME) {
				P1OUT |= STEPPER_ENABLE_PINS;		// Enable stepper driver
				TACCR0 = 1;							// Interrupt occurs immediately
				TACTL |= MC_1;						// Start TimerA
				I2C_State = 6;						// Done receiving
				RXIndex = 0;
			} else if(RXData == I2C_STEPPER_ENABLE) {
				P1OUT |= STEPPER_ENABLE_PINS;		// Enable stepper driver
				I2C_State = 6;						// Done receiving
				RXIndex = 0;
			} else if(RXData == I2C_STEPPER_DISABLE) {
				P1OUT &= ~STEPPER_ENABLE_PINS;		// Disable stepper driver
				I2C_State = 6;						// Done receiving
				RXIndex = 0;
			} else if(RXData == I2C_READ_STEPS){
				R_I2C_Expected = 1;
				I2C_State = 6;						// Go to transmit case
				RXIndex = 0;
			} else {
				I2C_State = 8;						// Go for another byte
				RXIndex++;
			}
			break;
		}
		case 1:{
			STEPPER_CTL0_buffer = RXData;			// Load buffer
			STEPPER_CTL0_buffer <<= 8;
			I2C_State = 8;							// Go for another byte
			RXIndex++;
			break;
		}
		case 2:{
			STEPPER_CTL0_buffer |= RXData;			// Load buffer
			I2C_State = 8;							// Go for another byte
			RXIndex++;
			break;
		}
		case 3:{
			STEPPER_CTL1_buffer = RXData;			// Load buffer
			STEPPER_CTL1_buffer <<= 8;
			I2C_State = 8;							// Go for another byte
			RXIndex++;
			break;
		}
		case 4:{
			STEPPER_CTL1_buffer |= RXData;			// Load buffer
			I2C_State = 8;							// Go for another byte
			RXIndex++;
			break;
		}
		case 5:{
			STEPPER_CTL2_buffer = RXData;			// Load buffer
			STEPPER_CTL2_buffer <<= 8;
			I2C_State = 8;							// Go for another byte
			RXIndex++;
			break;
		}
		case 6:{
			STEPPER_CTL2_buffer |= RXData;			// Load buffer
			I2C_State = 6;							// Done receiving
			RXIndex = 0;
			break;
		}
		}
		USISRL = 0x00;    		     				// Send Acknowledge
		USICNT |= 0x01;         		 			// Bit counter = 1, send (N)Acknowledge bit
		break;
	}

	case 9: {									// Check (N)Acknowledge, Transmit data byte
		RW_I2C_Mode = 0;
		if(TXIndex > 0){
			if(USISRL & 0x01){						// If NAcknowledged
				I2C_State = 6;						// Done transmitting
				TXIndex = 0;
				break;
			}
		}
		USICTL0 |= USIOE;        					// SDA = output
		switch(TXIndex){
		case 0:{
			STEPPER_CTL1_buffer = STEPPER_CTL1;		// Load output buffer
			USISRL = 								// Transmit the upper byte
					((STEPPER_CTL1_buffer >> 8) & 0xFF);
			I2C_State = 11;							// Receive (N)Acknowledge bit
			TXIndex++;
			break;
		}
		case 1:{
			USISRL = (STEPPER_CTL1_buffer & 0xFF);	// Transmit the lower byte
			TXIndex++;
			I2C_State = 11;							// Receive (N)Acknowledge bit
			break;
		}
		case 2:{
			USISRL = 0x00;							// Transmit blanks
			I2C_State = 11;							// Receive (N)Acknowledge bit
			break;
		}
		}
		USICNT |= 0x08;
		break;
	}

	case 11: {									// Receive (N)Acknowledge
		USICTL0 &= ~USIOE;
		USICNT |= 0x01;
		I2C_State = 9;							// Hop back to check (N)Acknowledge and transmit more data
		break;
	}
	}
	USICTL1 &= ~USIIFG;                  			// Clear pending flag
}
