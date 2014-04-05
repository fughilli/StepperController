/*
 * main.c
 *
 * AUTHOR: Kevin Balke
 *
 * (c) 2012-2014 Kevin Balke. All rights reserved.
 *
 * DESCRIPTION: I2C Servo Control Driver
 * I2C data to be presented in the form of 2 bytes per servo:
 *
 * 1st Data Byte:
 *
 * +---+---+---+---+---+---+---+---+
 * | s | s | s | x | d | d | d | d |
 * +---+---+---+---+---+---+---+---+
 *
 * 2nd Data Byte:
 *
 * +---+---+---+---+---+---+---+---+
 * | d | d | d | d | d | d | d | d |
 * +---+---+---+---+---+---+---+---+
 *
 * s = servo index 	(0 <= s <= 3)
 * d = target position 	(0 <= d <= 499)
 * x = don't care
 *
 * Device Address: 0x39 (57 in decimal)
 *
 * Control signal output in the form of 8 channel PWM with feedback running at 50Hz per channel, 12 bit resolution (pulse widths offset)
 */
#include <msp430.h>
#include "simple_io.h"
#include "simple_math.h"

#define PWM_FREQUENCY 50
#define TIMER_A_DIVIDER 32
const unsigned int SLV_Addr = 0x39;					// 0x39 (57)

/*-----Period, in TACLK cycles-----*/
#define PWM_PERIOD (CLOCK_SPEED / (PWM_FREQUENCY * TIMER_A_DIVIDER))

#define BASEBAND_TIME_US 		(544)
#define MAXBAND_TIME_US 		(2400)

#define BASEBAND_CLK_TIME 		(((CLOCK_SPEED/1000000) * BASEBAND_TIME_US) / (TIMER_A_DIVIDER)) // 272
#define MAXBAND_CLK_TIME 		(((CLOCK_SPEED/1000000) * MAXBAND_TIME_US) / (TIMER_A_DIVIDER)) // 1200

#define MAXBAND_CLK_TIME_DIFF (MAXBAND_CLK_TIME - BASEBAND_CLK_TIME)
/*-----Output pin configuration-----*/
#define NUM_SERVOS 8
const unsigned char PWM_PINS[] =
{ 0, 1, 2, 3, 4, 5, 14, 15 };
/*-----Input pin configuration-----*/
#define NUM_POTS 0
const unsigned char POT_PINS[] =
{0};
/*-----Rotation range constants-----*/
#define MAX_POS MAXBAND_CLK_TIME_DIFF
#define CENTER_POS (MAXBAND_CLK_TIME_DIFF/2)
#define MIN_POS 0
/*-----PWM state variables-----*/
volatile unsigned short current_servo = 0;
volatile unsigned short current_pot = 0;
volatile unsigned short servo_control_case = 0;		// 0 = deadband, 1 = signal
/*-----PWM event times-----*/
volatile unsigned int next_pwm_event = 0;
/*-----I2C state variables-----*/
volatile unsigned short I2C_State = 0;
#define POS_BUFFER_READY 	0x02
#define I2C_READ 			0x01
volatile unsigned short I2C_CONTROL = 0x00;			// Write, buffer not ready
volatile unsigned int servo_pos[NUM_SERVOS];
volatile unsigned int pot_pos[NUM_SERVOS];
volatile unsigned int RXData = 0;
volatile unsigned short RXMode = 0;
volatile unsigned short TXMode = 0;
volatile unsigned short selected_servo = 0;
volatile unsigned int servo_pos_buffer = 0;
/* TEST */
volatile unsigned int ACK_CNT = 0;
/* MAIN */
int main(void)
{
	WDTCTL = WDTPW + WDTHOLD;							// Stop watchdog timer

	BCSCTL1 = CALBC1_16MHZ;		// Set main clock to run from the DCO at 16 MHz
	DCOCTL = CALDCO_16MHZ;

	BCSCTL2 = DIVS_3;
	/*SET UP TIMER A*/
	TACTL = TASSEL_2 + ID_2 + MC_1;	// Clock TACLK from SMCLK, SMCLK clocked from DCO by default; divide SMCLK by 8, count up mode
	CCR0 = PWM_PERIOD;					// Wait one period before first TA event
	/*SET UP USCI (I2C)*/
	USICTL0 = USIPE6 + USIPE7 + USISWRST;// Enable pins 1.6 and 1.7 for use with the USI module (I2C), MSB first
	USICTL1 = USII2C + USISTTIE + USIIE;// Set the USI module to I2C mode and enable the start condition interrupt
	USICKCTL = USICKPL;									// SCL is inactive high
	USICNT |= USIIFGCC;                  	// Disable automatic clear control
#if (NUM_POTS)
			/*SET UP ADC10*/
			ADC10CTL0 = SREF_0 + ADC10SR + REFBURST; // Reference voltages set to Vcc and Vss
			ADC10CTL1 = ADC10DIV_0 + ADC10SSEL_0 + CONSEQ_0 + SHS_0;// No division on ADC10OSC, single-channel-single-conversion
			ADC10AE0 = 0;
#endif
	/*SET UP I/O*/INIT_PORT1;
	INIT_PORT2;

	unsigned int i;
	for (i = NUM_SERVOS-1; i; i++)
	{
		set_pin_output(PWM_PINS[i]);
		servo_pos[i] = CENTER_POS;
	}
#if (NUM_POTS)
	for(i = NUM_POTS-1; i; i++)
	{
		set_pin_analog_input(POT_PINS[i]);
		pot_pos[i] = 0;
	}

	select_analog_channel(POT_PINS[0]);
#endif

	TACTL |= TAIE;									// Enable Timer A interrupts
	CCTL0 |= CCIE;					// Enable Capture/Compare block 0 interrupt
#if (NUM_POTS)
			ADC10CTL0 |= ADC10IE + ADC10ON + ENC;	// Enable ADC10 interrupt
#endif
	USICTL0 &= ~USISWRST;                				// Enable USI
	USICTL1 &= ~USIIFG;                  				// Clear pending flag

	_BIS_SR(GIE | LPM0_bits);
	// Set general interrupt enable bit

	while (1)
		;											// Trap in main loop

	return 0;
}

#pragma vector=PORT2_VECTOR,PORT1_VECTOR,TIMERA1_VECTOR,WDT_VECTOR,NMI_VECTOR
__interrupt void ISR_trap(void)
{
	// the following will cause an access violation which results in a PUC reset
	WDTCTL = 0;
}

/* called when conversion result is ready */
#pragma vector=ADC10_VECTOR
__interrupt void handle_adc()
{
#if (NUM_POTS)
	ADC10CTL0 &= ~ENC;
	pot_pos[current_pot] = ADC10MEM;
	current_pot = (current_pot + 1) % NUM_POTS;
	select_analog_channel(POT_PINS[current_pot]);
	ADC10CTL0 |= ENC;
	//pot_pos[1] = ADC10MEM;
#endif
}

/* called when timer A reaches CCR0 */
#pragma vector=TIMERA0_VECTOR
__interrupt void handle_pwm()
{
	//TACTL = TASSEL_2 + ID_2 + MC_0;								//stop timer
	_BIC_SR(GIE);
	//clear general interrupt enable bit
	if (servo_control_case == 0)
	{								//if deadband just finished
		set_pin(PWM_PINS[current_servo]);
		//set the current servo's control pin high
		next_pwm_event = BASEBAND_CLK_TIME + servo_pos[current_servo];//set the next event to occur BASEBAND_CLK_TIME + the servo control width later
		servo_control_case = 1;
	}
	else
	{													//if pulse just finished
		clear_pin(PWM_PINS[current_servo]);
		//set the current servo's control pin low
		/* set the next event to occur at the end of the deadband that
		 * fills the rest of the PWM_PERIOD/NUM_SERVOS time (divide by
		 * NUM_SERVOS to control x servos one after the other)
		 */
		next_pwm_event = (PWM_PERIOD / NUM_SERVOS) - next_pwm_event;
		current_servo = (current_servo + 1) % NUM_SERVOS;//next servo next time
		if (I2C_CONTROL == POS_BUFFER_READY)
		{					//if the data from the i2c master has been received
			I2C_CONTROL &= ~POS_BUFFER_READY;					//clear flag
			// Clip the range of servo_pos_buffer
			if (servo_pos_buffer > MAXBAND_CLK_TIME_DIFF)
			{
				servo_pos_buffer = MAXBAND_CLK_TIME_DIFF;
			}
			servo_pos[selected_servo] = servo_pos_buffer;//move the data from the buffer into the servo control array
		}
		servo_control_case = 0;
	}
	CCR0 = next_pwm_event;
	//TACTL = TASSEL_2 + ID_2 + MC_1;								//start timer
#if (NUM_POTS)
			ADC10CTL0 |= ADC10SC;
#endif
	_BIS_SR_IRQ(GIE);
}

#pragma vector=USI_VECTOR
__interrupt void receive_data()
{
	if (USICTL1 & USISTTIFG)             	//Start entry?
	{
		I2C_State = 2;                     	//Enter 1st state on start
	}
	switch (I2C_State)
	{
	case 0: 							// Idle, should not get here
		break;

	case 2: 							// RX Address
		USICNT = (USICNT & 0xE0) + 0x08; 	// Bit counter = 8, RX address
		USICTL1 &= ~USISTTIFG;   			// Clear start flag
		I2C_State = 4;           			// Go to next state: check address
		break;

	case 4: 						// Process Address and send (N)Acknowledge
		if ((USISRL & 0x01) == 1)
		{       	// If read...
			I2C_CONTROL |= I2C_READ;		// Save R/W bit
			USISRL &= ~0x01;
		}
		USICTL0 |= USIOE;        			// SDA = output
		if (USISRL == (SLV_Addr << 1))  		// Address match?
		{
			USISRL = 0x00;        			// Send Acknowledge
			if (I2C_CONTROL & I2C_READ)
			{
				I2C_State = 5;				// Go to next state: TX Data
			}
			else
			{
				I2C_State = 8;				// Go to next state: RX data
			}
		}
		else
		{
			USISRL = 0xFF;      			// Send NAck
			I2C_State = 6;         	// Go to next state: prepare for next Start
		}
		USICNT |= 0x01;       		// Bit counter = 1, send (N)Acknowledge bit
		break;

	case 6: 							// Prepare for Start condition
		USICTL0 &= ~USIOE;       			// SDA = input
		I2C_CONTROL &= ~I2C_READ;         	// Reset slave address
		I2C_State = 0;           			// Reset state machine
		TXMode = 0;
		RXMode = 0;
		break;

	case 5:								// Transmit status data
		USICTL0 |= USIOE;					// SDA = output
		if (TXMode & 1)
		{						// Check if TXdata is odd
			USISRL = pot_pos[TXMode];		// Send potentiometer position
		}
		else
		{
			USISRL = servo_pos[TXMode / 2];	// Send servo position
		}
		TXMode++;
		USICNT |= 0x08;						// One byte to send
		I2C_State = 7;						// Receive (N)Acknowledge
		break;

	case 7:								// Receive (N)Acknowledge
		USICTL0 &= ~USIOE;					// SDA = input
		USISRL = 0;							// Clear shift register
		USICNT |= 0x01;						// One bit
		I2C_State = 9;						// Process (N)Acknowledge
		break;

	case 8: 							// Receive data byte
		USICTL0 &= ~USIOE;       			// SDA = input
		USICNT |= 0x08;         			// Bit counter = 8, RX data
		I2C_State = 10;        // Go to next state: Test data and (N)Acknowledge
		break;

	case 9:								// Process (N)Acknowledge
		if (TXMode < (NUM_SERVOS + NUM_POTS))
		{
			if (USISRL & 0xFF)
			{				// Received NAcknowledge
				I2C_State = 6;				// Done, wait for next start
				I2C_CONTROL &= ~I2C_READ;
				ACK_CNT = 0;
			}
			else
			{							// Received Acknowledge
				I2C_State = 5;				// Send another data byte
				ACK_CNT++;
			}
		}
		else
		{
			I2C_State = 6;
			I2C_CONTROL &= ~I2C_READ;
		}
		break;

	case 10:							// Check Data & TX (N)Acknowledge
		USICTL0 |= USIOE;        			// SDA = output
		RXData = USISRL;
		switch (RXMode)
		{
		case 0:
		{
			selected_servo = ((RXData & 0xE0) >> 5); //mask upper 2 bits and shift down
			servo_pos_buffer = (RXData & 0x0F);			//mask lower 4 bits
			servo_pos_buffer <<= 8;			//shift bits up into correct place
			I2C_State = 8;					//prepare to receive another byte
			RXMode++;
			break;
		}
		case 1:
		{
			servo_pos_buffer |= RXData;	//complete position control buffer with lower 8 bits of data
			I2C_CONTROL |= POS_BUFFER_READY;		//flag the data as ready
			I2C_State = 6;
			//I2C_State = 0;
			//ReadWrite_I2C_Mode = 0; //wasn't here before
			RXMode = 0;
			break;
		}
		}
		USISRL = 0x00;         			// Send Acknowledge
		USICNT |= 0x01;          	// Bit counter = 1, send (N)Acknowledge bit

		break;
	}
	USICTL1 &= ~USIIFG;                  	// Clear pending flags
}
