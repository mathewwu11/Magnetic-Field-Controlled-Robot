#include "lpc824.h"
#include "serial.h"
#include "lcd.h"

// LPC824 pinout:
//                             --------
//     PIO0_23/ADC_3/ACMP_I4 -|1     20|- PIO0_14/ADC_2/ACMP_I3
//             PIO0_17/ADC_9 -|2     19|- PIO0_0/ACMP_I1/TDO
//            PIO0_13/ADC_10 -|3     18|- VREFP
//                   PIO0_12 -|4     17|- VREFN
//              RESET/PIO0_5 -|5     16|- VSS
// PIO0_4/ADC_11/WAKEUP/TRST -|6     15|- VDD
//          SWCLK/PIO0_3/TCK -|7     14|- PIO0_8/XTALIN
//          SWDIO/PIO0_2/TMS -|8     13|- PIO0_9/XTALOUT
//          PIO0_11/I2C0_SDA -|9     12|- PIO0_1/ACMP_I2/CLKIN/TDI
//          PIO0_10/I2C0_SCL -|10    11|- PIO0_15
//                             --------
// WARNING pins 9 and 10 are OPEN DRAIN.  They need external pull-up resistors to VDD if used
// as outputs. 1kohm seems to work.
//
// Reserved pins:
// Pin 4:  BOOT button
// Pin 5:  Reset
// Pin 6:  TXD
// Pin 19: RXD

#define SYSTEM_CLK 30000000L
#define DEFAULT_F 14910L

#define OUT0 GPIO_B15
#define OUT1 GPIO_B1

#define TRACK_B GPIO_B13

unsigned int count = 0;
unsigned int direction = 0;

// Configure the pins as outputs
void ConfigPins(void)
{
	GPIO_DIR0 |= BIT14; 
	GPIO_DIR0 |= BIT15; 
	GPIO_DIR0 |= BIT1;

	GPIO_DIR0 &= ~(BIT13); 	// Configure PIO0_13 as input.

	// Disable SWCLK and SWDIO on pins 7 and 8. They iare enabled by default:
	SWM_PINENABLE0 |= BIT4; // Disable SWCLK
	SWM_PINENABLE0 |= BIT5; // Disable SWDIO

	// Configure the pins connected to the LCD as outputs
	GPIO_DIR0 |= BIT9; // Used for LCD_RS  Pin 13 of TSSOP20 package.
	GPIO_DIR0 |= BIT8; // Used for LCD_E.  Pin 14 of TSSOP20 package.
	GPIO_DIR0 |= BIT10;  // Used for LCD_D7. Pin 10 of TSSOP20 package. WARNING: NEEDS PULL-UP Resistor to VDD.
	GPIO_DIR0 |= BIT11;  // Used for LCD_D6. Pin 9 of TSSOP20 package. WARNING: NEEDS PULL-UP Resistor to VDD.
	GPIO_DIR0 |= BIT2; // Used for LCD_D5. Pin 8 of TSSOP20 package.
	GPIO_DIR0 |= BIT3; // Used for LCD_D4. Pin 7 of TSSOP20 package.
}

void InitTimer(void)
{
	SCTIMER_CTRL |= BIT2; // halt SCTimer

    // Assign a pin to the timer.
    // Assign GPIO_14 to SCT_OUT0_O
	SWM_PINASSIGN7 &= 0x00ffffff;
	SWM_PINASSIGN7 |= (14 << 24); 
	
	SYSCON_SYSAHBCLKCTRL |= BIT8; // Turn on SCTimer 
	SYSCON_PRESETCTRL |=  BIT8; // Clear the reset SCT control
	
	SCTIMER_CONFIG |= BIT0; // Unified 32 bit counter
	SCTIMER_MATCH0 = SYSTEM_CLK/(DEFAULT_F*2L); // Set delay period 
	SCTIMER_MATCHREL0 = SYSTEM_CLK/(DEFAULT_F*2L);
	SCTIMER_EV0_STATE = BIT0;  // Event 0 pushes us into state 0
	// Event 0 configuration:
	// Event occurs on match of MATCH0, new state is 1	
	SCTIMER_EV0_CTRL =  BIT12 + BIT14 + BIT15;
	// State 1 configuration
	SCTIMER_EV1_STATE = BIT1;  // Event 1 pushes us into state 1
	// Event 1 configuration
	// Event occurs on MATCH0, new state is 0
	SCTIMER_EV1_CTRL =  BIT12 + BIT14;
	// OUT0 is set by event 0
	SCTIMER_OUT0_SET = BIT0;
	// OUT1 is cleared by event 1
	SCTIMER_OUT0_CLR = BIT1;
	// Processing events 0 and 1
	SCTIMER_LIMIT_L = BIT0 + BIT1;
	// Remove halt on SCTimer
	SCTIMER_CTRL &= ~BIT2;		
		
	SCTIMER_EVEN = 0x01; //Interrupt on event 0
	NVIC_ISER0|=BIT9; // Enable SCT interrupts in NVIC
}

void Reload_SCTIMER (unsigned long Dly)
{
	SCTIMER_CTRL |= BIT2; // halt SCTimer
	SCTIMER_MATCH0 = Dly; // Set delay period 
	SCTIMER_MATCHREL0 = Dly;
	SCTIMER_COUNT = 0;
	SCTIMER_CTRL &= ~BIT2;	// Remove halt on SCTimer	
}

void wait_1ms(void)
{
	// For SysTick info check the LPC824 manual page 317 in chapter 20.
	SYST_RVR = (SYSTEM_CLK/1000L) - 1;  // set reload register, counter rolls over from zero, hence -1
	SYST_CVR = 0; // load the SysTick counter
	SYST_CSR = 0x05; // Bit 0: ENABLE, BIT 1: TICKINT, BIT 2:CLKSOURCE
	while((SYST_CSR & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SYST_CSR = 0x00; // Disable Systick counter
}

void delayms(int len)
{
	while(len--) wait_1ms();
}

/* Start ADC calibration */
void ADC_Calibration(void)
{
	unsigned int saved_ADC_CTRL;

	// Follow the instructions from the user manual (21.3.4 Hardware self-calibration)
	
	//To calibrate the ADC follow these steps:
	
	//1. Save the current contents of the ADC CTRL register if different from default.	
	saved_ADC_CTRL=ADC_CTRL;
	// 2. In a single write to the ADC CTRL register, do the following to start the
	//    calibration:
	//    � Set the calibration mode bit CALMODE.
	//    � Write a divider value to the CLKDIV bit field that divides the system
	//      clock to yield an ADC clock of about 500 kHz.
	//    � Clear the LPWR bit.
	ADC_CTRL = BIT30 | ((300/5)-1); // BIT30=CALMODE, BIT10=LPWRMODE, BIT7:0=CLKDIV
	// 3. Poll the CALMODE bit until it is cleared.
	while(ADC_CTRL&BIT30);
	// Before launching a new A/D conversion, restore the contents of the CTRL
	// register or use the default values.
	ADC_CTRL=saved_ADC_CTRL;
}


void InitADC(void)
{
	// Will use pins 1 and 2 of TSSOP-20 package (PIO_23 and PIO_17) for ADC.
	// These correspond to ADC Channel 3 and 9.  Also connect the
	// VREFN pin (pin 17 of TSSOP-20) to GND, and VREFP the
	// pin (pin 17 of TSSOP-20) to VDD (3.3V).
	
	SYSCON_PDRUNCFG &= ~BIT4; // Power up the ADC
	SYSCON_SYSAHBCLKCTRL |= BIT24;// Start the ADC Clocks
	ADC_Calibration();
	ADC_SEQA_CTRL &= ~BIT31; // Ensure SEQA_ENA is disabled before making changes	
	
	ADC_CTRL =1;// Set the ADC Clock divisor
	SWM_PINENABLE0 &= ~BIT16; // Enable the ADC function on PIO_23 (ADC_3, pin 1 of TSSOP20)	
	SWM_PINENABLE0 &= ~BIT22; // Enable the ADC function on PIO_17 (ADC_9, pin 9 of TSSOP20)	
}

// WARNING: in order to use the ADC with other pins, the pins need to be configured in
// the function above.
int ReadADC(int channel)
{
	ADC_SEQA_CTRL &= ~BIT31; // Ensure SEQA_ENA is disabled before making changes
	ADC_SEQA_CTRL &= 0xfffff000; // Deselect all previously selected channels	
	ADC_SEQA_CTRL |= (1<<channel); // Select Channel	
	ADC_SEQA_CTRL |= BIT31 + BIT18; // Set SEQA and Trigger polarity bits
	ADC_SEQA_CTRL |= BIT26; // Start a conversion:
	while( (ADC_SEQA_GDAT & BIT31)==0); // Wait for data valid
	return ( (ADC_SEQA_GDAT >> 4) & 0xfff);
}

// Read ADC at specified channel and return voltage
int ReadJoystick(int channel){
	int j = ReadADC(channel);
	int v = (j*33000)/0xfff;

	return v;
}

void STC_IRQ_Handler(void)
{
	SCTIMER_EVFLAG = 0x01; // Clear interrupt flag
	
	switch (direction){
		// STOP
		// send square wave for 100/800 clock cycles
		case 0: {
			if (count < 100) {
			OUT0 =! OUT0;
			OUT1 =! OUT0;
			}
			else {
				OUT0 = 0;
				OUT1 = 0;
			}
			break;
		}
		// LEFT
		// send square wave for 200/800 clock cycles
		case 1: {
			if (count < 200) {
			OUT0 =! OUT0;
			OUT1 =! OUT0;
			}
			else {
				OUT0 = 0;
				OUT1 = 0;
			}
			break;
		}
		// RIGHT
		// send square wave for 300/800 clock cycles
		case 2: {
			if (count < 300) {
			OUT0 =! OUT0;
			OUT1 =! OUT0;
			}
			else {
				OUT0 = 0;
				OUT1 = 0;
			}
			break;
		}
		// FORWARD
		// send square wave for 400/800 clock cycles
		case 3: {
			if (count < 400) {
			OUT0 =! OUT0;
			OUT1 =! OUT0;
			}
			else {
				OUT0 = 0;
				OUT1 = 0;
			}
			break;
		}
		// BACKWARD
		// send square wave for 500/800 clock cycles
		case 4: {
			if (count < 500) {
			OUT0 =! OUT0;
			OUT1 =! OUT0;
			}
			else {
				OUT0 = 0;
				OUT1 = 0;
			}
			break;
		}
		// TRACK
		// send square wave for 650/800 clock cycles
		case 5: {
			if (count < 650) {
			OUT0 =! OUT0;
			OUT1 =! OUT0;
			}
			else {
				OUT0 = 0;
				OUT1 = 0;
			}
			break;
		}
		// TRACKING
		case 6: {
			// while tracking, controller sends a constant 14910 Hz square wave
			OUT0 =! OUT0;
			OUT1 =! OUT0;
			count = 0;
			break;
		}

		default: {
			OUT0 = 0;
			OUT0 = 0;
			count = 0;
		}
	}

	count++; // increment count after each clock cycle
	if (count == 800) count = 0; // Reset count after 800 clock cycles
}

void main(void)
{
	int x, y;
	
	// Initialization
	ConfigPins();	
	InitTimer();
	initUART(115200);
	InitADC();
	LCD_4BIT();
	enable_interrupts();
	LCDprint("STOP", 1, 1);

	while(1) {
		// TRACKING MODE
		if (direction == 6) {
			// if joystick is pressed, send STOP and enter command mode
			if (TRACK_B == 0){
				direction = 0;
				LCDprint("STOP", 1, 1);
				while (TRACK_B == 0); // wait for release
				delayms(50); // debounce delay
			}
		}

		// COMMAND MODE
		else {
			// if robot is not currently stopped, send STOP
			if (direction != 0){
				direction = 0;
				LCDprint("STOP", 1, 1);
			}
			x = ReadJoystick(9);
			y = ReadJoystick(3);

			// LEFT
			if (y < 3000){
				direction = 1;
				LCDprint("LEFT", 1, 1);
				while (y < 3000) y = ReadJoystick(3);
			}

			// RIGHT
			if (y > 30000){
				direction = 2;
				LCDprint("RIGHT", 1, 1);
				while (y > 30000) y = ReadJoystick(3);
			}

			// FORWARD
			if (x > 30000){
				direction = 3;
				LCDprint("FORWARD", 1, 1);
				while (x > 30000) x = ReadJoystick(9);
			}

			// REVERSE
			if (x < 3000){
				direction = 4;
				LCDprint("BACKWARD", 1, 1);
				while (x < 3000) x = ReadJoystick(9);
			}

			// if joystick is pressed, send TRACK and enter tracking mode
			if (TRACK_B == 0){
				direction = 5;
				LCDprint("TRACKING", 1, 1);
				while (TRACK_B == 0); // wait for release
				direction = 6;
				delayms(50); // debounce delay
			}
		}
	}
}
