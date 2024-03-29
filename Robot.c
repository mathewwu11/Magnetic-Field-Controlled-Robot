// Robot.c
// Authors: James Chan, Lavis Chen, David Hsiao, Weymen Koo, and Mathew Wu

#include <stdio.h>
#include <stdlib.h>
#include <EFM8LB1.h>


volatile unsigned int servo_counter=0;
volatile unsigned char LEFT_BACKWARD=150, LEFT_FORWARD=150, RIGHT_FORWARD=150, RIGHT_BACKWARD=150;
unsigned char overflow_count;

// ~C51~  
#define LEFT_BACKWARD P2_1
#define LEFT_FORWARD P2_2
#define RIGHT_FORWARD P2_3
#define RIGHT_BACKWARD P2_4
#define COMPARE P2_5
#define MODE_LED P2_6

#define SYSCLK 72000000L
#define BAUDRATE 115200L
#define SARCLK 18000000L
#define RELOAD_10us (0x10000L-(SYSCLK/(12L*100000L))) // 10us rate

char _c51_external_startup (void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key
  
	VDM0CN=0x80;       // enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD

	#if (SYSCLK == 48000000L)	
		SFRPAGE = 0x10;
		PFE0CN  = 0x10; // SYSCLK < 50 MHz.
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20; // SYSCLK < 75 MHz.
		SFRPAGE = 0x00;
	#endif
	
	#if (SYSCLK == 12250000L)
		CLKSEL = 0x10;
		CLKSEL = 0x10;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 24500000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 48000000L)	
		// Before setting clock to 48 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		// Before setting clock to 72 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
	#endif
	
	P0MDOUT |= 0x10; // Enable UART0 TX as push-pull output
	XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)                     
	XBR1     = 0X00;
	XBR2     = 0x40; // Enable crossbar and weak pull-ups

	// Configure Uart 0
	#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
		#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
	#endif
	SCON0 = 0x10;
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready
  	
	return 0;
}

void Set_Pin_Output (unsigned char pin)
{
    unsigned char mask;

    mask=(1<<(pin&0x7));
    switch(pin/0x10)
    {
        case 0: P0MDOUT |= mask; break;
        case 1: P1MDOUT |= mask; break;
        case 2: P2MDOUT |= mask; break; 
        case 3: P3MDOUT |= mask; break; 
    }
}

void InitADC (void)
{
	SFRPAGE = 0x00;
	ADEN=0; // Disable ADC
	
	ADC0CN1=
		(0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
        (0x0 << 3) | // 0x0: No shift. 0x1: Shift right 1 bit. 0x2: Shift right 2 bits. 0x3: Shift right 3 bits.		
		(0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32
	
	ADC0CF0=
	    ((SYSCLK/SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz. Fsarclk = (Fadcclk) / (ADSC + 1)
		(0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.
	
	ADC0CF1=
		(0 << 7)   | // 0: Disable low power mode. 1: Enable low power mode.
		(0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)
	
	ADC0CN0 =
		(0x0 << 7) | // ADEN. 0: Disable ADC0. 1: Enable ADC0.
		(0x0 << 6) | // IPOEN. 0: Keep ADC powered on when ADEN is 1. 1: Power down when ADC is idle.
		(0x0 << 5) | // ADINT. Set by hardware upon completion of a data conversion. Must be cleared by firmware.
		(0x0 << 4) | // ADBUSY. Writing 1 to this bit initiates an ADC conversion when ADCM = 000. This bit should not be polled to indicate when a conversion is complete. Instead, the ADINT bit should be used when polling for conversion completion.
		(0x0 << 3) | // ADWINT. Set by hardware when the contents of ADC0H:ADC0L fall within the window specified by ADC0GTH:ADC0GTL and ADC0LTH:ADC0LTL. Can trigger an interrupt. Must be cleared by firmware.
		(0x0 << 2) | // ADGN (Gain Control). 0x0: PGA gain=1. 0x1: PGA gain=0.75. 0x2: PGA gain=0.5. 0x3: PGA gain=0.25.
		(0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.

	ADC0CF2= 
		(0x0 << 7) | // GNDSL. 0: reference is the GND pin. 1: reference is the AGND pin.
		(0x1 << 5) | // REFSL. 0x0: VREF pin (external or on-chip). 0x1: VDD pin. 0x2: 1.8V. 0x3: internal voltage reference.
		(0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)
	
	ADC0CN2 =
		(0x0 << 7) | // PACEN. 0x0: The ADC accumulator is over-written.  0x1: The ADC accumulator adds to results.
		(0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3

	ADEN=1; // Enable ADC
}

void Timer5_ISR (void) interrupt INTERRUPT_TIMER5
{
	SFRPAGE=0x10;
	TF5H = 0; // Clear Timer5 interrupt flag
	TMR5RL=RELOAD_10us;
	servo_counter++;
	if(servo_counter==2000)
	{
		servo_counter=0;
		LEFT_BACKWARD=1;
		LEFT_FORWARD=1;
		RIGHT_FORWARD=1;
		RIGHT_BACKWARD=1;
	}
	if(LEFT_BACKWARD==servo_counter)
	{
		LEFT_BACKWARD=0;
	}
	if(LEFT_FORWARD==servo_counter)
	{
		LEFT_FORWARD=0;
	}
	if(RIGHT_FORWARD==servo_counter)
	{
		RIGHT_FORWARD=0;
	}
	if(RIGHT_BACKWARD==servo_counter)
	{
		RIGHT_BACKWARD=0;
	}
}

// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
	CKCON0|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN0 & 0x80));  // Wait for overflow
		TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) Timer3us(250);
}

#define VDD 3.3035 // The measured value of VDD in volts

void TIMER0_Init(void)
{
	TMOD&=0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
	TMOD|=0b_0000_0001; // Timer/Counter 0 used as a 16-bit timer
	TR0=0; // Stop Timer/Counter 0
}

void InitPinADC (unsigned char portno, unsigned char pin_num)
{
	unsigned char mask;
	
	mask=1<<pin_num;

	SFRPAGE = 0x20;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
	SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

float Volts_at_Pin(unsigned char pin)
{
	 return ((ADC_at_Pin(pin)*VDD)/16383.0);
}

void LeftTurn(){
	LEFT_BACKWARD=1;
	LEFT_FORWARD=0;
	RIGHT_FORWARD=1;
	RIGHT_BACKWARD=0;
}

void RightTurn(){
	LEFT_BACKWARD=0;
	LEFT_FORWARD=1;
	RIGHT_FORWARD=0;
	RIGHT_BACKWARD=1;
}

void Forward(){
	LEFT_BACKWARD=0;
	LEFT_FORWARD=1;
	RIGHT_FORWARD=1;
	RIGHT_BACKWARD=0;
}

void Backward(){
	LEFT_BACKWARD=1;
	LEFT_FORWARD=0;
	RIGHT_FORWARD=0;
	RIGHT_BACKWARD=1;
}

void Stop(){
	LEFT_BACKWARD=0;
	LEFT_FORWARD=0;
	RIGHT_FORWARD=0;
	RIGHT_BACKWARD=0;
}

void main (void)
{
	float v[2];
	float vdiff;
	float diff_per;
	float period;
	int mode = 0;
	
	Set_Pin_Output(0x26); // set P2_6 (MODE_LED) to digital output
	MODE_LED = 0; 

    waitms(500); // Give PuTTy a chance to start before sending
	printf("\x1b[2J"); // Clear screen using ANSI escape sequence.
	
	printf (
	        "File: %s\n"
	        "Compiled: %s, %s\n\n",
	        __FILE__, __DATE__, __TIME__);
	
	InitPinADC(1, 6); // Configure P1.6 as analog input
	InitPinADC(1, 7); // Configure P1.7 as analog input
    InitADC();
	while(1)
	{
		if(mode == 1)
		{
			MODE_LED = 1;    
			while (1)
			{
				if (mode == 0)
				{
					break;
				}
				// Read 14-bit value from the pins configured as analog inputs
				v[0] = Volts_at_Pin(QFP32_MUX_P1_6); // right inductor
				v[1] = Volts_at_Pin(QFP32_MUX_P1_7); // left inductor
				vdiff = v[0]-v[1];
				diff_per=(vdiff/((v[0]+v[1])/2))*100;
				
				Stop();

				if ((v[1]-v[0])>0.5) {
					RightTurn();
				}
				else if ((v[0]-v[1]>0.5)) {
					LeftTurn();
				}
				else if((v[0]+v[1])/2>2.3) {
					Backward();
				}
				else if((v[0]+v[1])/2<2.1) {
					Forward();
				}
				
				waitms(50);
				
				if(COMPARE == 0){
					Stop();
					mode = 0;
				}
				
			}
		}
		else 
		{
			MODE_LED = 0;
			while(1)
			{
				
				do 
				{
					// Reset the counter
					TL0=0; 
					TH0=0;
					TF0=0;
					overflow_count=0;
				
					while(COMPARE!=0); // Wait for the signal to be zero
					while(COMPARE!=1); // Wait for the signal to be one
					TR0=1; // Start the timer
					while(COMPARE!=0) // Wait for the signal to be zero
					{
						if(TF0==1) // Did the 16-bit timer overflow?
						{
							TF0=0;
							overflow_count++;
						}
					}
					while(COMPARE!=1) // Wait for the signal to be one
					{
						if(TF0==1) // Did the 16-bit timer overflow?
						{
							TF0=0;
							overflow_count++;
						}
					}
					TR0=0; // Stop timer 0, the 24-bit number [overflow_count-TH0-TL0] has the period!
					period=(overflow_count*65536.0+TH0*256.0+TL0)*(12.0/SYSCLK);
				} while ((period * 1000) < 10);
				
				if( period * 1000.0 < 60.0)
				{
					Stop();
				}
				if(period * 1000.0 > 60.0 && period * 1000.0 < 80.0)
				{
					LeftTurn();
				}
				if(period * 1000.0 > 80.0 && period * 1000.0 < 110.0)
				{
					RightTurn();
				}
				if(period * 1000.0 > 110.0 && period * 1000.0 < 135.0)
				{
					Forward();
				}
				if(period * 1000.0 > 135.0 && period * 1000.0 < 165.0)
				{
					Backward();
				}
				if(period * 1000.0 > 165.0)
				{
					Stop();
					mode = 1;
					break;
				}
				
			}  
		}
	}
}	

