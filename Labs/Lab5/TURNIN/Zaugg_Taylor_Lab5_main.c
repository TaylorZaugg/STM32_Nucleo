/*
 * Taylor Zaugg
 * ECE 433
 * Lab 5 ADC
 * Date: 2025-02-06
 *
 * This program uses GPIOC pins 0 and 1 as analog inputs feedings into ADC1
 * The program sends LPUART Temperature output based on PC00 or ADC_IN1
 * There is a potentiometer connection on PC1 (ADC_IN2). The USER pushbutton
 * will set the speed that the temperature sensor is read and sent.
 *
 * The USER push button is used to send an interrupt that sends an injected
 * conversion to ADC1. This reads the current potentiometer value and sets
 * TIM1-ARR value between 1ms and 2000ms.
 *
 * Notes/Concerns/Issues:
 * The linear thermistor IC MCP9701's data sheet provides the sensor transfer
 * function used to calculate the temperature. However, initially I used the
 * coefficients from the 9700 sensor. This reading seemed more accurate to my
 * environment. In switching to the "correct" coefficients the output seems
 * to be off by about 10 degrees F. This may be due to other factors, such
 * as calibration, so I have kept the specified coefficients for the 9701
 * sensor.
 *
 * Pushbutton and potentiometer reading issues.
 * I spent most of my time trying to understand how to configure and get the
 * injected mode to function properly. There is still some fine tuning needed.
 * After pressing the button the speed seems to change almost randomly.
 * I say almost because there is usually a common speed based on the
 * potentiometer position. I don't like how inconsistent it feels.
 * Also, based on the actual voltage values output by the
 * potentiometer the max and minimum speeds only are reached during the
 * random overlapping pushbutton events.
 * Maybe if I used a sequence of injected values and averaged out the result
 * it might be more consistent. Using all the injected registers instead of
 * just 1. Or even using a second timer and collecting more data before
 * setting the speed.
 *
 * New realization in writing this and viewing the serial plotter graph
 * I think their may be some interference or a misconfiguration because when
 * the button is pressed the output speed seems to change based on temperature.
 * I have no idea what is happening or why this is the case but it makes
 * sense why I was getting such random results. I think by having the power
 * and grounds shared between the temp sensor and potentiometer there is a
 * lot of interference causing instability when the interrupt is ran.
 */

#include "stm32l552xx.h"
#include "stdio.h"

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) & 1      ) //Checks the bit number <idx> -- 0 means clear; !0 means set.

// Helping functions
void setClks();
void LPUART1init(void);
void LPUART1write(int c);
int  LPUART1read(void);
void myprint(char msg[]);
void InitGPIOC();
void BTNinit();
void InitTIM1();

void RLEDinit();
void RLEDtoggle();

void enGpiogPwr();

#define MIN_PERIOD 1      // 1ms
#define MAX_PERIOD 2000   // 2000 ms
#define PERIOD 250 // Default starting speed

int main (void) {

	// Configuration Functions ---------------------------
    setClks();     // Clocks are ready to use, 16Mhz for system
    enGpiogPwr();
    LPUART1init();
    InitTIM1();

    BTNinit(); //GPIO for USER button

    InitGPIOC(); // for both potentiometer and temp sensor, PC00 and PC01

    // ADC Configuration -------------------------------------

	// Enable ADC Clock
	bitset(RCC->AHB2ENR, 13);  // Enable ADC clock
	RCC->CCIPR1 |=0x3<<28;     // Route SYSCLK (HCLK) to ADC

    // Turn on ADC Voltage Regulator
	bitclear(ADC1->CR, 29);  // Get out of deep power down mode
    bitset(ADC1->CR, 28);

    // External Trigger Enable and Polarity; EXTEN= 0b01 for rising edge
    bitset(ADC1->CFGR,   10);
    bitclear(ADC1->CFGR, 11);

    //External trigger (EXT0) is connected to TIM1_CH1; EXTSEL=0000
    bitclear(ADC1->CFGR, 6); // 0b0000
    bitclear(ADC1->CFGR, 7);
    bitclear(ADC1->CFGR, 8);
    bitclear(ADC1->CFGR, 9);

    // Wait for the voltage regulator to stabilize
	delayMs(10);

    //Set regular mode for temp sensor or PC00
    ADC1->SQR1 = (1<<6)|(0); 	       // L=0 (one channel to read), SQ1=IN1 which is connected to PC0 (Called ADC1_IN1)

    // ADC INJECTED CONFIGURATION --------------------------------------------

    // Clear JL bits, so only 1 conversion happens
    bitclear(ADC1->JSQR, 0);
    bitclear(ADC1->JSQR, 1);

    // Set JSQR -> JEXTSEL to channel 2, bits 2-5
    bitclear(ADC1->JSQR, 2); //0b0010
    bitset(ADC1->JSQR, 3);
    bitclear(ADC1->JSQR, 4);
    bitclear(ADC1->JSQR, 5);

    bitset(ADC1->CFGR,31); //Set JQDIS to 1, injected queue disabled

    //JSQR -> JEXTEN clear, Hardware trigger disable, software only IF JQDIS =1
    bitclear(ADC1->JSQR, 6);
    bitclear(ADC1->JSQR, 7);

    //--------------------------------------------------------------

    // Enable Interrupt so that the Timer trigger the ADC to do  conversion
    // Once the conversion is done, the EOC flag is raised and the IRQ handler for ADC is called.
	NVIC_SetPriority(ADC1_2_IRQn, 0); //Priority 1 < 0, so push button interrupt goes through easy
	NVIC_EnableIRQ(ADC1_2_IRQn);
    ADC1->IER  |= 1<<2;                // Enable EOC Interrupt
    ADC1->CR   |= 1;            	   // Enable ADC
    bitset(ADC1->CFGR, 12);  // OVRMOD: Disable overrun mode (ADC keeps going even if user does not read)

	// Wait until ADC is Ready (ADRDY)
	while(bitcheck(ADC1->ISR, 0)==0);

    bitset(ADC1->CR, 2); // Start Conversion (won't actally do the conversion! It will wait for external trigger instead)

    // Enable TIM1
    TIM1->CR1 = 1;

    while (1){
    }

}

// External interrupt from TIM1, read temperature sensor and output readable data
void ADC1_2_IRQHandler(){
    uint16_t adc_val;
    char  txt [20];
    // Nested calls of this IRQ will be nasty! I will disable it.
	NVIC_DisableIRQ(ADC1_2_IRQn);
	// read conversion result  (Reading DR clear EOC flag)
	adc_val = (ADC1->DR)&0xfff;
	// VREF=3.3V, step size is 3.3V/(2^12)=3.3/4096
	float voltage = adc_val*(3.3/4096);
	/*
	 * Vout = Tc * Ta +V0C
	 * Ta = Ambient Temperature
	 * Vout = Sensor output voltage
	 * V0C = Sensor output Voltage at 0C
	 * Tc = Temperature Coefficient
	 *
	 * Output V at 0C for 9701 = 400mV
	 * Tc = 19.5 mV/C
	 *
	 * Vout = 0.0195 V/C * Ta + 0.4 V
	 *
	 * Ta = (Vout - 0.4 V) / 0.0195 V/C
	 */
	float tempC = (voltage - 0.4) / 0.0195; // Calculate temperature in Celsius
	float tempF = (tempC * 9 / 5) + 32;     // Convert Celsius to Fahrenheit


	// send output through LPUART via myprint
	sprintf(txt, "$%.02f;",  tempF);
	myprint(txt);
	TIM1->SR &= ~1; // Clear UIF Bit
	NVIC_EnableIRQ(ADC1_2_IRQn);
}

/*
 * Pushbutton interrupt handle.
 */
void EXTI13_IRQHandler(){
	NVIC_DisableIRQ(ADC1_2_IRQn);

	bitset(ADC1->CR,3); //Turn on injected conversion
    while(!bitcheck(ADC1->ISR, 5));//wait for JEOC flag for conversion

    uint16_t adc_val;
	adc_val = (ADC1->JDR1)&0xfff;//Read the injected output register

	uint16_t new_arr;
	new_arr = (adc_val * MAX_PERIOD)/4096 ; //scale the ADC output from 0 to 2000

	//Handle weird edge cases from noise or something
	if(new_arr < MIN_PERIOD){
		new_arr = MIN_PERIOD;
	}
	if(new_arr > MAX_PERIOD){
		new_arr = MAX_PERIOD;
	}

    TIM1->ARR = new_arr-1;  // Set new timer period
	TIM1->CNT = 0;        // Clear counter

	NVIC_EnableIRQ(ADC1_2_IRQn);

    // Clear pending Interrupt
	EXTI->RPR1 |= 1<<13; // Cleared by writing 1 to it!
	                    // Use RPR1 when trigger by Rising edge
						// FPR1 for falling

}

/////////////////////
// Helping Functions
/////////////////////

// TIMER1 SETUP
void InitTIM1(){
	bitset(RCC->APB2ENR, 11);    // enable TIM1 clock
	TIM1->PSC = 16000-1;				//1 MHz clock
	TIM1->ARR = PERIOD - 1;      // Count PERIOD-1 times
	TIM1->CCMR1 = 0x30;          // Set output to toggle on match (Output Compare Mode)
	TIM1->CCR1 = 1;				 // Output will toggle when CNT==CCR1
	bitset(TIM1->BDTR, 15);      // Main output enable
	TIM1->CCER |= 1;             // Enable CH1 compare mode
	TIM1->CNT = 0;               // Clear counter
}

void InitGPIOC(){

	bitset(RCC->AHB2ENR, 2);	// Enable Clock to GPIOC

	//These pins should be set to 00 from reset, but good to ensure
	//Set GPIO Port C, Pin 0 set 11 Analog mode
	bitset(GPIOC->MODER, 0);
	bitset(GPIOC->MODER, 1);

	//GPIOC Pin 1, Set analog mode (11)
	bitset(GPIOC->MODER, 2);
	bitset(GPIOC->MODER,3);
}

void BTNinit(){
	//RCC->AHB2ENR |= (1<<2); // Enable GPIOC
	// Set up the mode for button at C13
	bitclear(GPIOC->MODER, 26); // Clear bit 26 and 27
	bitclear(GPIOC->MODER, 27);

	RCC->APB2ENR   |= 1;  // Enable Clock to SYSCFG & EXTI

	bitset(EXTI->EXTICR[3], 9);  // Select PC13
	bitset(EXTI->RTSR1,13);     // Trigger on rising edge of PC13
	bitset(EXTI->IMR1,13);     // Interrupt mask disable for PC13

	NVIC_SetPriority(EXTI13_IRQn, 0); // 0 is higher than 1 (3 bit priority)

	NVIC_EnableIRQ(EXTI13_IRQn);

}

void setClks(){
	RCC->APB1ENR1 |=1<<28;   // Enable the power interface clock by setting the PWREN bits
	RCC->APB1ENR2 |=0x1;     // Enable LPUART1EN clock
	RCC->CCIPR1   |=0x800;   // 01 for HSI16 to be used for LPUART1
	RCC->CCIPR1   &= ~(0x400);
	RCC->CFGR     |=0x1;     // Use HSI16 as SYSCLK
	RCC->CR       |=0x161;   // MSI clock enable; MSI=4 MHz; HSI16 clock enable
	RCC->AHB2ENR  |= 0b100;  // Enable GPIOC
}

void LPUART1init(){
PWR->CR2      |=0x200;   // Enable VDDIO2 Independent I/Os supply
                        // Basically power up PORTG

// LPUART1 TX is connected to Port G pin 7, RX is connected to PG8
// GPIOG is connected to the AHB2 bus.
RCC->AHB2ENR |= (1<<6);   // Enable the clock of the GPIOG

// MCU LPUART1 TX is connected the MCU pin PG7
    // PG7 must be set to use AF (Alternate Function).
    // Note that you need to set AF to 8 to connect the LPUART1 TX to the GPIOG.7
GPIOG->MODER  &= ~(0x3<<(2*7)); // Clear the two bits for PG7
GPIOG->MODER  |=  (0x2<<(2*7)); // Set the mode to AF (10--> 0x2)
// Set the AF=8
GPIOG->AFR[0] &= ~(0xF<<(4*7)); // Clear the 4 bits for PG7
    GPIOG->AFR[0] |=  (0x8<<(4*7)); // Set the 4 bits to (8)

// MCU LPUART1 RX can be connected the MCU pin PG8
    // PA3 must be set to use AF (Alternate Function).
    // Note that you need to set AF to 8 to connect the LPUART1 RX to the GPIOG.8
GPIOG->MODER  &= ~(0x3<<(2*8)); // Clear the two bits for PG8
GPIOG->MODER  |=  (0x2<<(2*8)); // Set the mode to AF (10--> 0x2)

GPIOG->AFR[1] &= ~(0xF<<(4*0)); // Clear the 4 bits for PG8
    GPIOG->AFR[1] |=  (0x8<<(4*0)); // Set the 4 bits to (7)

// Enable the clock for the LPUART1
// LPUART1 is connected to the APB1 (Advanced Peripheral 1) bus.
    // LPUART1 enabled by setting bit 0

// LPUART1 CONFIGURATION //
    // We need to setup the baud rate to 115,200bps, 8 bit, 1 stop bit, no parity, and no hw flow control
LPUART1-> PRESC = 0;    //MSI16 going to the UART can be divided. 0000: input clock not divided

// Buadrate = (256 X LPUARTtck_pres)/LPUART_BRR
// LPUART_BRR = 256 * 16MHz / 115200=  35,555.5  ==> 0x8AE3
LPUART1->BRR = 0x8AE3;  //  (16000000/115200)<<8

// LPUART1 input clock is the HSI (high speed internal clock) which is 16MHz.
LPUART1->CR1  = 0x0;  // clear all settings
LPUART1->CR1 |= 1<<3; // Enable Transmitter
LPUART1->CR1 |= 1<<2; // Enable Receiver

// 00: 1 stop bit
LPUART1->CR2 = 0x0000;    // 1 stop bit and all other features left to default (0)
LPUART1->CR3 = 0x0000;    // no flow control and all other features left to default (0)

// Last thing is to enable the LPUART1 module (remember that we set the clock, configure GPIO, configure LPUART1)
LPUART1->CR1 |= 1; // Enable LPUART1

}

void myprint (char msg[]){
    uint8_t idx=0;
    while(msg[idx]!='\0')
    {
    	LPUART1write(msg[idx++]);
    }
}

/* Write a character to LPUART1 */
void LPUART1write (int ch) {
    while (!(LPUART1->ISR & 0x0080)); // wait until Tx buffer empty
    LPUART1->TDR = (ch & 0xFF);
}

/* Read a character from LPUART1 */
int LPUART1read(void) {
    while (!(LPUART1->ISR & 0x0020)) {}   // wait until char arrives
    return LPUART1->RDR;
}

void delayMs(int n) {
    int i;

    /* Configure SysTick */
    SysTick->LOAD = 16000;  /* reload with number of clocks per millisecond */
    SysTick->VAL = 0;       /* clear current value register */
    SysTick->CTRL = 0x5;    /* Enable the timer */

    for(i = 0; i < n; i++) {
        while((SysTick->CTRL & 0x10000) == 0) /* wait until the COUNTFLAG is set */
            { }
    }
    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0) */
}

void enGpiogPwr(){
	bitset(PWR->CR2, 9);
}

void RLEDinit(){
// Enable clock going to GPIOA
RCC->AHB2ENR|=1;

// Set up the mode
GPIOA->MODER |= 1<<18; // setting bit 18
GPIOA->MODER &= ~(1<<19);
}
void RLEDtoggle(){
GPIOA->ODR ^= 1<<9;
}
