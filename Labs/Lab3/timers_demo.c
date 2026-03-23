#include "stm32l552xx.h"
#include <stdio.h>

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &  1) //Checks the bit number <idx> -- 0 means clear; !0 means set.


// Helping functions (Already coded)
void setClks();
void LPUART1init(void);
void LPUART1write(int c);
int  LPUART1read(void);
void myprint(char msg[]);
void RLEDinit();
void RLEDtoggle();


//////////////////////////////////
/*         Main Function        */
//////////////////////////////////
int main(void) {
    setClks();     // Clocks are ready to use, 16Mhz for system
    LPUART1init(); // UART is ready to use
    RLEDinit();    // RED LED is ready to use


    // Hint: un-comment TEST CASEs one by one.
    // Code the the use function and test it.
    // Use debug mode


//    //CASE 1: SysTick Timer -- 24-bit countdown timer embedded inside Cortex M33 ARM Processor
//    while(1){
//        RLEDtoggle();
//
//        SysTick->LOAD=16000000-1;  // fits inside 24-bit '0xf42400'
//        SysTick->VAL = 0;          // clear current value register
//    	  SysTick->CTRL = 0b101;     // Enable the timer
//    	                             // CLKSOURCE= 1 HCLK (no prescalar)
//    	  while(bitcheck(SysTick->CTRL,16) == 0);
//    }


//    //CASE 2: Timer 4        16-bit General Purpose Timer
//    bitset(RCC->APB1ENR1, 2);      // enable TIM4 clock
//    while(1){
//        RLEDtoggle();
//
//    	//delay for 500000usec or 0.5sec
//    	TIM4->PSC = 160-1;           // divided by 16, for 10us tick
//    	TIM4->ARR = 50000 - 1;       // divided by val -- fits in 16-bits '0xc350'
//    	TIM4->CNT = 0;               // clear counter
//    	TIM4->CR1 = 1;               // enable TIM4
//    	while(bitcheck(TIM4->SR, 0)==0);   //Update interrupt flag
//    	bitclear(TIM4->SR, 0);
//    }


//    //CASE 3: Frequency Generator
//    #define PERIOD 2000 //msec  -- LED is on for 2sec and off for 2sec
//
//    // PA9 is connected TIM1_CH2  (AF1)
// 	RCC->AHB2ENR  |= 1; 		    // Enable GPIOA Clock
// 	bitclear(GPIOA->MODER, 9*2);    // AF is 0b10
// 	bitset(  GPIOA->MODER, 9*2+1);
// 	GPIOA->AFR[1] &= ~(0xf << 4); 	// Clear the 4 bits for PA9 (bit 4, 5, 6 and 7)
// 	bitset(GPIOA->AFR[1], 4); 	    // Set the AF1 to connect PA9 to TIM1_CH2
//
// 	bitset(RCC->APB2ENR, 11);    // enable TIM1 clock
// 	TIM1->PSC = 16000 - 1;       // Divided 16MHz source clk by 16000, for 1ms tick
// 	TIM1->ARR = PERIOD - 1;      // Count 1ms PERIOD times (2000ms or 2 secs)
// 	TIM1->CCMR1 = 0x3000;        // Set output to toggle on match (Output Compare Mode)
// 	TIM1->CCR2 = 1;				 // Output will toggle when CNT==CCR2
// 	bitset(TIM1->BDTR, 15);      // Main output enable (Needed for Advanced Timers only?)
// 	TIM1->CCER |= 1 << 4;        // Enable CH2 compare mode
// 	TIM1->CNT = 0;               // Clear counter
// 	TIM1->CR1 = 1;               // Enable TIM1
//    while (1);                   // Loop forever, LED is AUTOMATICALLY toggled!





//    //CASE 4: Configure LPTIM1 to use external input as counter clock source
//    // LPTIM_IN1 at PB5
//	RCC->AHB2ENR  |= 2; 		    // Enable GPIOB Clock
//	bitclear(GPIOB->MODER, 5*2);    // AF is 0b10
//	bitset(  GPIOB->MODER, 5*2+1);
//	GPIOB->AFR[0] &= ~(0xf << 20);  // Clear the 4 bits for PB5 (bit 20, 21, 22 and 23)
//	bitset(GPIOB->AFR[0], 20); 	    // Set the AF1 to connect PB5 to LPTIM_IN1
//
//    RCC->APB1ENR1 |= 1 << 31;       // Enable LPTIM1 clock */
//    LPTIM1->CFGR   = 1|(1 << 23);   // Use external clock source,
//                                    // Increment counter using external input
//    LPTIM1->CNT = 0;                // Reset count to 0
//    LPTIM1->CR  = 1;                // Enable timer
//    LPTIM1->CR |= 0b100;            // Timer in continuous mode – LPTIM must be enabled
//	LPTIM1->ARR = 0xFFFF;		    // Count up to max
//	                                // ARR must only be modified when the LPTIM is enabled
//
//	while (1)
//    {
//       // In a debug mode monitor:
//	   // This loop will be sending data to PC using LPUART.
//	   // Connect a wire from TX pin on CN6 connector to PB5.
//	   // This will cause the timer to count the edges of the LPUART TX signal.
//    	char buff[100];
//    	sprintf(buff, "$%d;", (int) LPTIM1->CNT); // Format a string as "$value;"
//    	myprint(buff);                            // Send it using LPUART1
//    	for(int i=0; i<10000;i++);                // Some delay
//   }



//   //CASE 5: PWM
//	// PC07 (Green LED) is connected to TIM3_CH2
//	RCC->AHB2ENR  |= 4 ; 		    // Enable GPIOC Clock
//	GPIOC->MODER  &= ~(0b11 << 14); // Clear the two bits for PB7
//	GPIOC->MODER  |=  (0x2) << 14; 	// Set the mode to AF (10--> 0x2)
//	GPIOC->AFR[0] &= ~(0xf << 28); 	// Clear the 4 bits for PB7
//	GPIOC->AFR[0] |=  (0x2) << 28; 	// Set the 4 bits to (AF2) to connect to TIM3_CH2
//
//    // configure TIM3
//	RCC->APB1ENR1 |= 0b10;       	// Enable TIM3 clock
//	TIM3->CR1 &= ~1;           		// disable TIM3 in case it was on
//
//	TIM3->PSC = 16000 - 1;    		// divided by 16000, for 1ms tick
//	TIM3->ARR = 1000 - 1;     		// divided by 1000  so the counter counts 0 to 999
//	TIM3->CCMR1 = 0x6000;       	// Set output mode to pwm 1 where output is 1 if CNT < CCR, 0 otherwise
//	TIM3->CCR2 = 100;				// Compare to CNT. 1 if CNT < 100, 0 if CNT > 100
//	TIM3->CCER |= 1 << 4;      		// Enable CH2 compare mode
//	TIM3->CNT = 0;            		// Clear counter
//	TIM3->CR1 = 1;            		// Enable TIM3
//
//    while(1){
//    	char buff[100];
//    	sprintf(buff, "$%d;", (int) TIM3->CNT); // Format a string as "$value;"
//    	myprint(buff);                          // Send it using LPUART1
//    	for(int i=0; i<10000;i++);              // Some delay
//}



}


/////////////////////
// Helping Functions
/////////////////////
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

void setClks(){
RCC->APB1ENR1 |=1<<28;   // Enable the power interface clock by setting the PWREN bits
RCC->APB1ENR2 |=0x1;     // Enable LPUART1EN clock
RCC->CCIPR1   |=0x800;   // 01 for HSI16 to be used for LPUART1
RCC->CCIPR1   &= ~(0x400);
RCC->CFGR     |=0x1;     // Use HSI16 as SYSCLK
RCC->CR       |=0x161;   // MSI clock enable; MSI=4 MHz; HSI16 clock enable used for LPUART1
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
