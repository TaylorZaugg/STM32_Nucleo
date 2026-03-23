#include "stm32l552xx.h"
#include "stm32l5xx_it.h"

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &       1) //Checks the bit number <idx> -- 0 means clear; !0 means set.

// Helping functions
void setClks();
void RLEDinit();
void RLEDtoggle();

int main(){
	setClks();

	//	1- Enable GPIO as input
	RLEDinit();

	//  2- Enable Timer Update Interrupt
	//Frequency Generator
	#define PERIOD 100 //msec  -- LED is on for 0.1sec and off for 0.1sec

	bitset(RCC->APB2ENR, 11);    // enable TIM1 clock
	TIM1->PSC = 16000 - 1;       // Divided 16MHz source clk by 16000, for 1ms tick
	TIM1->ARR = PERIOD - 1;      // Count 1ms PERIOD times
	TIM1->CNT = 0;               // Clear counter
	TIM1->DIER |= 1;             // Set Update Interrupt Enable
	TIM1->CR1 = 1;               // Enable TIM1

	//	3- Setup Interrupt Priority
	NVIC_SetPriority(TIM1_UP_IRQn, 0); // 0 is higher than 1 (3 bit priority)

	//	5- Enable IRQ in NVIC
	NVIC_EnableIRQ(TIM1_UP_IRQn);

	//	8- Enable Global Interrupt Flag
	__enable_irq();   // No need since it is enabled by default

	while(1); // nothing inside the while loop
}
//	9- Define Interrupt Service Routine (ISR)
// Interrupt Service Routine to be called when TIM1_UP_IRQn is raised
void TIM1_UP_IRQHandler(){
	RLEDtoggle();
	TIM1->SR &= ~1; // Clear flag
}


void setClks(){
	RCC->APB1ENR1 |=1<<28;   // Enable the power interface clock by setting the PWREN bits
	RCC->APB1ENR2 |=0x1;     // Enable LPUART1EN clock
	RCC->CCIPR1   |=0x800;   // 01 for HSI16 to be used for LPUART1
	RCC->CCIPR1   &= ~(0x400);
	RCC->CFGR     |=0x1;     // Use HSI16 as SYSCLK
	RCC->CR       |=0x161;   // MSI clock enable; MSI=4 MHz; HSI16 clock enable
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
