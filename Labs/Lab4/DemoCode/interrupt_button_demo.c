#include "stm32l552xx.h"
//#include "stm32l5xx_it.h"

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &       1) //Checks the bit number <idx> -- 0 means clear; !0 means set.

// Helping functions
void setClks();
void RLEDinit();
void RLEDtoggle();
void BTNinit();

int main(){
	setClks();
	RLEDinit();

	//	1- Enable GPIO as input
	BTNinit();

	//	2- Enable Clock to SYSCFG
	// To use EXTI you need to enable SYSCFG
	RCC->APB2ENR   |= 1;  // Enable Clock to SYSCFG & EXTI

	//	3- Select Input Using EXTI-->CRx
	EXTI->EXTICR[3] = (0x2)<<8;  // Select PC13

    //  REG        |  31-24   | 23-16  |  15-8   |    7-0    |
	//-------------+----------+--------+---------+-----------+
	// EXTICR[0]   |  GPIOx3  | GPIOx2 |  GPIOx1 |  GPIOx0   |
	// EXTICR[1]   |  GPIOx7  | GPIOx6 |  GPIOx5 |  GPIOx4   |
	// EXTICR[2]   |  GPIOx11 | GPIOx10|  GPIOx9 |  GPIOx8   |
	// EXTICR[3]   |  GPIOx15 | GPIOx14|  GPIOx13|  GPIOx12  |
	//-------------+----------+--------+---------+-----------+
    //  MUXSEL:   0 1 2 3 4 5 6 7
	//  PORT:     A B C D E F G H


	//	4- Select Trigger type (Failing or Raising Edge)
	EXTI->FTSR1    |= 1<<13;     // Trigger on falling edge of PC13
	                             // Use RTSR1 register for rising edge

	//	5- Disable Interrupt Mask
	EXTI->IMR1     |= 1<<13;     // Interrupt mask disable for PC13

	//	6- Setup Interrupt Priority
	NVIC_SetPriority(EXTI13_IRQn, 0); // 0 is higher than 1 (3 bit priority)

	//	7- Enable IRQ in NVIC
	NVIC_EnableIRQ(EXTI13_IRQn);

	//	8- Enable Global Interrupt Flag
	__enable_irq();   // No need since it is enabled by default

	while(1);
}
//	9- Define Interrupt Service Routine (ISR)
// Interrupt Service Routine to be called when EXTI13_IRQn is raised
void EXTI13_IRQHandler(){
	RLEDtoggle();
    // 10- Clear pending Interrupt
	EXTI->FPR1 |= 1<<13; // Cleared by writing 1 to it!
	                     // Use RPR1 when trigger by Rising edge

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

void BTNinit(){
	RCC->AHB2ENR |= (1<<2); // Enable GPIOC
	// Set up the mode for button at C13
	bitclear(GPIOC->MODER, 26); // Clear bit 26 and 27
	bitclear(GPIOC->MODER, 27);
}
