/* Taylor Zaugg
 * ECE 433
 * Lab 4 Interrupts
 * Date: 2025-02-27
 *
 * This program uses GPIOE pins 0-9 to toggle LEDs from a 10-segment bar graph LED based on LPUART input interrupts
 *
 * LEDs 0,3,4,5,6,7,8,9 all toggle when the associated number is sent via LPUART
 *
 * LED 1 and 2 are connected to timers. Such that LED 1 blinks 1 timer per second.
 * and LED 2 blinks 2 times per second after timers are turned on by by LPUART
 *
 * When the user pushbutton is pressed my name will be transmitted to the computer via LPUART one char at a time
 * until the termination character is reached, '\0'.
 *
 * Notes/Concerns/Issues:
 * First version of the EXTI13_IRQHandler didn't have the line "while (!(LPUART1->ISR & (1 << 7))"
 * Without it the code worked in debug mode. However, in real-time it would run to fast and
 * exit before the transmit register could be read. I'm not a big fan of the double while loop it now uses
 * but it does work. It waits for the transmit register to be empty before sending the next char.
 *
 * Initially when toggling the timers for LED 1 and LED 2, if the toggle request is send while the LEDs are ON
 * they will stay ON. If it is send when the LEDs are OFF they will stay OFF. This required additional logic
 * to ensure the LEDs stay OFF when the timers toggle OFF. Now when the timers are toggled ON or OFF the
 * LED bits will be cleared. Since the timer turns the LEDs back on again anyway this is fine, but I don't think it's
 * very elegant.
 */

#include "stm32l552xx.h"
#include "stm32l5xx_it.h"

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &       1) //Checks the bit number <idx> -- 0 means clear; !0 means set.

// Helping functions
void setClks();
void enGpiogPwr();
void initGpiog();
void initGpioe();
void initLPUART1();
void BTNinit();
void initTimers();

int main(){
	setClks();

	//Enable PWR going to PORT G
	enGpiogPwr();

	//Configure GPIOG
	initGpiog();

	//configure GPIOE
	initGpioe();

	initTimers();

	// Enable LPUART1 RX
	// BRR = 256*16000000/115200 =
	LPUART1->BRR = 35555;
	LPUART1->CR1 = 0xD | (1<<5); // Enable Receive Data Not Empty Interrupt (RXNEIE)

	// Setup Interrupt Priority for LPUART1
	NVIC_SetPriority(LPUART1_IRQn, 0); // 0 is higher than 1 (3 bit priority)

	// Enable IRQ in NVIC
	NVIC_EnableIRQ(LPUART1_IRQn);

	// Enable GPIO as input for Button
	BTNinit();

	// Enable Clock to SYSCFG
	// To use EXTI you need to enable SYSCFG
	RCC->APB2ENR   |= 1;  // Enable Clock to SYSCFG & EXTI

	// Select Input Using EXTI-->CRx
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


	// Further config of user button interrupt
	// Select Trigger type (Failing or Raising Edge)
	EXTI->RTSR1    |= 1<<13;     // Trigger on rising edge of PC13
	                             // Use FTSR1 register for rising edge

	// Disable Interrupt Mask
	EXTI->IMR1     |= 1<<13;     // Interrupt mask disable for PC13

	// Setup Interrupt Priority for EXTI13
	NVIC_SetPriority(EXTI13_IRQn, 0); // 0 is higher than 1 (3 bit priority)

	// Enable IRQ in NVIC
	NVIC_EnableIRQ(EXTI13_IRQn);

	// Enable Global Interrupt Flag
	__enable_irq();   // No need since it is enabled by default

	while(1); // nothing inside the while loop
}

/* Define Interrupt Service Routine (ISR)
 * Interrupt Service Routine to be called when LPUART1_IRQn is raised
 * This function will check incoming LPUART1 RDR for char's sent from the
 * connected computer. Numbers 0-9 will turn ON corresponding LEDs 0-9.
 * The LEDs should be connected to pins PE_0 through PE_9.
 * LED 1 and 2 are tied to TIM2 and TIM3. So, instead of turning the LEDs ON
 * directly the function will enable or disable the timers.
*/
void LPUART1_IRQHandler(){
	char b=LPUART1->RDR;  // Reading RDR clears flag

    // Toggle LEDs based on char b value
    if      (b == '0') GPIOE->ODR ^= 1;
    else if (b == '1') {
    	TIM2->CR1 ^= 1; //TIM2 start stop for LED 1
    	GPIOE->ODR &= ~(1 << 1); // Not ideal implementation but prevents LED from staying on
    							// if timer shuts down while LED is set ON.
    }
    else if (b == '2'){
    	TIM3->CR1 ^= 1; // TIM3 start stop for LED 2
    	GPIOE->ODR &= ~(1 << 2); // Toggle PE_2 LED OFF
    }
    else if (b >= '3' && b <= '9') GPIOE->ODR ^= (1 << (b - '0'));
    else{
    	TIM2->CR1 &= ~1; // TIM2 stop for LED 1
    	TIM3->CR1 &= ~1; // TIM3 stop for LED 2
    	GPIOE->ODR = 0x000;  // Clear all LEDs
    }
}

/* Interrupt Service Routine to be called when EXTI13_IRQn is raised
 * When the USER pushbutton is pressed, on the rising edge, the microcontroller will
 * send the char buf containing my name to the computer connected via LPUART
 * It does this through a while loop that checks for the end of the buffer.
 * "while(!(LPUART1->ISR & (1 << 7)));" is used to prevent premature exit of the
 * function. It needs to wait for the transmit data to have been properly recieved
 * before sending the next char.
 * Rising edge ends up acting like a natural debounce which is nice.
 */
void EXTI13_IRQHandler(){
	char buf [] = "Taylor Zaugg ";
	int i = 0;
	while(!(buf[i] == '\0')){

		while(!(LPUART1->ISR & (1 << 7))); //Not sure I like the double loop implementation, but it works
		LPUART1->TDR = buf[i++]; //send next char from buffer to TDR
	}
    // Clear pending Interrupt
	EXTI->RPR1 |= 1<<13; // Cleared by writing 1 to it!
	                    // Use RPR1 when trigger by Rising edge
						// FPR1 for falling
}

void TIM2_IRQHandler() {
	GPIOE->ODR ^= (1 << 1); // Toggle PE_1 LED
	TIM2->SR &= ~1; // Clear flag
}

void TIM3_IRQHandler() {
	GPIOE->ODR ^= (1 << 2); // Toggle PE_2 LEd
    TIM3->SR &= ~1; // Clear flag
}

void setClks(){
	RCC->APB1ENR1 |=1<<28;   // Enable the power interface clock by setting the PWREN bits
	bitset(RCC->AHB2ENR,  6);  // GPIOG en
	bitset(RCC->AHB2ENR, 4); //GPIOE en
	RCC->APB1ENR2 |=0x1;     // Enable LPUART1EN clock
	RCC->CCIPR1   |=0x800;   // 01 for HSI16 to be used for LPUART1
	RCC->CCIPR1   &= ~(0x400);
	RCC->CFGR     |=0x1;     // Use HSI16 as SYSCLK
	RCC->CR       |=0x161;   // MSI clock enable; MSI=4 MHz; HSI16 clock enable
}

void initTimers() {
    bitset(RCC->APB1ENR1, 0); // Enable TIM2 clock
    bitset(RCC->APB1ENR1, 1); // Enable TIM3 clock

    // Configure TIM2 for 1Hz, 500ms on 500ms off for PE_1
    TIM2->PSC = 16000 - 1;  // 1ms tick (16MHz / 16,000)
    TIM2->ARR = 500 - 1; // 500ms set
    TIM2->DIER |= 1; // Enable update interrupt
    NVIC_EnableIRQ(TIM2_IRQn);

    // Configure TIM3 for 2Hz, 250ms on 250ms off for PE_2
    TIM3->PSC = 16000 - 1;
    TIM3->ARR = 250 - 1; // 250ms set
    TIM3->DIER |= 1;
    NVIC_EnableIRQ(TIM3_IRQn);
}

void enGpiogPwr(){
	bitset(PWR->CR2, 9);
}

void initGpiog(){
	//set GPIOG.7 to AF
	bitset(GPIOG->MODER,   15);  // Setting 10 in pin 7 two bit mode cfgs
	bitclear(GPIOG->MODER,  14);

	bitset(GPIOG->AFR[0], 31);  // Programming 0b1000
	bitclear(GPIOG->AFR[0], 30);
	bitclear(GPIOG->AFR[0], 29);
	bitclear(GPIOG->AFR[0], 28);

	//set GPIOG.8 to AF
	bitset(GPIOG->MODER,   17);  // Setting 10 in pin 8 two bit mode cfgs
	bitclear(GPIOG->MODER,  16);

	bitset(  GPIOG->AFR[1], 3);  // Programming 0b1000
	bitclear(GPIOG->AFR[1], 2);
	bitclear(GPIOG->AFR[1], 1);
	bitclear(GPIOG->AFR[1], 0);
}

void initGpioe(){
	//Clear mode bits
	GPIOE->MODER &= ~(0x3FFFFF);

	//set as outputs (01) For each pin PE_0 through PE_9
	GPIOE->MODER |= 0x155555;
}

void BTNinit(){
	RCC->AHB2ENR |= (1<<2); // Enable GPIOC
	// Set up the mode for button at C13
	bitclear(GPIOC->MODER, 26); // Clear bit 26 and 27
	bitclear(GPIOC->MODER, 27);
}
