/* Taylor Zaugg
 * ECE 433
 * Lab 1 Blinky
 * Date: 2025-01-30
 */

#include "stm32l552xx.h"

// Some helper macros provided by IntrC.c
#define bitset(word, idx) ((word) |= (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx) ((word) &= ~(1<<(idx))) //Clears the bit number <idx>-- All other bits are not affected.
#define bitflip(word, idx) ((word) ^= (1<<(idx))) //Flips the bit number <idx>-- All other bits are not affected.
#define bitcheck(word, idx) ((word>>idx) & 1 ) //Checks the bit number <idx>-- 0 means clear; 1 means set.

/* delay_ms Prototype function for delaying time in ms
 * uint32_t val: desired delay in ms
*/
void delay_ms(uint32_t val);

int main(){
	// Turn on output mode for GPIO related to LED pins for Green, Red and Blue LED's

	//LED green PC07
	// So, RCC port c clock needs turned on,
	// and GPIO pin 7 to 01 for output
	bitset(RCC->AHB2ENR, 2);
	bitset(GPIOC->MODER, 14);
	bitclear(GPIOC->MODER, 15);
	//LED green should be available to use

	//LED blue, PB07
	bitset(RCC->AHB2ENR, 1);
	bitset(GPIOB->MODER, 14);
	bitclear(GPIOB->MODER, 15);

	//LED red, PA09
	bitset(RCC->AHB2ENR, 0);
	bitset(GPIOA->MODER, 18);
	bitclear(GPIOA->MODER, 19);

	// MSI 4 MHz clock is default after system reset.

	// Send output to LEDs, repeat in loop
	/*
	 * Green LED, blinks 1 time per second (500ms ON, 500ms OFF)
	 * Blue LED, blinks 2 times per second (250ms ON, 250ms OFF)
	 * Red LED, blinds 4 times per second (125ms ON, 250ms OFF)
	 * Using nested for loops and a single 125ms delay working to delay all LEDs
	 */
	 
	while(1){
		// Toggle GREEN LED on, 500ms delay
		// GREEN LED, GPIOA pin 7
		GPIOC->ODR ^= 1<<7;
		for(int i=0; i<2; i++){
			// Toggle BLUE LED on, 250ms delay
			// BLUE LED, GPIOB pin 7
			GPIOB->ODR ^= 1<<7;
			for(int i=0; i<2;i++){
				// Toggle RED LED on, 125ms delay
				// Red LED, GPIOA pin 9
				GPIOA->ODR ^= 1<<9;
				// delay per red led, drives other delays based on loop
				delay_ms(125);
			}
		}
	}
	return 0;
}

void delay_ms(uint32_t val){
	/*delay multiplier currently based on the MSI 4MHz clock
	 * as this is the reset default
	 * 4,000,000 cycles
	 * the value we want is in ms, or 1000ms to 1 s.
	 * Need to add a multiplier so the delay value slows the clock down
	 * an appropriate amount. Trial and error estimates this to be 400.
	*/
	int delaymult = 400;
	for(int i=0;i < (val * delaymult); i++);
	return;
}


