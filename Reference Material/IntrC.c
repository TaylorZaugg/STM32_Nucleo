#include "stm32l552xx.h" // Has all definitions for the specific MCU
// You could include more libraries if needed


// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) & 1      ) //Checks the bit number <idx> -- 0 means clear; 1 means set.


// This is the main function that the CPU will execute
// on a start-up.
int main(){

	// Procedure to use General Purpose Input/Output (GPIO)
	// 1- Enable the clock feeding into the GPIO port
	// 2- Set the Mode for the pin (Input, output, alternate function, analog)
	// 3- For digital input, read IDR
	//    For digital output, write ODR
	//

    // Enable the clock going to GPIO A9 by setting the ENABLE bit
	// in the Reset and Clock Control Register for AHB2
	RCC->AHB2ENR  |= 0x1;

	// Initialize GPIOA PIN 9 Mode --> (00) Input,
	//                                 (01) Output,
	//                                 (10) Alternate Function,
	//                             and (11) Analog mode
	// Clear the two bits for A9   //  30 28 26 24  22 20 18 16  14 12 10  8   6  4  2  0 : bit index
    GPIOA->MODER  &= ~((0x3)<<18); // <00_00_00_00__00_00_00_00__00_00_00_00__00_00_00_00>
    // Sets A9 to be output        //                     ^^
    GPIOA->MODER  |=   (0x1)<<18;

    // To control how strong the pin drive is
    //GPIOA->OSPEEDR |= (0x3)<<18;

    // Method 1:
    // To drive the A9 high
    GPIOA->ODR    |=  (0x1) << 9;

    // Method 2:
    // GPIOA->ODR absolute address is 0x42020014
    // Create a pointer and write to it would be identical to method 1
    uint32_t * ptr=0x42020014;

    // Just a string
    char mytxt [] = "ECE433/ECE533\n";

    /* Infinite loop */
   while (1) {
	//// Method 1: flip A9
	//bitflip(GPIOA->ODR, 9);

	//// Method 2:
	//GPIOA->ODR    ^=  (0x1) << 9;

	//// Method 3: using a pointer
	*ptr ^=  (0x1) << 9;


	// Pause
    for (int i=0; i<0xabcd; i++)
    	mytxt[i%5] = mytxt[i%9]; // Just to keep cpu busy
  }

  return 0;
}


