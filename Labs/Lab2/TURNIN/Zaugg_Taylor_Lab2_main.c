/* Taylor Zaugg
 * ECE 433
 * Lab 2 USART
 * Date: 2025-02-13
 *
 * This program uses LPUART to communicate back and forth with a connected computer
 * When the computer sends 'g' or 'G' the GREEN LED on the board will toggle ON
 * When the computer sends 'b' or 'B' the BLUE LED on the board will toggle ON
 * When the computer sends 'r' or 'R' the RED LED on the board will toggle ON
 * When the computer sends any other character it will turn ALL LED's OFF
 *
 * LPUART communicates at 57600 Baud rate, no parity, 8 bits, 1 stop bit
 *
 * when the USER pushbutton is pressed on the board (blue button), the computer will receive 1 character at a time via LPUART
 * The full buffer is "Taylor Zaugg " when it sends the last character in the buffer it will wrap around and begin again.
 *
 * NOTES:
 * I updated the "delay_ms" function based on feedback from lab 1. This delay function is used to debounce the pushbutton.
 * The delay should be more accurate now.
 * Previous delay multiplier was 400
 * blue LED was blinking at 1.5421Hz when 2Hz was expected
 * 		1.5421 / 2 = 0.77105
 * 		0.77105 * 400 = 308.42
 * New delay multiplier is now set as 308.
 */

#include "stm32l552xx.h" // Has all definitions for the specific MCU
// You could include more libraries if needed

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1    ) //Checks the bit number <idx> -- 0 means clear; !0 means set.

// Function declarations
void setClks();
void enGpiogPwr();
void initGpiog();
void initGpioLED();// LED GPIO's enable switch on
void initLPUART1();
void initGPIOButton();// USER Push button enable and configuration
void delay_ms(uint32_t val);


// This is the main function that the CPU will execute
// on a start-up.
int main(){
	//Enable clocks
	setClks();

	//Enable PWR going to PORT G
	enGpiogPwr();

	//Configure GPIOG
	initGpiog();

	//Configure GPIO A, B, and C for LED output
	initGpioLED();

	//Configure LPUART1
	initLPUART1();

	//Configure GPIO C for pushbutton input
	initGPIOButton();

	//Name to send one char at time via UART
	char buf [] = "Taylor Zaugg ";

	uint8_t i=0; //buf position variable

	uint8_t input;	// store value from RDR to compare

	// char values to keep current states of LED's when toggled
	char green = 0;
	char blue = 0;
	char red = 0;

	// Main Loop
	while(1){

		// LPUART1->ISR bit 5 is RXNE - Read Data Register Not Empty
		if(bitcheck(LPUART1->ISR, 5)){

			//LPUART1->RDR is read register.
			input =(uint8_t)LPUART1->RDR;

			switch(input){	// Check which LED(s) to turn on
									// Numbers are the related ascii values, for debugging
				case 'G': case 'g': // "G" = 71, "g" = 103
					green = 1;
					break;
				case 'B': case 'b': // "B" = 66, "b" =98
					blue = 1;
					break;
				case 'R': case 'r': // "R" = 82, "r" = 114
					red = 1;
					break;
				default:
					red = 0;
					green = 0;
					blue = 0;
			}
		}

		//Green LED on or off?
		if(green)
			bitset(GPIOC->ODR ,7);
		else
			bitclear(GPIOC->ODR, 7);

		//Blue LED on or off?
		if(blue)
			bitset(GPIOB->ODR, 7);
		else
			bitclear(GPIOB->ODR, 7);

		// Red LED on or off?
		if(red)
			bitset(GPIOA->ODR, 9);
		else
			bitclear(GPIOA->ODR, 9);

		/*
		 * Button is PC13 or GPIOC pin 13.
		 * GPIOC power already on(but double checked via initGPIOButton()
		 * Should be good to use
		 */
		if(bitcheck(GPIOC->IDR, 13)){	//Check if button pressed
			delay_ms(200); //delay as debounce
			LPUART1->TDR = buf[i++]; //send next char from buffer
			if (buf[i] == '\0') i=0; //reset i if at end of buffer, loop back to beginning
		}
	}
	return 0;
}

void setClks(){
	bitset(RCC->APB1ENR1, 28);  // Enable Clock to PWR Interface
	bitset(RCC->AHB2ENR,   6);  // Enable Clock to GPIOG
	bitset(RCC->APB1ENR2,  0);  // Enable Clock to LPUART
	bitset(RCC->CCIPR1,   11);  // Select the high speed internal (HSI) oscillator as the clock to LPUART1 (16MHz)
	bitclear(RCC->CCIPR1, 10);  //
	bitset(RCC->CR, 8);         // HSI16 clock enable
}
void enGpiogPwr(){
	bitset(PWR->CR2, 9);        // Enable GPIOG power
}
void initGpiog(){
	//set GPIOG.7 to AF
	bitset(GPIOG->MODER,    15);  // Setting 0b10 in pin 7 two bit mode cfgs
	bitclear(GPIOG->MODER,  14);

	bitset(GPIOG->AFR[0],   31);  // Programming 0b1000
	bitclear(GPIOG->AFR[0], 30);
	bitclear(GPIOG->AFR[0], 29);
	bitclear(GPIOG->AFR[0], 28);

	//set GPIOG.8 to AF
	bitset(GPIOG->MODER,    17);  // Setting 0b10 in pin 8 two bit mode cfgs
	bitclear(GPIOG->MODER,  16);

	bitset(  GPIOG->AFR[1], 3);  // Programming 0b1000
	bitclear(GPIOG->AFR[1], 2);
	bitclear(GPIOG->AFR[1], 1);
	bitclear(GPIOG->AFR[1], 0);

}

void initGPIOButton(){
	bitset(RCC->AHB2ENR, 2); //ensure clock is on for GPIOC

	bitclear(GPIOC->MODER, 26); //Ensure MODER set for input or 00 for pin 13
	bitclear(GPIOC->MODER, 27);
}


void initGpioLED(){
	// Turn on output mode for GPIO related to LED pins for Green, Red and Blue LED's
	//LED green PC07
	bitset(RCC->AHB2ENR, 2);
	bitset(GPIOC->MODER, 14);
	bitclear(GPIOC->MODER, 15);

	//LED blue, PB07
	bitset(RCC->AHB2ENR, 1);
	bitset(GPIOB->MODER, 14);
	bitclear(GPIOB->MODER, 15);

	//LED red, PA09
	bitset(RCC->AHB2ENR, 0);
	bitset(GPIOA->MODER, 18);
	bitclear(GPIOA->MODER, 19);
}
void delay_ms(uint32_t val){
	/*delay multiplier currently based on the MSI 4MHz clock
	 * as this is the reset default
	 * 4,000,000 cycles
	 * the value we want is in ms, or 1000ms to 1 s.
	 * Need to add a multiplier so the delay value slows the clock down
	 * an appropriate amount. Trial and error estimates this to be 400.
	*/
	int delaymult = 308;
	for(int i=0;i < (val * delaymult); i++);
	return;
}

void initLPUART1(){

	// BRR = 256*16000000/115200 =
	//LPUART1->BRR = 35555;

	// Want baud rate of 57600
	// BRR = 256*16000000/57600 = 71111.111
	LPUART1->BRR = 71111;
	LPUART1->CR1 = 0xD; // 0x1101  --> TX, RX are enabled and UART is Enabled.
}
