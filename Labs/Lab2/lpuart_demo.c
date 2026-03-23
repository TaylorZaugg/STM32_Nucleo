#include "stm32l552xx.h" // Has all definitions for the specific MCU
// You could include more libraries if needed


// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1    ) //Checks the bit number <idx> -- 0 means clear; !0 means set.


// Function declaration
void setClks();
void enGpiogPwr();
void initGpiog();
void initLPUART1();

// This is the main function that the CPU will execute
// on a start-up.
int main(){
	//Enable clocks
	setClks();

	//Enable PWR going to PORT G
	enGpiogPwr();

	//Configure GPIOG
	initGpiog();

	//Configure LPUART1
	initLPUART1();

	// Test sending
	char buf [] = "Hello !!";
	uint8_t i=0;

	while(bitcheck(LPUART1->ISR, 7) == 1 ){
		LPUART1->TDR = buf[i++];
		if (buf[i] == '\0') i=0;
		for(int j =0; j<10000; j++);
	}

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
void initLPUART1(){

	// BRR = 256*16000000/115200 =
	LPUART1->BRR = 35555;
	LPUART1->CR1 = 0xD; // 0x1101  --> TX, RX are enabled and UART is Enabled.
}

