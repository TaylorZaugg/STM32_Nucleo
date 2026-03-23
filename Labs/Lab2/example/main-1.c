// Includes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "main.h"
#include "stdbool.h"

// Some helper macros ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1   ) //Checks the bit number <idx> -- 0 means clear; !0 means set.


//Pre-Declarations of functions. Full declarations are after main(). ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void InitGPIOA();
void InitGPIOB();
void InitGPIOC();
void InitGPIOG();
void InitLPUART1(uint32_t baud, uint8_t stop, uint8_t data_len, bool parity_en, bool parity_odd);
void Delay_ms();


int main(){
	//Variable Declarations ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	char my_name [] = "Richard Groves\r\n";	// My name with carriage return and line feed.
	char read_data = '\0';  // Initialize with a null character
	bool usr_btn_oneshot = 0b0;				// One-shot bit to ensure the button press is only evaluated once per press.
	uint8_t i=0;							// Incrementing variable to track character output for "my_name".

	//Configure GPIOs ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitGPIOA();
	InitGPIOB();
	InitGPIOC();
	InitGPIOG();

	//Configure LPUART1 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitLPUART1(57600,1,8,false,false);	//InitLPUART1(baud, stop, data_len, parity_en, parity_odd)


	//Logic ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	while(1){

		//When the user button is pressed, transmit the next character of "my_name".
		if ((bitcheck(GPIOC->IDR, 13) == 1) & (bitcheck(LPUART1->ISR, 7) != 0 ) & !usr_btn_oneshot) {
			usr_btn_oneshot = 0b1;
			LPUART1->TDR = my_name[i++];
			if (my_name[i] == '\0') i=0;
			Delay_ms(50);
		}

		//When the user button is released, clear the one-shot bit.
		if ((bitcheck(GPIOC->IDR, 13) == 0) & usr_btn_oneshot){
			usr_btn_oneshot = 0b0;
			Delay_ms(50);
		}

		//Evaluate received data from UART.
		//'r' or 'R' turn on the red LED.
		//'b' or 'B' turn on the blue LED.
		//'g' or 'G' turn on the green LED.
		//All other values will turn off all LEDs.
		if (bitcheck(LPUART1->ISR, 5) == 1) {
			read_data = LPUART1->RDR;
			if ((read_data == 'r') | (read_data == 'R')) {
				bitflip(GPIOA->ODR, 9);	//Red LED - Toggle
			} else if ((read_data == 'b') | (read_data == 'B')) {
				bitflip(GPIOB->ODR, 7);	//Blue LED - Toggle
			} else if ((read_data == 'g') | (read_data == 'G')) {
				bitflip(GPIOC->ODR, 7);	//Green LED - Toggle
			} else {
				bitclear(GPIOA->ODR, 9);	//Red LED - Off
				bitclear(GPIOB->ODR, 7);	//Blue LED - Off
				bitclear(GPIOC->ODR, 7);	//Green LED - Off
			}
		}
	}

}

//Function Declarations ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void InitGPIOA(){
	/**
	  * @brief	A function to enable/configure GPIOA.
	  *
	  * @param 	N/A.
	  *
	  * @retval	N/A
	*/

	bitset(RCC->AHB2ENR, 0);	// Enable Clock to GPIOA

	//Set GPIO Port A, Pin 9 (RED LED) to "General purpose output mode".
	bitset(GPIOA->MODER, 18);
	bitclear(GPIOA->MODER, 19);
}

void InitGPIOB(){
	/**
	  * @brief	A function to enable/configure GPIOB.
	  *
	  * @param 	N/A.
	  *
	  * @retval	N/A
	*/

	bitset(RCC->AHB2ENR, 1);	// Enable Clock to GPIOB

	//Set GPIO Port B, Pin 7 (BLUE LED) to "General purpose output mode".
	bitset(GPIOB->MODER, 14);
	bitclear(GPIOB->MODER, 15);
}

void InitGPIOC(){
	/**
	  * @brief	A function to enable/configure GPIOC.
	  *
	  * @param 	N/A.
	  *
	  * @retval	N/A
	*/

	bitset(RCC->AHB2ENR, 2);	// Enable Clock to GPIOC

	//Set GPIO Port C, Pin 7 (GREEN LED) to "General purpose output mode".
	bitset(GPIOC->MODER, 14);
	bitclear(GPIOC->MODER, 15);

	//Set GPIO Port C, Pin 13 (User Button) to "Input mode".
	bitclear(GPIOC->MODER, 26);
	bitclear(GPIOC->MODER, 27);
}

void InitGPIOG(){
	/**
	  * @brief	A function to enable/configure GPIOG.
	  *
	  * @param 	N/A.
	  *
	  * @retval	N/A
	*/

	bitset(RCC->AHB2ENR,   6);  // Enable Clock to GPIOG
	bitset(RCC->APB1ENR1, 28);  // Enable Clock to PWR Interface
	bitset(PWR->CR2, 9);        // Enable GPIOG power

	//Set GPIOG.7 to AF8 (LPUART1_TX)
	bitset(GPIOG->MODER,    15);  // Setting 0b10 (Alternate Function) in pin 7 two bit mode cfgs
	bitclear(GPIOG->MODER,  14);

	bitset(GPIOG->AFR[0],   31);  // Programming 0b1000 (AF8 = LPUART1_TX)
	bitclear(GPIOG->AFR[0], 30);
	bitclear(GPIOG->AFR[0], 29);
	bitclear(GPIOG->AFR[0], 28);

	//Set GPIOG.8 to AF8 (LPUART1_RX)
	bitset(GPIOG->MODER,    17);  // Setting 0b10 (Alternate Function) in pin 8 two bit mode cfgs
	bitclear(GPIOG->MODER,  16);

	bitset(GPIOG->AFR[1], 3);  // Programming 0b1000 (AF8 = LPUART1_RX)
	bitclear(GPIOG->AFR[1], 2);
	bitclear(GPIOG->AFR[1], 1);
	bitclear(GPIOG->AFR[1], 0);

}

void InitLPUART1(uint32_t baud, uint8_t stop, uint8_t data_len, bool parity_en, bool parity_odd){
	/**
	  * @brief   This function initializes LPUART1 with the given baud rate, stop bits, data length, and parity settings.
	  *
	  * @param    baud:       The baud rate for LPUART1 communication.
	  * @param    stop:       The number of stop bits to be used.
	  *                       This could be 1 or 2, defining the end of a frame of data.
	  * @param    data_len:   The length of data bits in UART communication.
	  *                       This could be 7, 8 or 9.
	  * @param    parity_en:  Enable or disable parity check.
	  *                       If true, parity checking is enabled, otherwise it's disabled.
	  * @param    parity_odd: Define the type of parity (if parity is enabled).
	  *                       If true, odd parity is used; if false, even parity is used.
	  *
	  * @retval   None
	*/
	uint32_t reg_val = (uint32_t)(((uint64_t)256 * 16000000) / baud);
	bitset(RCC->APB1ENR2,  0);  // Enable Clock to LPUART

	bitset(RCC->CCIPR1,   11);  // Select the high speed internal (HSI) oscillator as the clock to LPUART1 (16MHz)
	bitclear(RCC->CCIPR1, 10);  //
	bitset(RCC->CR, 8);         // HSI16 clock enable

	// Configure the # of stop bits.
	if (stop == 1){
		bitclear(LPUART1->CR2,13);
		bitclear(LPUART1->CR2,12);
	} else if (stop == 2) {
		bitset(LPUART1->CR2,13);
		bitclear(LPUART1->CR2,12);
	}

	// Configure the # of data bits.
	if (data_len == 7){
		bitset(LPUART1->CR1,28);
		bitclear(LPUART1->CR1,12);
	} else if (data_len == 8) {
		bitclear(LPUART1->CR1,28);
		bitclear(LPUART1->CR1,12);
	} else if (data_len == 9) {
		bitclear(LPUART1->CR1,28);
		bitset(LPUART1->CR1,12);
	}


	// Configure the parity.
	if (parity_en){
		bitset(LPUART1->CR1,10);

		//Configure parity odd/even if parity is enabled.
		if (parity_odd){
			bitset(LPUART1->CR1,9);
		} else{
			bitclear(LPUART1->CR1,9);
		}
	} else {
		bitclear(LPUART1->CR1,10);
	}

	// BRR = 256*16000000/115200 ~= 35555
	LPUART1->BRR = reg_val;
	LPUART1->CR1 = 0xD; // 0x1101  --> TX, RX are enabled and UART is Enabled.
}

void Delay_ms(uint32_t val){
	/**
	  * @brief	A function to delay the program (in milliseconds).
	  *
	  * @param 	val Delay time in ms.
	  *
	  * @retval	N/A
	*/

	// Convert desired ms (val) to a loop count.
	float count = val * 282.398124876451;

	// Loop to delay program.
	int x = 0;
	while(x < count){
		x += 1;
	}
}

void Error_Handler(void) {
}
