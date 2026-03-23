// Includes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "main.h"
#include "stdbool.h"

// Some helper macros ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1   ) //Checks the bit number <idx> -- 0 means clear; !0 means set.


//Pre-Declarations of functions. Full declarations are after main(). ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void InitGPIOC();
void InitGPIOE();
void InitGPIOG();
void InitBtn();
void InitTIM2();
void InitTIM3();
void InitLPUART1(uint32_t baud, uint8_t stop, uint8_t data_len, bool parity_en, bool parity_odd);
void Delay_ms();

//Global Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint8_t i=0;							// Incrementing variable to track character output for "my_name".

int main(){

	//Configure GPIOs ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitGPIOC();
	InitGPIOE();
	InitGPIOG();

	//Configure Timers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	bitset(RCC->CFGR, 0);     // Use HSI16 as SYSCLK
	InitTIM2();
	InitTIM3();

	//Configure Button Interrupt ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitBtn();

	//Configure LPUART1 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitLPUART1(115200,1,8,false,false);	//InitLPUART1(baud, stop, data_len, parity_en, parity_odd)





	//Infinite Loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	while(1);

}

//Function Declarations ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void InitGPIOC(){
	/**
	  * @brief	A function to enable/configure GPIOC.
	  * @param 	None
	  * @retval	None
	*/

	bitset(RCC->AHB2ENR, 2);	// Enable Clock to GPIOC

	//Set GPIO Port C, Pin 13 (User Button) to "Input mode".
	bitclear(GPIOC->MODER, 27);
	bitclear(GPIOC->MODER, 26);
}

void InitGPIOE(){
	/**
	  * @brief	A function to enable/configure GPIOE.
	  * @param 	None
	  * @retval	None
	*/

	bitset(RCC->AHB2ENR,4);	// Enable Clock to GPIOE

	//Set GPIO Port E, Pin 0 to "General purpose output mode".
	bitclear(GPIOE->MODER, 1);
	bitset(GPIOE->MODER, 0);
	//Set GPIO Port E, Pin 1 to "General purpose output mode".
	bitclear(GPIOE->MODER, 3);
	bitset(GPIOE->MODER, 2);
	//Set GPIO Port E, Pin 2 to "General purpose output mode".
	bitclear(GPIOE->MODER, 5);
	bitset(GPIOE->MODER, 4);
	//Set GPIO Port E, Pin 3 to "General purpose output mode".
	bitclear(GPIOE->MODER, 7);
	bitset(GPIOE->MODER, 6);
	//Set GPIO Port E, Pin 4 to "General purpose output mode".
	bitclear(GPIOE->MODER, 9);
	bitset(GPIOE->MODER, 8);
	//Set GPIO Port E, Pin 5 to "General purpose output mode".
	bitclear(GPIOE->MODER, 11);
	bitset(GPIOE->MODER, 10);
	//Set GPIO Port E, Pin 6 to "General purpose output mode".
	bitclear(GPIOE->MODER, 13);
	bitset(GPIOE->MODER, 12);
	//Set GPIO Port E, Pin 7 to "General purpose output mode".
	bitclear(GPIOE->MODER, 15);
	bitset(GPIOE->MODER, 14);
	//Set GPIO Port E, Pin 8 to "General purpose output mode".
	bitclear(GPIOE->MODER, 17);
	bitset(GPIOE->MODER, 16);
	//Set GPIO Port E, Pin 9 to "General purpose output mode".
	bitclear(GPIOE->MODER, 19);
	bitset(GPIOE->MODER, 18);
}
void InitGPIOG(){
	/**
	  * @brief	A function to enable/configure GPIOG.
	  * @param 	None
	  * @retval	None
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

void InitBtn(){
	/**
	  * @brief Initializes button configuration.
	  * @param None
	  * @retval None
	*/
	RCC->APB2ENR   |= 1;  // Enable Clock to SYSCFG & EXTI

	bitset(EXTI->EXTICR[3], 9);  // Select PC13
	bitset(EXTI->RTSR1,13);     // Trigger on rising edge of PC13
	bitset(EXTI->IMR1,13);     // Interrupt mask disable for PC13

	NVIC_SetPriority(EXTI13_IRQn, 0); // 0 is higher than 1 (3 bit priority)

	NVIC_EnableIRQ(EXTI13_IRQn);

}

void InitTIM2(){
	/**
	  * @brief Initializes TIM2 timer.
	  * @param None
	  * @retval None
	*/
	bitset(RCC->APB1ENR1, 0);	// enable TIM2 clock
	TIM2->PSC = 16000 - 1;     	// Set Prescale to get 1kHz
	TIM2->ARR = 500 - 1;      	// Count 500ms
	TIM2->CNT = 0;             	// Clear counter
	bitset(TIM2->DIER, 0);    	// Set Update Interrupt Enable

	//	3- Setup Interrupt Priority
	NVIC_SetPriority(TIM2_IRQn, 0); // 0 is higher than 1 (3 bit priority)

	//	5- Enable IRQ in NVIC
	NVIC_EnableIRQ(TIM2_IRQn);
}

void InitTIM3(){
	/**
	  * @brief Initializes TIM3 timer.
	  * @param None
	  * @retval None
	*/
	bitset(RCC->APB1ENR1, 1);	// enable TIM3 clock
	TIM3->PSC = 16000 - 1;     	// Set Prescale to get 1kHz
	TIM3->ARR = 250 - 1;      	// Count 250ms
	TIM3->CNT = 0;             	// Clear counter
	bitset(TIM3->DIER, 0);    	// Set Update Interrupt Enable

	//	3- Setup Interrupt Priority
	NVIC_SetPriority(TIM3_IRQn, 0); // 0 is higher than 1 (3 bit priority)

	//	5- Enable IRQ in NVIC
	NVIC_EnableIRQ(TIM3_IRQn);
}

void InitLPUART1(uint32_t baud, uint8_t stop, uint8_t data_len, bool parity_en, bool parity_odd){
	/**
	  * @brief   This function initializes LPUART1 with the given baud rate, stop bits, data length, and parity settings.
	  * @param    baud:       The baud rate for LPUART1 communication.
	  * @param    stop:       The number of stop bits to be used.
	  *                       This could be 1 or 2, defining the end of a frame of data.
	  * @param    data_len:   The length of data bits in UART communication.
	  *                       This could be 7, 8 or 9.
	  * @param    parity_en:  Enable or disable parity check.
	  *                       If true, parity checking is enabled, otherwise it's disabled.
	  * @param    parity_odd: Define the type of parity (if parity is enabled).
	  *                       If true, odd parity is used; if false, even parity is used.
	  * @retval   None
	*/
	uint32_t reg_val = 256*(16000000/baud);
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

	LPUART1->BRR = reg_val;
	bitset(LPUART1->CR1,5);// Enable Receive Data Not Empty Interrupt (RXNEIE)
	bitset(LPUART1->CR1,3);// Enable Transmitter
	bitset(LPUART1->CR1,2);// Enable Receiver
	bitset(LPUART1->CR1,0);// Enable LPUART1


	NVIC_SetPriority(LPUART1_IRQn, 0);// Set Interrupt Priority

	NVIC_EnableIRQ(LPUART1_IRQn);// Enable IRQ in NVIC

}

void EXTI13_IRQHandler(){
	/**
	  * @brief Handler for EXTI 13 interrupts.
	  * @param None
	  * @retval None
	*/
	bitset(LPUART1->CR1,6);// Enable Transmit Data Empty Interrupt (TXEIE)
	bitset(EXTI->RPR1, 13);// Clear flag
}

void TIM2_IRQHandler(){
	/**
	  * @brief Handler for TIM2 interrupts.
	  * @param None
	  * @retval None
	*/
	bitflip(GPIOE->ODR, 1);// Toggle PE1
	bitclear(TIM2->SR, 0); // Clear flag
}


void TIM3_IRQHandler(){
	/**
	  * @brief Handler for TIM3 interrupts.
	  * @param None
	  * @retval None
	*/
	bitflip(GPIOE->ODR, 2);// Toggle PE2
	bitclear(TIM3->SR, 0); // Clear flag
}

void LPUART1_IRQHandler(){
	/**
	  * @brief Handler for LPUART1 interrupts.
	  * @param None
	  * @retval None
	*/

	char my_name [] = "Richard Groves\r\n";	// My name with carriage return and line feed.

	if (bitcheck(LPUART1->ISR, 5) != 0 ){
		uint8_t read_data=(LPUART1->RDR - 48);// Rx data (ASCII value - 48. This will set the variable to the value 0-9 if the ASCII character is in that range)

		if      (read_data==1) {
			bitset(TIM2->CR1, 0);     	// Enable TIM2
		}
		else if (read_data==2) {
			bitset(TIM3->CR1, 0);     	// Enable TIM3
		}
		else if ((read_data>=0) && (read_data<=9)) {
			bitset(GPIOE->ODR, read_data);
		}
		else {
			bitclear(TIM2->CR1, 0);     // Disable TIM2
			bitclear(TIM3->CR1, 0);     // Disable TIM3
			bitclear(GPIOE->ODR, 0);	// Turn Off PE0-9
			bitclear(GPIOE->ODR, 1);
			bitclear(GPIOE->ODR, 2);
			bitclear(GPIOE->ODR, 3);
			bitclear(GPIOE->ODR, 4);
			bitclear(GPIOE->ODR, 5);
			bitclear(GPIOE->ODR, 6);
			bitclear(GPIOE->ODR, 7);
			bitclear(GPIOE->ODR, 8);
			bitclear(GPIOE->ODR, 9);
		}
	}
	else {
		if (bitcheck(LPUART1->ISR, 7) != 0){
			LPUART1->TDR = my_name[i++];
		}

		if (my_name[i] == '\0'){
			bitclear(LPUART1->CR1,6);// Disable Transmit Data Empty Interrupt (TXEIE)
			i=0;
		}
	}
}

void Error_Handler(void) {
}
