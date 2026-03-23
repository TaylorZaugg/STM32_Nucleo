 /*
  * Taylor Zaugg
  * ECE 433
  * Lab 6 DAC
  * Date: 2025-03-13
  *
  * This program uses GPIOA pin 4 as a DAC analog output.
  * The DAC is fed sine wave values from the global buffer variable int dac_cal_12b [100]
  * Each time TIM2 counts to 10us it will trigger the DAC to output of dac_cal_12b[step]
  * Where step is a global volatile variable that keeps track of where the DAC output is
  * in the buffer so it can reset to the beginning when it finished the wave.
  *
  * Testing:
  * Values in the debugger seem to show that things are working.
  * Attempts to use the ADC to test the DAC output did not go well. I cannot get the ADC to trigger an
  * interrupt now for some reason. Since I think that the DAC is somewhat working, I try to come back to
  * testing later after other parts if I have time
  */


// Includes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "main.h"
#include "stdbool.h"
#include "stdio.h"
#include "math.h"

// Some helper macros ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1   ) //Checks the bit number <idx> -- 0 means clear; !0 means set.


//Pre-Declarations of functions. Full declarations are after main(). ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void InitGPIOC();
void InitGPIOG();
void InitBtn();
void InitADC();
void InitLPUART1(uint32_t baud, uint8_t stop, uint8_t data_len, bool parity_en, bool parity_odd);
void LPUART1write();
void myprint();
void Delay_ms();

// lab 6 new/changed functions
void InitDAC();
void InitTIM2();

void RLEDinit();
void RLEDtoggle();
void InitTIM1();

//Global Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint8_t i=0;								// Incrementing variable to track character output for "my_name".

volatile int step = 0; //incrementing var to track sinewave buffer position for the dac
int dac_val_12b [100]; // to be used later

int main(){

	//Configure GPIOs ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitGPIOC();
	InitGPIOG();

	RLEDinit(); //Debugging

	//Configure Timers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	bitset(RCC->CFGR, 0);     				// Use HSI16 as SYSCLK
	InitTIM2();
	InitTIM1();
	//Configure Button Interrupt ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//InitBtn();

	//Configure LPUART1 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitLPUART1(115200,1,8,false,false);	//InitLPUART1(baud, stop, data_len, parity_en, parity_odd)

	//Configure ADC ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitADC();

	//Configure DAC ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitDAC();

	//Make sine wave buffer ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	for(int step = 0; step < 100; step++){
		dac_val_12b[step]=(4095/2)*(1+sin(2*3.14*step/99)); // where step is 0 to 99
	}

	//Start TIM2 Interrupt last ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    NVIC_SetPriority(TIM2_IRQn, 0);  // Set TIM2 interrupt priority
    NVIC_EnableIRQ(TIM2_IRQn);       // Enable TIM2 interrupt in NVIC

	//Infinite Loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	while(1);

}

// Lab 6 Function Declarations new and modified

void TIM2_IRQHandler(void) {
    if (bitcheck(TIM2->SR,0)) {  // Check if UIF is set
        bitclear(TIM2->SR, 0);  // Clear UIF flag
        DAC->DHR12R1 = dac_val_12b[step];  // Update DAC output
        step = (step + 1) % 100;  // Increment step with wrap-around
    }
}


void InitDAC(){
	// Enable DAC1 RCC, bit 29 
	bitset(RCC->APB1ENR, 29);

	//Enable GPIO in analog mode
	//GPIO A, pin 4 DAC_OUT1 pin 5 DAC_OUT2
	bitset(RCC->AHB2ENR, 0);// Enable GPIOA Clock

	// GPIO bit 9 and 8 need to be 11 for analog out
	bitset(GPIOA->MODER, 8);
	bitset(GPIOA->MODER, 9);


	//TODO Configure and Enable DAC
	bitset(DAC->CR, 0);// bit 0 EN1, enable

	//Config? Default fine?
	//Needs to run off of timer, TIM2 is fine.

	bitset(DAC->CR, 1); //Enable channel 1 trigger for timer

	//TSEL bits 5:2,
	//TIM2_TRGO Internal signal from on-chip timers 0010
	bitclear(DAC->CR,2);	//0b0010
	bitset(DAC->CR,3);
	bitclear(DAC->CR,4);
	bitclear(DAC->CR,5);
}

void InitTIM2(){
    // Enable TIM2 clock, bit 0
    bitset(RCC->APB1ENR1, 0);  // TIM2 clock enable

	//TODO Configure timer to interrupt every 10us (100 samples per 1 msec for 1KHz sinewave)
    // Set prescaler for 1MHz timer clock
    TIM2->PSC = 16 - 1;  // 16MHz/16 = 1MHz

    // Set auto-reload for 100kHz update rate (1MHz/(9+1) = 100kHz)
    TIM2->ARR = 10 - 1;  // ARR = 9

    // Clear counter
    TIM2->CNT = 0;

	bitclear(TIM2->CCMR1,14); 	//Configure channel 2 to toggle
	bitset(TIM2->CCMR1,13);
	bitset(TIM2->CCMR1,12);

    // Configure TIM2 TRGO to send an update event as trigger output:
    TIM2->CR2 &= ~(0b111 << 4);   // Clear MMS bits
    TIM2->CR2 |= (0b010 << 4);    // MMS = 0b010: Update event

    // Enable update interrupt (UIE) so the handler gets called
    bitset(TIM2->DIER, 0);   // Bit 0: UIE

    // Enable TIM2 counter
    bitset(TIM2->CR1, 0);  // Start timer (CEN)
}


void InitADC(){
    // ADC Configuration -------------------------------------

	// Enable ADC Clock
	bitset(RCC->AHB2ENR, 13);  // Enable ADC clock
	RCC->CCIPR1 |=0x3<<28;     // Route SYSCLK (HCLK) to ADC

    // Turn on ADC Voltage Regulator
	bitclear(ADC1->CR, 29);  // Get out of deep power down mode
    bitset(ADC1->CR, 28);

    // External Trigger Enable and Polarity; EXTEN= 0b01 for rising edge
    bitset(ADC1->CFGR,   10);
    bitclear(ADC1->CFGR, 11);

    //External trigger (EXT0) is connected to TIM1_CH1; EXTSEL=0000
    bitclear(ADC1->CFGR, 6); // 0b0000
    bitclear(ADC1->CFGR, 7);
    bitclear(ADC1->CFGR, 8);
    bitclear(ADC1->CFGR, 9);

    // Wait for the voltage regulator to stabilize
	Delay_ms(10);

    //Set regular mode for temp sensor or PC00
    ADC1->SQR1 = (1<<6)|(0); 	       // L=0 (one channel to read), SQ1=IN1 which is connected to PC0 (Called ADC1_IN1)

}

void InitTIM1(){
	bitset(RCC->APB2ENR, 11);    // enable TIM1 clock
	TIM1->PSC = 16000-1;				//1 MHz clock
	TIM1->ARR = 9;      // Count PERIOD-1 times
	TIM1->CCMR1 = 0x30;          // Set output to toggle on match (Output Compare Mode)
	TIM1->CCR1 = 1;				 // Output will toggle when CNT==CCR1
	bitset(TIM1->BDTR, 15);      // Main output enable
	TIM1->CCER |= 1;             // Enable CH1 compare mode
	TIM1->CNT = 0;               // Clear counter
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

//Function Declarations ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void Delay_ms(uint32_t val){
	/**
	  * @brief	A function to delay the program (in milliseconds).
	  * @param 	val Delay time in ms.
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

void myprint (char msg[]){
	/**
	 * @brief Sends a string over LPUART1 by transmitting each character until the null terminator is reached. This function is used for sending messages via LPUART1 to facilitate debugging or communication over serial.
	 * @param msg A pointer to the character array (string) that will be sent over LPUART1. The string must be null-terminated.
	 * @retval None
	*/

    uint8_t idx=0;

    // Loop through each character in the string.
    while(msg[idx]!='\0')
    {
    	LPUART1write(msg[idx++]);
    }
}

void LPUART1write (int ch) {
	/**
	  * @brief Transmits a single character over LPUART1. It waits for the transmit data register to be empty (indicating the UART is ready to transmit the next character) before sending the character. This function is a fundamental building block for serial communication over LPUART1.
	  * @param ch The character to be transmitted over LPUART1. Although the parameter is an integer, only the least significant byte (LSB) will be sent.
	  * @retval None
	*/

    while (!(LPUART1->ISR & 0x0080)); 	// wait until Tx buffer empty
    LPUART1->TDR = (ch & 0xFF);			// Write character to the Transmit Data Register (TDR).
}


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

	//Set GPIO Port C, Pin 0 to "Analog Input".
    bitset(GPIOC->MODER, 0);
    bitset(GPIOC->MODER, 1);

	//Set GPIO Port C, Pin 1 to "Analog Input".
    bitset(GPIOC->MODER, 2);
    bitset(GPIOC->MODER, 3);
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

	bitset(EXTI->EXTICR[3], 9);  	// Select PC13
	bitset(EXTI->RTSR1,13);     	// Trigger on rising edge of PC13
	bitset(EXTI->IMR1,13);     		// Interrupt mask disable for PC13

	NVIC_SetPriority(EXTI13_IRQn, 0); 	// 0 is higher than 1 (3 bit priority)

	NVIC_EnableIRQ(EXTI13_IRQn);		// Enable EXTI13 interrupt in NVIC.

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
	bitset(LPUART1->CR1,3);// Enable Transmitter
	bitset(LPUART1->CR1,2);// Enable Receiver
	bitset(LPUART1->CR1,0);// Enable LPUART1

}

//void EXTI13_IRQHandler(){
//	/**
//	  * @brief Handler for EXTI 13 interrupts.
//	  * @param None
//	  * @retval None
//	*/
//    bitset(ADC1->CR,3);     // Start injected conversion
//
//	bitset(EXTI->RPR1, 13);	// Clear flag
//}


void ADC1_2_IRQHandler(){
	RLEDtoggle(); // Is this even working?
	NVIC_DisableIRQ(ADC1_2_IRQn);

	// Check if regular conversion is complete.
	if (bitcheck(ADC1->ISR,2)==1){

		char  txt [20];

		int adc_val = (ADC1->DR);						// Read regular ADC value
		int scaled_val = (adc_val * 99) / 4095; 	// Convert ADC code to milli-volts.

		sprintf(txt, "$%d;", scaled_val);				// Format output string.
		myprint(txt);									// Send string over UART.
	}
	TIM1->SR &= ~1; // Clear UIF Bit
	NVIC_EnableIRQ(ADC1_2_IRQn);
}


void Error_Handler(void) {
}
