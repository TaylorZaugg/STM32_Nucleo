// Includes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "main.h"
#include "stdbool.h"
#include "stdio.h"

// Some helper macros ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1   ) //Checks the bit number <idx> -- 0 means clear; !0 means set.


//Pre-Declarations of functions. Full declarations are after main(). ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void InitGPIOA();
void InitDAC();
void InitCOMP();
void InitADC();
void Delay_ms(uint32_t val);

// Global Variables
float adc_val;
bool adc_complete = false;

int main(){

	//Configure GPIOs ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitGPIOA();


	//Configure ADC/DAC/COMP/OPAMP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitADC();

	while(adc_complete==false);	// Wait until ADC has completed

	InitDAC();
	InitCOMP();


	//Infinite Loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	while(1){
	}

}

//Function Declarations ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void InitDAC(){
	/**
	  * @brief Configures DAC1 to output a fixed voltage as comparator reference.
	  * @param None
	  * @retval None
	*/

	bitset(RCC->APB1ENR1, 29);	// Enable Clock to GPIOA
	DAC->DHR12R1 = adc_val+200; // Set output to VT+.

	bitclear(DAC->MCR,2); 		// Set DAC Channel 1 to connect to on chip peripherals ('b011).
	bitset(DAC->MCR,1);
	bitset(DAC->MCR,0);

	bitset(DAC->CR,0);			// Enable DAC

}

void InitCOMP(){
	/**
	  * @brief Configures COMP1 to compare DAC output against PA2 analog input.
	  * @param None
	  * @retval None
	*/

	bitset(RCC->APB2ENR , 0);	// Enable Clock to COMP, SYSCFG & VREFBUF

	bitset(COMP1->CSR,8); 		// Set INPSEL = 'b10. PA2
	bitclear(COMP1->CSR,7);

	bitset(COMP1->CSR,6); 		// Set INMSEL = 'b100. DAC Channel 1
	bitclear(COMP1->CSR,5);
	bitclear(COMP1->CSR,4);

	bitset(EXTI->RTSR1,21); // Enable rising edge trigger
	bitset(EXTI->FTSR1,21);; // Enable falling edge trigger
	bitset(EXTI->IMR1,21);  // Unmask EXTI line 21 (COMP1)

	NVIC_EnableIRQ(COMP_IRQn);		//Enable interrupt for COMP
	NVIC_SetPriority(COMP_IRQn, 0); 	//Set priority

	bitset(COMP1->CSR,0); 		//Enable comparator 1.
}

void InitGPIOA(){
	/**
	  * @brief	A function to enable/configure GPIOA.
	  * @param 	N/A.
	  * @retval	N/A
	*/

	bitset(RCC->AHB2ENR, 0);	// Enable Clock to GPIOA


	bitset(GPIOA->MODER, 5); 	//Set GPIO Port A, Pin 2 to "Analog Input".
	bitset(GPIOA->MODER, 4);


	bitclear(GPIOA->MODER, 19);	//Set GPIO Port A, Pin 9 (RED LED) to "General purpose output mode".
	bitset(GPIOA->MODER, 18);
}

void InitADC(){
    /**
      * @brief Sets up the ADC in continuous conversion mode.
      * @param  None
      * @retval None
    */

    bitset(RCC->AHB2ENR, 13);  // Enable ADC clock
    RCC->CCIPR1 |= 0x3 << 28;  // Set ADC Clock to SYSCLK

    bitclear(ADC1->CR, 29);    // Disable deep power down
    bitset(ADC1->CR, 28);      // Enable voltage regulator

    Delay_ms(10);              // Wait for the voltage regulator to stabilize

    bitset(ADC1->IER, 2);      // Enable EOCIE (end of conversion interrupt enable)

    bitclear(ADC1->SQR1, 10);  // Select channel (ADC1_IN7 -> PA2)
    bitclear(ADC1->SQR1, 9);
    bitset(ADC1->SQR1, 8);
    bitset(ADC1->SQR1, 7);
    bitset(ADC1->SQR1, 6);

    bitclear(ADC1->SQR1, 3);   // L=0000 (1 channel)
    bitclear(ADC1->SQR1, 2);
    bitclear(ADC1->SQR1, 1);
    bitclear(ADC1->SQR1, 0);

    bitset(ADC1->CFGR, 13);    // Enable continuous mode (CONT=1)

    bitset(ADC1->CR, 0);       // Enable ADC

    while (bitcheck(ADC1->ISR, 0) == 0); // Wait until ADC is Ready (ADRDY)

    bitset(ADC1->CR, 2);       // Start continuous conversions

    NVIC_SetPriority(ADC1_2_IRQn, 1);
    NVIC_EnableIRQ(ADC1_2_IRQn);

}


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

void ADC1_2_IRQHandler(){
    /**
      * @brief ADC interrupt handler to process continuous conversions.
      * @param  None
      * @retval None
    */

    if (bitcheck(ADC1->ISR, 2)) {  // Check if EOC flag is set
        adc_val = ADC1->DR;        // Read ADC value (automatically clears EOC)
        adc_complete = true;       // Update flag

        bitset(ADC1->CR, 1);       // Disable ADC
    }
}


void COMP_IRQHandler()
{

    if (bitcheck(COMP1->CSR,30)==1) {
        bitset(GPIOA->ODR, 9);   // Turn ON Red LED
        bitset(EXTI->RPR1, 21);  // Clear the rising edge flag
    }
    else {
        bitclear(GPIOA->ODR, 9); // Turn OFF Red LED
        bitset(EXTI->FPR1, 21);  // Clear the falling edge flag
    }
}


void Error_Handler(void) {
}
