 /*
  * Taylor Zaugg
  * ECE 433
  * Lab 6 COMP
  * Date: 2025-03-13
  *
  * ISSUES: I was unable to verify the test based on the issues setting a ambient temperature value.
  * The LED will toggle when the sensor is over the ambient, but setting the ambient temp would have
  * been much better through an ADC value from the previous lab or a current ambient value. 
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

void Delay_ms();
void InitDAC();
void InitCOMP();

void RLEDinit();
void RLEDtoggle();

int main() {
	InitDAC();
	InitCOMP();
	RLEDinit();

	//TODO Set DAC1 output little higher V than ambient of temp sensor
	
	uint16_t Vt0 = 550;  // TODO This would have been better to set as a real sensor value + some value


	//Loop writing same value to the dac hold register every so often so comparing stays consistent
	// DAC->DHR12R1

	while(1){
		DAC->DHR12R1 = Vt0;
		if(bitcheck(COMP1->CSR,30) ){
			bitset(GPIOA->ODR,9);
		}
		else{
			bitclear(GPIOA->ODR,9);
		}
		Delay_ms(50);
	}
}

// Lab 6 Function Declarations new and modified

void InitCOMP(){

	//Enable COMP1 RCC---------------------------
	// COMP1 on APB2 enable
	bitset(RCC->APB2ENR,0);

	//Enable GPIO Analog for COMP----------------------
	//Enable GPIOA RCC
	bitset(RCC->AHB2ENR,0);

	//Set GPIOA pin 2 to "Analog Input" or 11 bits 5:4
	bitset(GPIOA->MODER, 5);
	bitset(GPIOA->MODER, 4);

	// COMP1->CSR->INPSEL for COMP to GPIOA pin 2, 0b10 for this.----------------------
	bitset(COMP1->CSR, 8);
	bitclear(COMP1->CSR, 7);

	//External input for COMP for Temp sensor on GPIO bit

	//Config COMP 1 inputs in CSR register --------------------
	// Internal DAC value. CSR bits 6:4, DAC Channel1 = 0b100
	bitset(COMP1->CSR, 6);
	bitclear(COMP1->CSR,7);
	bitclear(COMP1->CSR,8);


	//Enable COMP1 -------------------------------
	bitset(COMP1->CSR, 0);

}

//TODO Set DAC 1 little higher than Vt0, as Vt+
void InitDAC(){
	// Enable DAC1 RCC
	// DAC1 interface clock enable
	bitset(RCC->APB1ENR1, 29);

	//Configure DAC1 so output is connected to chip peripherals internally, via MCR register
	//DAC MODE1 bits 2:0, channel1 mode
	//011 DAC channel1 is connected to on chip peripherals with buffer disabled
	bitclear(DAC->MCR, 2); //0b011
	bitset(DAC->MCR, 1);
	bitset(DAC->MCR, 0);

	// bit 0 EN1, enable DAC
	bitset(DAC->CR, 0);

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


void Error_Handler(void) {
}
