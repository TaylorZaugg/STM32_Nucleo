// Includes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "main.h"
#include "stdbool.h"
#include "stdlib.h"


// Some helper macros ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1   ) //Checks the bit number <idx> -- 0 means clear; !0 means set.


//Pre-Declarations of functions. Full declarations are after main(). ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void InitGPIOA();
void InitGPIOC();
void InitGPIOB();
void InitInt();
void InitI2C();
void InitIOExp();
void I2C_Write(uint8_t slv_addr, uint8_t* data, uint8_t data_length );
uint8_t* I2C_Read(uint8_t slv_addr, uint8_t reg_addr, uint8_t data_length );
bool I2C_CheckAddr(uint8_t slv_addr);
void Delay_ms();

// Global Constants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static const uint8_t I2C_SLAVE_ADDR = 0b00100000;


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * - NOTES
 * If I/O Expander is not found at I2C_SLAVE_ADDR, the red LED on the NUCLEO
 * board will light up.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * -WIRING
 *	      NUCLEO					      I/O Expander
 *_______________________       ________________________________
 *|                     |       |              	  			   |
 *|                  3V3|------>|VDD (Pin 18)  	  (Pin 17) GPIO|-->(LED 4)
 *|                     |   |   |              				   |
 *|                     |   --->|~RESET (Pin 6)	  (Pin 16) GPIO|-->(LED 3)
 *|                     |       |              				   |
 *|     	   			|   	|			      (Pin 15) GPIO|-->(LED 2)
 *|                     |       |              			       |
 *|      PB.8 (I2C1_SCL)|------>|SCL (Pin 1)   	  (Pin 14) GPIO|-->(LED 1)
 *|                     |       |              				   |
 *|      PB.9 (I2C1_SDA)|------>|SDA (Pin 2)   	  (Pin 13) GPIO|<--(SWITCH 4)
 *|                     |       |              			       |
 *|     	 PC.6 (GPIO)|<------|INT (Pin 8)      (Pin 12) GPIO|<--(SWITCH 3)
 *|                     |       |              				   |
 *|       		     GND|------>|VSS (Pin 9)      (Pin 11) GPIO|<--(SWITCH 2)
 *|                     |   |   |              			 	   |
 *|     			    |   --->|A2 (Pin 3)	 	  (Pin 10) GPIO|<--(SWITCH 1)
 *|                     |   |   |              				   |
 *|     			    |   --->|A1 (Pin 4)	 				   |
 *|                     |   |   |              				   |
 *|     			    |   --->|A0 (Pin 5)	 				   |
 *|                     |       |              				   |
 *|_____________________|       |______________________________|
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

int main(){

	//Configure GPIOs ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitGPIOA();
	InitGPIOC();
	InitGPIOB();

	//Configure Timers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	bitset(RCC->CFGR, 0);     // Use HSI16 as SYSCLK

	//Configure Button Interrupt ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitInt();

	//Configure I2C ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitI2C();

	bool Addr_Found = I2C_CheckAddr(I2C_SLAVE_ADDR);
	if (Addr_Found){
		bitclear(GPIOA->ODR, 9);	//Red LED - Off
	} else {
		bitset(GPIOA->ODR, 9);		//Red LED - On
	}

	//Configure I/O Expander ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitIOExp();


	//Infinite Loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	while(1);

}

//Function Declarations ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void InitGPIOA(){
	/**
	  * @brief	A function to enable/configure GPIOA.
	  * @param 	N/A.
	  * @retval	N/A
	*/

	bitset(RCC->AHB2ENR, 0);	// Enable Clock to GPIOA

	//Set GPIO Port A, Pin 9 (RED LED) to "General purpose output mode".
	bitset(GPIOA->MODER, 18);
	bitclear(GPIOA->MODER, 19);
}

void InitGPIOC(){
	/**
	  * @brief	A function to enable/configure GPIOC.
	  * @param 	None
	  * @retval	None
	*/

	bitset(RCC->AHB2ENR, 2);	// Enable Clock to GPIOC

	//Set GPIO Port C, Pin 6 to "Input mode".
	bitclear(GPIOC->MODER, 13);
	bitclear(GPIOC->MODER, 12);
}

void InitGPIOB(){
	/**
	  * @brief	A function to enable/configure GPIOB.
	  * @param 	None
	  * @retval	None
	*/

	bitset(RCC->AHB2ENR, 1);	// Enable Clock to GPIOB

	//Set GPIO Port B, Pin 8 to "Alternate function mode".
	bitset(GPIOB->MODER, 17);
	bitclear(GPIOB->MODER, 16);
	bitclear(GPIOB->AFR[1], 3);  // Programming 0b0100 (AF4 = I2C1_SCL)
	bitset(GPIOB->AFR[1], 2);
	bitclear(GPIOB->AFR[1], 1);
	bitclear(GPIOB->AFR[1], 0);
	bitset(GPIOB->OSPEEDR, 17); // Set OSPEED to 'b11 (High-High Speed)
	bitset(GPIOB->OSPEEDR, 16);
	bitset(GPIOB->OTYPER, 8);	// Set output to "open-drain" mode.
	bitclear(GPIOB->PUPDR, 17);	// Programming 0b01 (Pull up)
	bitset(GPIOB->PUPDR, 16);


	//Set GPIO Port B, Pin 9 to "Alternate function mode".
	bitset(GPIOB->MODER, 19);
	bitclear(GPIOB->MODER, 18);
	bitclear(GPIOB->AFR[1], 7);  // Programming 0b0100 (AF4 = I2C1_SDA)
	bitset(GPIOB->AFR[1], 6);
	bitclear(GPIOB->AFR[1], 5);
	bitclear(GPIOB->AFR[1], 4);
	bitset(GPIOB->OSPEEDR, 19); // Set OSPEED to 'b11 (High-High Speed)
	bitset(GPIOB->OSPEEDR, 18);
	bitset(GPIOB->OTYPER, 9);	// Set output to "open-drain" mode.
	bitclear(GPIOB->PUPDR, 19);	// Programming 0b01 (Pull up)
	bitset(GPIOB->PUPDR, 18);
}

void InitI2C(){
	/**
	 * @brief Initializes the I2C peripheral with custom timing and settings.
	  * @param 	None
	  * @retval	None
	 */

	bitset(RCC->APB1ENR1, 21);  // Enable Clock to I2C1

	bitset(RCC->APB1RSTR1, 21);  // Reset I2C1
	bitclear(RCC->APB1RSTR1, 21);  // Reset I2C1

	bitclear(I2C1->TIMINGR, 31); // Set PRESC = 3 (0b0011)
	bitclear(I2C1->TIMINGR, 30);
	bitset(I2C1->TIMINGR, 29);
	bitset(I2C1->TIMINGR, 28);

	bitclear(I2C1->TIMINGR, 23); // Set SCLDEL = 4 (0b0100)
	bitset(I2C1->TIMINGR, 22);
	bitclear(I2C1->TIMINGR, 21);
	bitclear(I2C1->TIMINGR, 20);

	bitclear(I2C1->TIMINGR, 19);	// Set SDADEL = 2 (0b0010)
	bitclear(I2C1->TIMINGR, 18);
	bitset(I2C1->TIMINGR, 17);
	bitclear(I2C1->TIMINGR, 16);

	bitclear(I2C1->TIMINGR, 15); // Set SCLH = 15 (0b00001111)
	bitclear(I2C1->TIMINGR, 14);
	bitclear(I2C1->TIMINGR, 13);
	bitclear(I2C1->TIMINGR, 12);
	bitset(I2C1->TIMINGR, 11);
	bitset(I2C1->TIMINGR, 10);
	bitset(I2C1->TIMINGR, 9);
	bitset(I2C1->TIMINGR, 8);

	bitclear(I2C1->TIMINGR, 7); // Set SCLL = 19 (0b00010011)
	bitclear(I2C1->TIMINGR, 6);
	bitclear(I2C1->TIMINGR, 5);
	bitset(I2C1->TIMINGR, 4);
	bitclear(I2C1->TIMINGR, 3);
	bitclear(I2C1->TIMINGR, 2);
	bitset(I2C1->TIMINGR, 1);
	bitset(I2C1->TIMINGR, 0);

	bitset(I2C1->CR1, 0); // Enable Peripheral

}

void InitInt(){
	/**
	  * @brief Initializes interrupt configuration.
	  * @param None
	  * @retval None
	*/
	RCC->APB2ENR   |= 1;  			// Enable Clock to SYSCFG & EXTI

	bitset(EXTI->EXTICR[1], 17);  	// Select PC6
	bitset(EXTI->RTSR1,6);     		// Trigger on rising edge of PC6
	bitset(EXTI->IMR1,6);     		// Interrupt mask disable for PC6

	NVIC_SetPriority(EXTI6_IRQn, 0); // 0 is higher than 1 (3 bit priority)

	NVIC_EnableIRQ(EXTI6_IRQn);		// Enable Interrupt

}

void I2C_Write(uint8_t slv_addr, uint8_t* data, uint8_t data_length ){
	/**
	 * @brief Writes data to a device over I2C.
	 * @param slv_addr The 7-bit slave address of the device to write to.
	 * @param data Pointer to the data array to be written.
	 * @param data_length The number of bytes to write.
	 * @retval None
	*/

	while (bitcheck(I2C1->ISR,15) != 0);		// Wait for not busy

	I2C1->CR2 = 0;								// Clear CR2
	bitset(I2C1->CR2, 25); 						// Enable Automatic End Mode

	I2C1->CR2 |= slv_addr<<1; 					// Set slave address.
	bitclear(I2C1->CR2,10); 					// Master request a write transfer.
	I2C1->CR2 |= ((data_length) <<16); 			// Set number of bytes.
	bitset(I2C1->CR2, 13); 						// Start write.

    for (uint8_t i = 0; i < data_length; i += 1) {
        while (bitcheck(I2C1->ISR, 0) != 1); 	// Wait for the transmit buffer to be empty.
        I2C1->TXDR = data[i];
    }

    while (bitcheck(I2C1->ISR, 0) != 1); 		// Wait for the transmit buffer to be empty.
}

uint8_t* I2C_Read(uint8_t slv_addr, uint8_t reg_addr, uint8_t data_length ){
	/**
	 * @brief Reads data from a device over I2C.
	 *
	 * @param slv_addr The 7-bit slave address of the device to read from.
	 * @param reg_addr The register address to start reading from.
	 * @param data_length The number of bytes to read.
	 * @return Pointer to the buffer containing the read data.
	*/

    uint8_t* buffer = (uint8_t*)malloc(data_length * sizeof(uint8_t)); // Allocate memory for the read data.


    uint8_t data_array[1] = {reg_addr};	// Create array to give to the write function.
    I2C_Write(slv_addr, data_array, 1);


    //Clear any lingering data.
    while (bitcheck(I2C1->ISR, 2) == 1){
    	(void)I2C1->RXDR;
    }

	I2C1->CR2 = 0;					// Clear CR2
	bitset(I2C1->CR2, 25); 			// Enable Automatic End Mode

	I2C1->CR2 |= slv_addr<<1; 		// Set slave address.
	bitset(I2C1->CR2,10); 			// Master request a read transfer.
	I2C1->CR2 |= (data_length) <<16;// Set number of bytes.
	bitset(I2C1->CR2, 13); 			// Start send.

    for (uint8_t i = 0; i < data_length; i += 1) {
        while (bitcheck(I2C1->ISR, 2) != 1); 	// Wait for the receive buffer to not be empty.
        buffer[i] = I2C1->RXDR;
    }

    return buffer;

}

bool I2C_CheckAddr(uint8_t slv_addr) {
	/**
	 * @brief Checks if a device responds to a given I2C address.
	 *
	 * @param slv_addr The 7-bit slave address of the device to check.
	 * @return true if the device acknowledges the address, false otherwise.
	*/

	while (bitcheck(I2C1->ISR,15) != 0);	// Wait for not busy

	bitset(I2C1->ICR,4);	// Clear the NACKF

	I2C1->CR2 = 0;			// Clear CR2
	bitset(I2C1->CR2, 25); 	// Enable Automatic End Mode

	I2C1->CR2 |= slv_addr<<1; 	// Set slave address.
	bitclear(I2C1->CR2,10); 	// Master request a write transfer.
	I2C1->CR2 |= (0x00) <<16; 	// Set number of bytes.
	bitset(I2C1->CR2, 13); 		// Start send.

	while (bitcheck(I2C1->ISR, 0) != 1); 	// Wait for the transmit buffer to be empty.

	while (bitcheck(I2C1->ISR,15) != 0);	// Wait for not busy

    if (bitcheck(I2C1->ISR, 4) != 1){		// Check if ACK received (NACK will be false).
    	return true;
    } else {
    	return false;
    }
}

void InitIOExp(){
	/**
	 * @brief Initializes the I/O Expander via I2C.
	  * @param None
	  * @retval None
	*/

    uint8_t data_array[2] = {0x00,0x0F};//Set IO direction. {register,data}
	I2C_Write(I2C_SLAVE_ADDR,data_array,2); //I2C_Write(uint8_t slv_addr, uint8_t* data, uint8_t data_length )

	data_array[0] = 0x01; data_array[1] = 0x0F;	//Set IO polarity. {register,data}
	I2C_Write(I2C_SLAVE_ADDR,data_array,2); //I2C_Write(uint8_t slv_addr, uint8_t* data, uint8_t data_length )

	data_array[0] = 0x02; data_array[1] = 0x0F; //Set IO interrupt enables. {register,data}
	I2C_Write(I2C_SLAVE_ADDR,data_array,2); //I2C_Write(uint8_t slv_addr, uint8_t* data, uint8_t data_length )

	data_array[0] = 0x03; data_array[1] = 0x00; //Set IO interrupt default values. {register,data}
	I2C_Write(I2C_SLAVE_ADDR,data_array,2); //I2C_Write(uint8_t slv_addr, uint8_t* data, uint8_t data_length )

	data_array[0] = 0x04; data_array[1] = 0x00; //Set IO interrupt control types. {register,data}
	I2C_Write(I2C_SLAVE_ADDR,data_array,2); //I2C_Write(uint8_t slv_addr, uint8_t* data, uint8_t data_length )

	data_array[0] = 0x05; data_array[1] = 0b00000010; //Set up control register (— — SEQOP DISSLW HAEN ODR INTPOL —). {register,data}
	I2C_Write(I2C_SLAVE_ADDR,data_array,2); //I2C_Write(uint8_t slv_addr, uint8_t* data, uint8_t data_length )

	data_array[0] = 0x06; data_array[1] = 0x0F; //Set IO pull-up resistors. {register,data}
	I2C_Write(I2C_SLAVE_ADDR,data_array,2); //I2C_Write(uint8_t slv_addr, uint8_t* data, uint8_t data_length )

	uint8_t* IO_State = I2C_Read(I2C_SLAVE_ADDR, 0x09, 1); // Read I/O states.

	data_array[0] = 0x09; data_array[1] = IO_State[0]<<4; //Set output states. {register,data}
	I2C_Write(I2C_SLAVE_ADDR,data_array,2); //I2C_Write(uint8_t slv_addr, uint8_t* data, uint8_t data_length )
}

void EXTI6_IRQHandler(){
	/**
	  * @brief Handler for EXTI 6 interrupts.
	  * @param None
	  * @retval None
	*/

	Delay_ms(100);	// Give switch time to de-bounce before clearing interrupt on the I/O Expander.

	while (bitcheck(GPIOC->IDR,6)==1){
		(void)I2C_Read(I2C_SLAVE_ADDR, 0x08, 1);	// Read interrupt captured I/O states to clear interrupt.
	}
	uint8_t* IO_State = I2C_Read(I2C_SLAVE_ADDR, 0x09, 1); // Read current I/O states.

	uint8_t data_array[2] = {0x09,IO_State[0]<<4}; 	//Set output states. {register,data}
	I2C_Write(I2C_SLAVE_ADDR,data_array,2); 		//I2C_Write(uint8_t slv_addr, uint8_t* data, uint8_t data_length )

	bitset(EXTI->RPR1, 6);// Clear flag
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

void Error_Handler(void) {
}
