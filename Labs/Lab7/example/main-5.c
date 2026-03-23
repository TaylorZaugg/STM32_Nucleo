// Includes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "main.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"

// Some helper macros ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1   ) //Checks the bit number <idx> -- 0 means clear; !0 means set.
#define ConcatenateWords(word1, word2) (((uint16_t)(word1) << 8) | (word2))//concat two 8bit words to 1 16bit.



//Pre-Declarations of functions. Full declarations are after main(). ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void InitGPIOC();
void InitGPIOE();
void InitGPIOG();
void InitBtn();
void Delay_ms(uint32_t val);
void SendLPUARTResp(uint8_t value);
void InitSPI();
void WriteSPIPROMStatus(uint8_t data);
uint8_t ReadSPIPROMStatus();
void WriteSPIPROM(uint8_t addr, uint8_t* data, uint8_t length);
uint8_t ReadSPIPROM(uint8_t addr);
void SendSPI( uint8_t inst, uint8_t addr, uint8_t* data, uint8_t length, bool en_addr, bool en_data);
void InitLPUART1(uint32_t baud, uint8_t stop, uint8_t data_len, bool parity_en, bool parity_odd);

// Global Constants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static const uint8_t SPIPROM_READ     = 0b00000011;    // Command to read data from SPI EEPROM.
static const uint8_t SPIPROM_WRITE    = 0b00000010;    // Command to write data to SPI EEPROM.
// static const uint8_t SPIPROM_WRDI  = 0b00000100;    // Command to reset the write enable latch (commented out).
static const uint8_t SPIPROM_WREN     = 0b00000110;    // Command to set the write enable latch.
static const uint8_t SPIPROM_RDSR     = 0b00000101;    // Command to read the status register.
static const uint8_t SPIPROM_WRSR     = 0b00000001;    // Command to write to the status register.
#define BUFFER_SIZE                   196              // Size of the buffer used for RX and TX operations.
#define EEPROM_MAX_ADDR               196              // Maximum EEPROM addressable space.
#define PAGE_SIZE                     16               // EEPROM page size in bytes.


//Global Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
char        resp_str[32];          // UART response string buffer.
uint8_t     resp_inc = 0;          // Index for current position in resp_str for UART transmission.
bool        resp_rdy = 0;          // Flag for response readiness for UART (true = ready).
bool        resp_act = 0;          // Flag for active response transmission via UART.
uint8_t     rd_PTR=0;              // Read pointer for EEPROM data retrieval.
uint8_t     PTR=0;                 // Write pointer to track next free EEPROM memory address.

bool write_complete = false;

uint32_t    timeoutCounter = 0;    // Counter for managing operation timeouts.
uint8_t     rxBuffer[BUFFER_SIZE]; // Buffer to store received data before processing or EEPROM storage.
uint8_t     bufferIndex = 0;       // Index to track the current position




/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * -LPUART
   BAUD = 115200
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * -WIRING
	      NUCLEO				     EEPROM
_________________________        _______________
|                        |       |             |
|     		          3V3|------>|VCC (Pin 8)  |
|                        |   |   |             |
|     					 |   --->|~HOLD (Pin 7)|
|                        |   |   |             |
|     	   				 |   --->|~WP (Pin 3)  |
|                        |       |             |
|     	     PE.12 (GPIO)|------>|~CS (Pin 1)  |
|                        |       |             |
|     	 PE.13 (SPI1_SCK)|------>|SCK (Pin 6)  |
|                        |       |             |
|     	PE.14 (SPI1_MISO)|<------|SO (Pin 2)   |
|                        |       |             |
|     	PE.15 (SPI1_MOSI)|------>|SI (Pin 5)   |
|                        |       |             |
|     				  GND|------>|VSS (Pin 4)  |
|                        |       |             |
|________________________|       |_____________|
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/




int main(){
	//Local Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	uint8_t rec_cnt = 0; 	// Number of bytes received in last LPUART transmission.

	//Configure GPIOs ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitGPIOC();
	InitGPIOE();
	InitGPIOG();

	//Configure Timers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	bitset(RCC->CFGR, 0);  	// Use HSI16 as SYSCLK

	//Configure Button Interrupt ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitBtn();

	//Configure COMMs ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitSPI();
	InitLPUART1(115200,1,8,false,false);	//InitLPUART1(baud, stop, data_len, parity_en, parity_odd)


	//Infinite Loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	while (1) {

		if (bufferIndex > 0) {
			// Scan for newline
			for (uint8_t i = 0; i < bufferIndex; i++) {
				if (rxBuffer[i] == '\n') {
					rec_cnt = i + 1; // Include the newline

					uint8_t currentPageEnd = PAGE_SIZE - (PTR % PAGE_SIZE);
					uint8_t bytesToWrite = (rec_cnt <= currentPageEnd) ? rec_cnt : currentPageEnd;

					if ((bytesToWrite % 2 != 0) && (bytesToWrite == currentPageEnd)) {
						uint8_t firstByteOfThisPage = ReadSPIPROM(PTR - (PTR % PAGE_SIZE));
						uint8_t temp_length = bytesToWrite + 1;
						uint8_t temp_buffer[temp_length];
						memcpy(temp_buffer, rxBuffer, bytesToWrite);
						temp_buffer[temp_length - 1] = firstByteOfThisPage;
						WriteSPIPROM(PTR, temp_buffer, temp_length);
					} else {
						WriteSPIPROM(PTR, rxBuffer, bytesToWrite);
					}

					// Shift remaining data (if any)
					if (bufferIndex > bytesToWrite) {
						memmove(rxBuffer, rxBuffer + bytesToWrite, bufferIndex - bytesToWrite);
					}
					bufferIndex -= bytesToWrite;

					// Update EEPROM pointer
					PTR += bytesToWrite;
					if (PTR >= EEPROM_MAX_ADDR) PTR = 0;

					write_complete = true;
					break; // Only handle one message per loop
				}
			}

			if (write_complete) {
				SendLPUARTResp(rec_cnt); // Send response exactly once
			}
		}
	}


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

	//Set GPIO Port E, Pin 12 to 'b01 "General purpose output mode".
	bitclear(GPIOE->MODER, 25);
	bitset(GPIOE->MODER, 24);

	//Set GPIO Port E, Pin 13 to "Alternate Function".
	bitset(GPIOE->MODER, 27);
	bitclear(GPIOE->MODER, 26);
	bitclear(GPIOE->AFR[1], 23);  // Programming 'b0101 (AF5 = SPI1_SCK)
	bitset(GPIOE->AFR[1], 22);
	bitclear(GPIOE->AFR[1], 21);
	bitset(GPIOE->AFR[1], 20);
	bitset(GPIOE->OSPEEDR, 27); // Set OSPEED to 'b11 (High-High Speed)
	bitset(GPIOE->OSPEEDR, 26);


	//Set GPIO Port E, Pin 14 to "Alternate Function".
	bitset(GPIOE->MODER, 29);
	bitclear(GPIOE->MODER, 28);
	bitclear(GPIOE->AFR[1], 27);  // Programming 'b0101 (AF5 = SPI1_MISO)
	bitset(GPIOE->AFR[1], 26);
	bitclear(GPIOE->AFR[1], 25);
	bitset(GPIOE->AFR[1], 24);
	bitset(GPIOE->OSPEEDR, 29); // Set OSPEED to 'b11 (High-High Speed)
	bitset(GPIOE->OSPEEDR, 28);

	//Set GPIO Port E, Pin 15 to "Alternate Function".
	bitset(GPIOE->MODER, 31);
	bitclear(GPIOE->MODER, 30);
	bitclear(GPIOE->AFR[1], 31);  // Programming 'b0101 (AF5 = SPI1_MOSI)
	bitset(GPIOE->AFR[1], 30);
	bitclear(GPIOE->AFR[1], 29);
	bitset(GPIOE->AFR[1], 28);
	bitset(GPIOE->OSPEEDR, 31); // Set OSPEED to 'b11 (High-High Speed)
	bitset(GPIOE->OSPEEDR, 30);

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

void SendLPUARTResp(uint8_t value) {
	/**
	 * @brief  Sends a response over LPUART1 indicating the number of bytes received.
	 * @param  value The number of bytes received to be included in the response message.
	 * @retval None
	 */

    sprintf(resp_str, "Received %d Bytes\r\n", value); // Convert the decimal value to a string in ASCII format

    resp_act = 1;
    resp_inc = 0;
	bitset(LPUART1->CR1,6);// Enable Transmit Data Empty Interrupt (TXEIE)
	write_complete = false;
}

void InitSPI(){
	/**
	 * @brief  Initializes the SPI interface with predefined settings.
	 * @param  None
	 * @retval None
	 */

	bitset(RCC->APB2ENR,12);// Enable clock to SPI1.

	bitclear(SPI1->CR1,5); // Set Baud Rate Control = 'b001 (fPCLK/4). This should be 16MHz/4 = 4MHz (The max for the EEPROM is 5MHz).
	bitclear(SPI1->CR1,4);
	bitset(SPI1->CR1,3);

	bitset(SPI1->CR1,2); // Set to master.

	bitclear(SPI1->CR1,1); // Set clock polarity to 0.
	bitclear(SPI1->CR1,0); // Set clock phase to 0.

	bitset(SPI1->CR2,11); // Set data size to 'b1111 (16-bit)
	bitset(SPI1->CR2,10);
	bitset(SPI1->CR2,9);
	bitset(SPI1->CR2,8);

	bitset(SPI1->CR1,9); // Enable software slave management.
	bitset(SPI1->CR1, 8); // SSI: Internal slave select set.

    bitset(GPIOE->ODR, 12); // De-select the slave device
	bitset(SPI1->CR1,6); // Enable SPI1.
}


void WriteSPIPROMStatus(uint8_t data){
	/**
	  * @brief  Writes to the status register of the SPI EEPROM.
	  * @param  data  The new value to be written to the EEPROM's status register.
	  * @retval None
	  */

    // Create an array to hold the data
    uint8_t data_array[1] = {data};

    // Pass the array to SendSPI
    SendSPI(SPIPROM_WRSR, 0x00, data_array, 1, 0, 1); //( uint8_t inst, uint8_t addr, uint8_t* data, uint8_t length, bool en_addr, bool en_data)
}

uint8_t ReadSPIPROMStatus(){
	/**
	  * @brief  Reads the status register of the SPI EEPROM.
	  * @param  None
	  * @retval The status register value of the SPI EEPROM.
	  */

	SendSPI( SPIPROM_RDSR, 0x00, 0x00, 0x00, 0, 0); //( uint8_t inst, uint8_t addr, uint8_t* data, uint8_t length, bool en_addr, bool en_data)
    while (bitcheck(SPI1->SR,0) != 1){};			// Wait for receive buffer not empty.
	return (uint8_t) (SPI1->DR);					// Return status register value.
}

void WriteSPIPROM(uint8_t addr, uint8_t* data, uint8_t length){
	/**
	  * @brief  Writes data to a specified address in the SPI EEPROM.
	  *         Before writing, it ensures the EEPROM is not write-protected by reading the
	  *         status register. It then enables writing, waits for the EEPROM to be ready,
	  *         and writes the data to the specified address.
	  * @param  addr  The starting address in the EEPROM where the data will be written.
	  * @param  data  Pointer to the data array to be written.
	  * @param  length The number of bytes to write.
	  * @retval None
	  */

	uint8_t stat_reg;

	do {
		stat_reg = ReadSPIPROMStatus();
	} while (stat_reg & 0x01); // Wait while write in progress flag is set

	SendSPI( SPIPROM_WREN, 0x00, 0x00, 0x00, 0, 0); 		//( uint8_t inst, uint8_t addr, uint8_t* data, uint8_t length, bool en_addr, bool en_data)
	(void)SPI1->DR; 										// Clear RX FIFO from write enable instruction.

	SendSPI(SPIPROM_WRITE, addr, data, length, true, true); //( uint8_t inst, uint8_t addr, uint8_t* data, uint8_t length, bool en_addr, bool en_data)
}

uint8_t ReadSPIPROM(uint8_t addr){
	/**
	  * @brief  Reads data from a specified address in the SPI EEPROM.
	  * @param  addr  The address in the EEPROM from which the data will be read.
	  * @retval The data byte read from the specified EEPROM address.
	  */

	SendSPI( SPIPROM_READ, addr, 0x00, 0x00, 1, 0); //( uint8_t inst, uint8_t addr, uint8_t* data, uint8_t length, bool en_addr, bool en_data)
	while (bitcheck(SPI1->SR,0) != 1);				// Wait for receive buffer not empty.
	return (uint8_t) (SPI1->DR>>8);					// Return value.
}

void SendSPI( uint8_t inst, uint8_t addr, uint8_t* data, uint8_t length, bool en_addr, bool en_data){
	/**
	  * @brief  Generic function to send commands and data over SPI to the EEPROM.
	  * @param  inst    The SPI instruction byte.
	  * @param  addr    The EEPROM memory address for read/write operations (if applicable).
	  * @param  data    Pointer to the data array for write operations (if applicable).
	  * @param  length  The length of the data to send/receive.
	  * @param  en_addr A boolean to enable/disable sending of the address.
	  * @param  en_data A boolean to enable/disable sending of the data.
	  * @retval None
	  */

    uint8_t clean_cnt = 0;
    while (bitcheck(SPI1->SR,1) != 1);				// Wait for the transmit buffer to be empty.

    bitclear(GPIOE->ODR, 12); 						// Select the slave device by pulling its CS (Chip Select) low.

    if (en_addr && en_data){
        SPI1->DR = ConcatenateWords(inst, addr); 	// Send instruction and address if both are enabled.

        for (uint8_t i = 0; i < length; i += 2) {
            uint16_t two_bytes = 0x00; 				// Initialize with zeroes to build the data word.
            if (i + 1 < length) {
                two_bytes = ConcatenateWords(data[i], data[i + 1]); // Concatenate two bytes if possible.
            } else {
                two_bytes = data[i] << 8; 							// Only one byte left, shift it to the MSB.
            }

            SPI1->DR = two_bytes; 					// Send the data word.
            while (bitcheck(SPI1->SR, 1) != 1); 	// Wait for the transmit buffer to be empty again.
        }
        clean_cnt = 2; 								// Set number of cleanup reads needed for RXNE flag.

    } else if (en_addr) {
        SPI1->DR = ConcatenateWords(inst, addr); 	// Send instruction and address, no data.
        SPI1->DR = 0x0000; 							// Dummy write to ensure clock pulses for address read.
        clean_cnt = 1; 								// Cleanup read needed for RXNE flag.

    } else if (en_data) {
        SPI1->DR = ConcatenateWords(inst, 0x00); 	// Send instruction, no address.
        clean_cnt = 1; 								// Cleanup read needed for RXNE flag.

    } else {
        SPI1->DR = ConcatenateWords(inst, 0x00); // Send only the instruction, no address or data.
        // Cleanup will be done outside this function for this case. This is because the write enable (cleanup needed) and ReadSPIPROMStatus (no cleanup) will apply to this.
    }

    while (bitcheck(SPI1->SR,1) != 1);	// Wait again for the transmit buffer to be empty.
    while (bitcheck(SPI1->SR,7) != 0); 	// Wait until SPI is not busy (BSY flag is cleared).
    bitset(GPIOE->ODR, 12); 			// De-select the slave device by pulling CS high.

    while (clean_cnt > 0){
        (void)SPI1->DR; 				// Perform cleanup reads to clear RXNE flag.
        clean_cnt -= 1;
    }

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
	if (PTR > 0){ 				// Only react when there is something in the EEPROM to print.
		rd_PTR = 0;
		bitset(LPUART1->CR1,6);	// Enable Transmit Data Empty Interrupt (TXEIE)
	}
	bitset(EXTI->RPR1, 13);		// Clear flag
}


void LPUART1_IRQHandler(){
	/**
	  * @brief Handler for LPUART1 interrupts.
	  * @param None
	  * @retval None
	*/

	// Check if the Receive Data Not Empty (RXNE) interrupt flag is set.
    if (bitcheck(LPUART1->ISR, 5) != 0){
        uint8_t read_data = (uint8_t)(LPUART1->RDR); 	// Read the received data from the receiver data register.
        rxBuffer[bufferIndex++] = read_data;          	// Store the received data in the rxBuffer and increment buffer index.

        timeoutCounter = 160000; 						// Reset the timeout counter to its initial value to know when RX transmission is complete.
    }
    else {
        while (bitcheck(LPUART1->ISR, 7) != 1); // Wait for the Transmit Data Register Empty (TXE) flag before sending data.

        // Check if a response is actively being sent.
        if (resp_act){
            LPUART1->TDR = resp_str[resp_inc]; 	// Load the next byte of the response string into the Transmit Data Register (TDR).
            resp_inc += 1; 						// Increment the index for the next byte of the response string.

            // Check if the end of the response string is reached.
            if (resp_str[resp_inc] == '\0'){
                bitclear(LPUART1->CR1,6); 		// Disable Transmit Data Empty Interrupt (TXEIE) as transmission is complete.
                resp_act = 0; 					// Clear the flag indicating active response transmission.
            }
        } else {
            // If not responding to a previous command, read and transmit data from the SPI EEPROM.
            LPUART1->TDR = ReadSPIPROM(rd_PTR); // Transmit data read from SPI EEPROM.
            rd_PTR += 1; 						// Increment the read pointer.

            // Check if all data has been transmitted.
            if (rd_PTR == PTR) {
                bitclear(LPUART1->CR1,6); 	// Disable Transmit Data Empty Interrupt (TXEIE) as all data has been transmitted.
                rd_PTR = 0; 				// Reset the read pointer.
                PTR = 0;    				// Reset the write pointer, preparing for the next set of data.
            }
        }
    }
}

void Error_Handler(void) {
}
