/* Taylor Zaugg
 * Lab 8 - I2C
 * 2025-04-10
 *
 * This program initializes a GPIO expander using I2C protocol.
 * The expander is controlling 4 LED's based on 4 switches and their communication to the MCU through I2C
 * (Yes just running power from the switch to the LED would do the same thing, but that wouldn't give practice with I2C)
 * Device used: Microchip I2C IO expander MCP23008 18 DIP
 *
 * Expander Pins connection to MCU:
 * MCP23008                Pin           MCU Connection pin
 *     SLC                  1           I2C1_SLC, GPIOB pin 8
 *     SDA                  2           I2C1_SDA, GPIOB pin 9
 *  IT- Interrupt Output    8           GPIOC, pin 6
 *
 * I2C Address of Device: 0b0100000
 *
 * Device Register Addresses:
 *
 * Address:		Access
 * 0x00			IODIR	- I/O Direction Register
 * 0x01			IPOL 	- Input Polarity Port Register
 * 0x02			GPINTEN	- Interrupt-on-change pins
 * 0x03			DEFVAL	- Default Value Register
 * 0x04			INTCON	- Interrupt-onChange Control Register
 * 0x05			IOCON	- I/O Expander Configuration Register
 * 0x06			GPPU	- GPIO Pull-up Resistor Register
 * 0x07			INTF	- Interrupt Flag Register
 * 0x08			INTCAP (read-only) - Interrupt Captured Value for Port Register
 * 0x09			GPIO	- General Purpose I/O Port Register
 * 0x0a			OLAT	- Output Latch Register 0
 *
 * I2C Control Byte Format
 *
 * Start  | 0 | 1 | 0 | 0 | 0 | 0 | 0 | R/W |  ACK  | A7 | A6 | A5 | A4 | A3 | A2 | A1 | A0 |  ACK  |
 *                 CONTROL BYTE                                REGISTER ADDRESS
 *
 *
 *
 */

// I2C Initialization function. This function will
// DONE 1- Enable clocks (I2C and GPIO)
// DONE 2- Enable GPIO in AF and Open-drain mode. (OTYPER)
// DONE 3- If there is no external pull-up resistor, you may enable the MCU internal pull-up (PUPDR)
// DONE 4- Reset I2C module, so that I2C state machine is at idle state
// DONE 5- Program I2C Timing based on targeted speed (check page 1522 in Reference Manual) (TIMINGR)
// DONE 6- Enable I2C peripheral


// I2C Write Operation (Page 1517):
// 1- Make sure that the I2C peripheral is not busy
// 2- Write slave address (SADDR)
// 3- Write the type of transaction (RD_WRN=0)
// 4- Configure auto-end if needed (AUTOEND)
// 5- Configure number of bytes to be sent (NBYTES)
// 6- Issue the START condition
// 7- Wait until flag TXIS is asserted or TXE
// 8- Write data byte to be sent to I2C_TXDR
// 9- Check that the data reg is empty (TXIS or TXE)
// 10- Issue the STOP condition if AUTOEND was set to 0; nothing if it was set to 1


// I2C Read Operation (Page 1521):
// 1- Make sure that the peripheral is not busy
// 2- Write slave address (SADDR)
// 3- Write the type of transaction (RD_WRN=1)
// 4- Configure auto-end if needed (AUTOEND)
// 5- Configure number of bytes to be sent (NBYTES)
// 6- Issue the START condition
// 7- Check that there is a slave with a matching address (ADDR)
// 8- Issue ACK or NACK
// 9- Wait until RXNE
// 10- Read I2C_RXDR
// 11- Issue the STOP condition if AUTOEND was set to 0; nothing if it was set to 1

#include "stm32l552xx.h"
#include "stdio.h"


// Some helper macros ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1   ) //Checks the bit number <idx> -- 0 means clear; !0 means set.

#define ConcatenateWords(word1, word2) (((uint16_t)(word1) << 8) | (word2))//concat two 8bit words to 1 16bit.

void InitI2C();
void WriteOperation(char address, char data);
char ReadOperation();
uint8_t reverse_bits(uint8_t byte);

static unsigned int EEPROMADDRESS = 0b0100000; //0x20 7, 7 bit address

int main(){

	InitI2C();
	char IODIR = 0x00;
	char GPIO = 0x09;
	// 1 = input, 0 = output
	//	char IODIRconfig = 0b11110000;
	char IODIRconfig = 0x0f;

	WriteOperation(IODIR, IODIRconfig);


	char LEDsON = 0b11111111;

	WriteOperation(GPIO, LEDsON);


	while(1){
		for(int i =0 ; i < 10000; i++);
		WriteOperation(GPIO, LEDsON);
	}

}

char ReadOperation(){
	// I2C Read Operation (Page 1521):
	// 1- Make sure that the peripheral is not busy
	while(!bitcheck(I2C1->ISR, 15)); //Wait for busy to be 0

	// 2- Write slave address (SADDR)
	// I2C1->CR2 bits 9:0

	//NOT bits 9:0
	//7 bit address uses bits 7:1

	//char reverseEEaddr = reverse_bits(EEPROMADDRESS);

	I2C1->CR2 |= EEPROMADDRESS <<1;


	// 3- Write the type of transaction (RD_WRN=1)
	// I2C->CR2 RD_WRN bit 10
	// 0: Master request write
	// 1: Master request read
	bitset(I2C1->CR2, 10);

	// 4- Configure auto-end if needed (AUTOEND)
	// I2C1->CR2 bit 25
	bitset(I2C1->CR2, 25); //TURN AUTOEND ON

	// 5- Configure number of bytes to be sent (NBYTES)
	// I2C1->CR2 NBYTES bits 23:16
	//Should send 1 byte? 8 bits? 4 bits for switches, 4 bits for LED's?
	//TODO Double check this in operation

	I2C1->CR2 |= 1 << 16; // 1 byte mode

	// 6- Issue the START condition
	// 6- Issue the START condition
	bitset(I2C1->CR2, 13); // START bit ON

	// 7- Check that there is a slave with a matching address (ADDR)
	if(bitcheck(I2C1->ISR,3)){
		// 8- Issue ACK or NACK
		bitclear(I2C1->CR2, 15); //NACK bit = CR2 bit 15 clear = ACK bit
	}
	else{
		bitset(I2C1->CR2, 15); //NACK bit = CR2 bit 15. set = NACK bit
	}

	// 9- Wait until RXNE
	while(bitcheck(I2C1->ISR, 2) == 0 );

	// 10- Read I2C_RXDR
	char buffer = I2C1->RXDR;

	// 11- Issue the STOP condition if AUTOEND was set to 0; nothing if it was set to 1

	return buffer;
}


void WriteOperation(char address, char data){
	// I2C Write Operation (Page 1517):
	// 1- Make sure that the I2C peripheral is not busy========================
	// I2C1->ISR BUSY flag. Bit 15. Indicates communication on the bus. Cleared by hardware when STOP condition, or when PE=0
	while(bitcheck(I2C1->ISR, 15) == 1); //Wait for busy to be 0

	// 2- Write slave address (SADDR)==========================================
	// I2C1->CR2 bits 9:0
	//NOT bits 9:0
	//7 bit address uses bits 7:1

	//char reverseEEaddr = reverse_bits(EEPROMADDRESS);

	I2C1->CR2 |= EEPROMADDRESS <<1;
	//I2C1->CR2 |= reverseEEaddr <<1;

	// 3- Write the type of transaction (RD_WRN=0)=============================
	// I2C->CR2 RD_WRN bit 10
	// 0: Master request write
	// 1: Master request read
	bitclear(I2C1->CR2, 10);

	// 4- Configure auto-end if needed (AUTOEND) TODO use? or No? Set on for now
	// I2C1->CR2 bit 25
	bitset(I2C1->CR2, 25); //TURN AUTOEND ON

	// 5- Configure number of bytes to be sent (NBYTES)========================
	// I2C1->CR2 NBYTES bits 23:16
	//Send two bytes. Register address, and then data
	// However, address locations stated to be 0x00 which is two bytes, but first byte always 0

	I2C1->CR2 |= 2<<16;

//	bitset(I2C1->CR2, 17); //2 byte mode

	// 6- Issue the START condition
	bitset(I2C1->CR2, 13); // START bit ON

	// 7- Wait until flag TXIS is asserted or TXE =============================
	// I2C->ISR TXE transmit not empty bit 0
	// TXIS Transmit interrupt status, bit set by hardware when TXDR is empty and data must be written to TXDR. Cleared when next data is written.
	while(bitcheck(I2C1->ISR, 0) == 0);// wait

	// 8- Write data byte to be sent to I2C_TXDR===============================
	//Put address and data in MSB first format?

	//char reverseaddress = reverse_bits(address);
	//I2C1->TXDR = reverseaddress;
	I2C1->TXDR = address;
//	bitset(I2C1->ISR, 0); //Flush txdr
	while(bitcheck(I2C1->ISR,0)  == 0);//wait for TXIS set for second byte

	//send second byte
	//char reversedata = reverse_bits(data);
	//I2C1->TXDR = reversedata;
	I2C1->TXDR = data;
//	bitset(I2C1->ISR, 0); //Flush txdr

	// 9- Check that the data reg is empty (TXIS or TXE)=======================
	while(bitcheck(I2C1->ISR, 0) == 0);// wait GETTING STUCK HERE

	//TODO send manual stop condition?
	// 10- Issue the STOP condition if AUTOEND was set to 0; nothing if it was set to 1===========
	//AUTOEND CURRENTLY SET
	//bitset(I2C1->CR2,14); // SEND STOP BIT
}

void InitI2C(){

	//Enable Clocks i2c & GPIO

	bitset  (RCC->CCIPR1, 13);	//I2C to use HSI16 clock 16MHz
	bitclear(RCC->CCIPR1, 12);
	bitset(RCC->CR, 8);         // HSI16 clock enable

	bitset(RCC->APB1ENR1, 21); //I2C1EN

	//Set clock to HSI16, ensure 16MHz clock being used.
	// RCC CCIPR1, I2C1SEl bits 13:12, 10 for HSI16

	bitset(RCC->AHB2ENR, 1); //GPIOB Enable
	bitset(RCC->AHB2ENR, 2); //GPIOC Enable

	// GPIO using GPIOC pin 6, for INT
	// SLC and SDA are on pind PB8 and PB9, GPIOB
	// Enable GPIO in AF and Open-drain mode (OTYPER)

	//PB08 AF AF4 for I2C_SLC
	//PB09 AF AF4 for I2C_SDA

	//GPIOC CONFIG? ===========================================================
	// GPIOC pin 6 is set to the EEPROM INTerrupt pin, pin 8


	//OTYPER for GPIOB, Open-Drain mode = 1 ==============================================
	// Pin 8, OTYPER bit 8
	bitset(GPIOB->OTYPER, 8);

	// Pin 9, OTYPER bit 9
	bitset(GPIOB->OTYPER, 9);


	//MODER FOR GPIOB =========================================================
	//Set mode 10 for AF
	//GPIOB pin 8, MODER bits 17:16
	bitset(GPIOB->MODER, 17);
	bitclear(GPIOB->MODER, 16);

	//GPIO pin 9, MODER bits 19:18
	bitset(GPIOB->MODER, 19);
	bitclear(GPIOB->MODER,18);


	//ALTERNATE FUNCTION MODE FOR GPIOB =======================================

	//Set AF mode to AF4, high register, 0b0100 = AF4
	//GPIOB pin 8, AFRH bits 3:0, set 0b0100
	bitclear(GPIOB->AFR[1], 3);
	bitset(GPIOB->AFR[1],   2);
	bitclear(GPIOB->AFR[1], 1);
	bitclear(GPIOB->AFR[1], 0);

	//GPIOB pin 9, AFRH bits 7:4, set 0b0100
	bitclear(GPIOB->AFR[1], 7);
	bitset(GPIOB->AFR[1],   6);
	bitclear(GPIOB->AFR[1], 5);
	bitclear(GPIOB->AFR[1], 4);


	//INTERNAL PULL UP RESISTOR SET ===========================================
	//MAKE SURE SET I2C SPEED TO 100KHz

	//Set pull up resistors for GPIOB pin 9,8.
	// pull-up = 0b 01
	//GPIOB pin 8, PUPDR bits 17:16, set 0b01
	bitclear(GPIOB->PUPDR, 17);
	bitset(GPIOB->PUPDR,   16);

	//GPIOB pin 9, PUPDR bits 19:18, set 0b01
	bitclear(GPIOB->PUPDR, 19);
	bitset(GPIOB->PUPDR,   18);


	//Reset I2C module, put state at idle =====================================
	// Clear the PE bit in I2C_CR1 register, bit 0
	bitclear(I2C1->CR1, 0);
	bitcheck(I2C1->CR1, 0);


	//I2C SPEED 100KHZ SET ====================================================
	//Program i2C timing based on target speed (check 1522 in reference manual) (TIMINGR)

	//TABLE SETTINGS FOR 100kHz Standard-mode(Sm)

	// f_I2CCLK
	// PRESC = 1 for f_I2CCLK = 8MHz
	// PRESC = 3 for f_I2CCLK = 16 MHz	<-- USE THIS HSI16 16MHz
	// PRESC = 0xB for f_I2CCLK = 48 MHz

	// THESE ALL SAME FOR 100kHz for 8, 16, and 48 MHz
	// SCLL 0x13
	// t_SCLL 20x250 ns = 5.0 us
	// SCLH = 0xF
	// t_SCLH 16 x 250 ns = 4.0us
	// t_SCL(1) = ~10us(2)
	// SDADEL = 0x2
	// t_SDADEL 2x 150 ns = 500 ns
	// SCLDEL = 0x4
	// t_SCLDEL 5 x 250 ns = 1250 ns

	// PRESC, 16MHz
	//Set 3 or 0b0011 for TIMINGR bits 31:28
	bitclear(I2C1->TIMINGR, 31);
	bitclear(I2C1->TIMINGR, 30);
	bitset(I2C1->TIMINGR,   29);
	bitset(I2C1->TIMINGR,   28);

	//TODO May need to check the |= is working properly
	//SCLL 0x13, TIMINGR bits 7:0
	I2C1->TIMINGR |= 0x13; // 0b00010011

	// SCLH 0xF, TIMINGR bits 15:8
	I2C1->TIMINGR |= 0x0F << 8; //0b00001111

	//SDADEL 0x2, TIMINGR bits 19:16, 0b0010
	bitclear(I2C1->TIMINGR, 19);
	bitclear(I2C1->TIMINGR, 18);
	bitset(I2C1->TIMINGR,   17);
	bitclear(I2C1->TIMINGR, 16);

	//SCLDEL = 0x4, TIMINGR bits 23:20, 0b0100
	bitclear(I2C1->TIMINGR, 23);
	bitset(I2C1->TIMINGR,   22);
	bitclear(I2C1->TIMINGR, 21);
	bitclear(I2C1->TIMINGR, 20);


	// Enable I2C peripheral ===================================================

	bitset(I2C1->CR1, 0);
}

uint8_t reverse_bits(uint8_t byte) {
    byte = ((byte & 0xF0) >> 4) | ((byte & 0x0F) << 4);
    byte = ((byte & 0xCC) >> 2) | ((byte & 0x33) << 2);
    byte = ((byte & 0xAA) >> 1) | ((byte & 0x55) << 1);
    return byte;
}
