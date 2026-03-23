/*
 * HM-3300/3600 Dust Sensor
 * When using IIC communication clock frequency is 100kHz to 400kHz
 * I2C standard mode (100KHz), the sensor acts as a slave device to the IIC at address 0x08.
 *
 * In sleep mode, the fan stops working. It takes at least 30 seconds for stabilize when restart the ran.
 * To obtain accurate measurement data, it is recommended that the sensor work time should not be less than 30 seconds after wake up.
 *
 * Control Instruction Format:
 *
 * |Start	|Address Write	|Answer	|Command	|answer		|end	|
 * ------------------------------------------------------------------
 * |Start	|0x80			|ACK	|0x88		|ACK		|Stop	|
 *
 * Data frame format is as follows:
 * |Start	|Address Read	|Answer	|Data		|answer		|Data...	| answer	|end	|
 * ------------------------------------------------------------------------------------------
 * |Start	|0x81			|ACK	|Data1		|ACK		|Data...n	|ACK		|Stop	|
 *
 * Data Format:
 * data1~2		|	Reserved
 * data3~4		|	Sensor Number
 * data5~6		|	PM1.0 Concentration (CF = 1 , Standard particulate)	unit ug/m^3
 * data7~8		|	PM2.5 Concentration (CF = 1 , Standard particulate)	unit ug/m^3
 * data9~10		|	PM10  Concentration (CF = 1 , Standard particulate)	unit ug/m^3
 * data11~12	|	PM1.0 Concentration (Atmospheric environment)		unit ug/m^3
 * data13~14	|	PM2.5 Concentration (Atmospheric environment)		unit ug/m^3
 * data15~16	|	PM10  Concentration (Atmospheric environment)		unit ug/m^3
 * data17~18	|	the number of particles with diameter 0.3um or above in 1 liter of air
 * data19~20	|	the number of particles with diameter 0.5um or above in 1 liter of air
 * data21~22	|	the number of particles with diameter 1.0um or above in 1 liter of air
 * data23~24	|	the number of particles with diameter 2.5um or above in 1 liter of air
 * data25~26	|	the number of particles with diameter 5.0um or above in 1 liter of air
 * data27~28	|	the number of particles with diameter 10um  or above in 1 liter of air
 * data29		|	Data0~28 Checksum
 *
 *
 *
 * Recommended household PM levels
 *
 * PM2.5 (data 7~8)
 * PM2.5(ug/m^3)	Airquality		Action
 * 0-12				Good			fan off
 * 13-35			Moderate		Fan on
 * >35				Unhealthy		Fan	on
 *
 * note: include hysteresis to prevent relay chattering and/or default run time(eg. at least run for 1 minute)
 *
 * Alergen detection
 * PM10
 * Pollen				10-100 um
 * Dust mite debreis	10-40um
 * Pet Dander			~2..5-10um
 *
 * Data triggers to use
 * Relevent data:
 * PM2.5 concentration	->	Data7~8
 * PM10	 concentration	->	Data9~10
 *
 * if pm2.5 >= 25 	OR 	PM10 >= 50 	-> FAN ON
 * if pm2.5 <= 15	AND	PM10 <= 40 	-> FAN OFF
 *
 */

// Includes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "main.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"


// Some helper macros ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1   ) //Checks the bit number <idx> -- 0 means clear; !0 means set.




//Pre-Declarations of functions. Full declarations are after main(). ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void InitGPIOA();
void InitGPIOC();
void InitGPIOB();
void InitGPIOG();
void InitGPIOE();
void InitInt();
void InitI2C();
void I2C_Write(uint8_t slv_addr, uint8_t* data, uint8_t data_length );
uint8_t* I2C_Read(uint8_t slv_addr, uint8_t reg_addr, uint8_t data_length );
bool I2C_CheckAddr(uint8_t slv_addr);
void Delay_ms();
void InitLPUART1(uint32_t baud, uint8_t stop, uint8_t data_len, bool parity_en, bool parity_odd);
void LPUART1write (int ch);
void myprint (char msg[]);
void InitBtn();

// New functions

void ToggleRelay(); //Uses GPIOA pin __
void CheckParticles();
void Reset();
void InitAirSensor();
uint8_t* GetAirSensorData();
void SensorToLPUART(uint8_t* data);
void NicePrint(char* string, uint16_t data);
uint16_t GetPMData(uint8_t* data, uint8_t location);
void SetLEDarray(uint8_t* data);

// Add labels and send data to LPUART
uint16_t GetPM10Data_Standard(uint8_t* data, bool print);
uint16_t GetPM2Data_Standard(uint8_t* data, bool print);
uint16_t GetPM1Data_Standard(uint8_t* data, bool print);
uint16_t GetPM10Data_Atmospheric(uint8_t* data, bool print);
uint16_t GetPM2Data_Atmospheric(uint8_t* data, bool print);
uint16_t GetPM1Data_Atmospheric(uint8_t* data, bool print);

// Global Constants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//static const uint8_t I2C_SLAVE_ADDR = 0x80 //default for actual sensor

//Grove Arduino example code uses 0x40
static const uint8_t I2C_SLAVE_ADDR = 0x40;
//static const uint8_t I2C_SLAVE_ADDR = 0x80;

#define PM1THRESHOLD 15
#define PM2THRESHOLD 10
#define PM10THRESHOLD 25

bool MANUALTOGGLE = false; //Set by pushbutton to force pause operations. Disables relay being on.

int main(){

	//Configure GPIOs ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitGPIOA();
	InitGPIOC();
	InitGPIOB();
	InitGPIOG();
	InitGPIOE();

	InitBtn();
	//Configure Timers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	bitset(RCC->CFGR, 0);     // Use HSI16 as SYSCLK

	//Configure Button Interrupt ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//InitInt();

	//Configure LPUART ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitLPUART1(115200,1,8,false,false);	//InitLPUART1(baud, stop, data_len, parity_en, parity_odd)

	//Configure I2C ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitI2C();

	//Check that slave is on network ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	//Blue LED for debugging purposes, sometimes on power up the startup gets
	//Stuck trying to run I2C_CheckAddr. When this happens pressing the reset
	//Switch tends to fix it. So, if you see blue hit reset.
	bitset(GPIOB->ODR, 7);	//Blue LED - ON, checking for slv addr
	bool Addr_Found = I2C_CheckAddr(I2C_SLAVE_ADDR);
	if (Addr_Found){
		bitclear(GPIOA->ODR, 9);	//Red LED - Off
	} else {
		bitset(GPIOA->ODR, 9);		//Red LED - On
	}
	bitclear(GPIOB->ODR, 7); //Outside of I2C Check addr Blue LED - OFF


	// Put Sensor in I2C mode ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	InitAirSensor(); //Send I2C start command to sensor

	uint16_t PM1 = 0;	//Holds current PM1 Mass concentrations
	uint16_t PM2 = 0;	//Holds current PM2.5 Mass concentrations
	uint16_t PM10 = 0;	//Holds current PM10 mass concentrations

	uint8_t PM1LowerThreshold = PM1THRESHOLD - (PM1THRESHOLD * 0.25);
	uint8_t PM2LowerThreshold = PM2THRESHOLD - (PM2THRESHOLD * 0.5);
	uint8_t PM10LowerThreshold = PM10THRESHOLD -(PM10THRESHOLD *0.5);

	//Infinite Loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	while(1){

		while(MANUALTOGGLE){
			bitset(GPIOA->ODR, 9);		//Red LED - On, Signal in manual off mode
			bitclear(GPIOA->ODR, 2);	//Turn Relay off
			Delay_ms(100);		//slow things down for a moment, may not be needed, mostly for checking operations
		}
		if(!MANUALTOGGLE){
			bitclear(GPIOA->ODR, 9);	//Switch red led off when back to normal
		}

		uint8_t* data = GetAirSensorData();

		PM10 = GetPM10Data_Atmospheric(data, true);
		PM2 = GetPM2Data_Atmospheric(data, true);
		PM1 = GetPM1Data_Atmospheric(data, true);

//		PM10 = GetPM10Data_Standard(data, true);
//		PM2 = GetPM2Data_Standard(data, true);
//		PM1 = GetPM1Data_Standard(data, true);

		SetLEDarray(data);

		if(PM2 >= PM2THRESHOLD || PM10 >= PM10THRESHOLD || PM1 >= PM1THRESHOLD){
			bitset(GPIOA->ODR, 2); //Turn fan relay on
		}
		if(PM2 <= PM2LowerThreshold && PM10 <= PM10LowerThreshold &&  PM1<= PM1LowerThreshold){
			bitclear(GPIOA->ODR,2);	//Turn fan relay off
		}

//		SensorToLPUART(data);

		Delay_ms(10000);//Helps prevent fan from switching on and off too fast
	}


}

void EXTI13_IRQHandler(){
	NVIC_DisableIRQ(EXTI13_IRQn);
	Delay_ms(100);
	if(MANUALTOGGLE == true){
		MANUALTOGGLE = false;
	}
	if(MANUALTOGGLE == false){
		MANUALTOGGLE = true;
	}
	NVIC_EnableIRQ(EXTI13_IRQn);
	EXTI->RPR1 |= 1<<13; // Cleared by writing 1 to it!
	                     // Use RPR1 when trigger by Rising edge
						 // Use FPR1 when trigger by Lower edge
}

uint16_t GetPMData(uint8_t* data, uint8_t location){
	if(location > 28 || location < 0){
		char error[] = "Error invalid data address";
		myprint(error);
		return 0; //todo make an error rather than just return
	}
	uint16_t PM = (data[location] << 8) | data[location+1];  // MSB first
	return PM;
}

void NicePrint(char* string, uint16_t data){
	char buffer[64]; //safe size, tried doing fully dynamic, but added to much complexity for this purpose
	snprintf(buffer, sizeof(buffer), "%s %u\r\n", string, data);
	myprint(buffer);
}

uint16_t GetPM2Data_Standard(uint8_t* data, bool print){
	uint16_t PM2 = GetPMData(data, 6);
	if(print)
		NicePrint("PM2.5 Standard Value:",  PM2);
	return PM2;
}

uint16_t GetPM10Data_Standard(uint8_t* data, bool print){
	uint16_t PM10 = GetPMData(data, 8);
	if(print)
		NicePrint("PM10 Standard Value:",  PM10);
	return PM10;
}

uint16_t GetPM1Data_Standard(uint8_t* data, bool print){
	uint16_t PM1 = GetPMData(data, 4);
	if(print)
		NicePrint("PM1 Standard Value:",  PM1);
	return PM1;
}

uint16_t GetPM2Data_Atmospheric(uint8_t* data, bool print){
	uint16_t PM2 = GetPMData(data, 12);
	if(print)
		NicePrint("PM2.5 Atmospheric Value:",  PM2);
	return PM2;
}

uint16_t GetPM10Data_Atmospheric(uint8_t* data, bool print){
	uint16_t PM10  = GetPMData(data, 14);
	if(print)
		NicePrint("PM10 Atmospheric Value:",  PM10);
	return PM10;
}

uint16_t GetPM1Data_Atmospheric(uint8_t* data, bool print){
	uint16_t PM1 = GetPMData(data, 10);
	if(print)
		NicePrint("PM1 Atmospheric Value:",  PM1);
	return PM1;
}

void SetLEDarray(uint8_t* data){
	//GPIOE pins 0-9 LED array

	uint16_t pm25 = GetPM2Data_Atmospheric(data, false);
    int led_index;

    if (pm25 <= 2)       led_index = 0;
    else if (pm25 <= 5)  led_index = 1;
    else if (pm25 <= 8)  led_index = 2;
    else if (pm25 <= 12) led_index = 3;
    else if (pm25 <= 17) led_index = 4;
    else if (pm25 <= 25) led_index = 5;
    else if (pm25 <= 35) led_index = 6;
    else if (pm25 <= 50) led_index = 7;
    else if (pm25 <= 70) led_index = 8;
    else                 led_index = 9;

    // Shift bitmask left by 2 to target PE2–PE11
    uint16_t led_mask = ((1 << (led_index + 1)) - 1) << 2;

    // Clear bits 2–11, then set new mask
    GPIOE->ODR = (GPIOE->ODR & ~(0x3FF << 2)) | (led_mask & (0x3FF << 2));
}

void SensorToLPUART(uint8_t* data) {
    if (data == NULL) return;

    char buffer[64];
    for (uint8_t i = 0; i < 28; i += 2) {
        uint16_t value = (data[i] << 8) | data[i + 1];  // MSB first
        snprintf(buffer, sizeof(buffer), "Value %u: %u\r\n", (i ) / 2, value);
        myprint(buffer);
    }
}

uint8_t* GetAirSensorData(){
	uint8_t* SensorData = I2C_Read(I2C_SLAVE_ADDR, 0x81, 29 );
	return SensorData;
}

void InitAirSensor(){
	//Set sensor in I2C mode.
	//Command 0x88
	//Address 0x80?
	uint8_t data = 0x88;
	uint8_t* datap = &data;
	uint8_t data_length = 2;
	I2C_Write(I2C_SLAVE_ADDR, datap, data_length );
}


void ToggleRelay(){
	//Toggle Relay, bitflip GPIOA pin 2
	bitflip(GPIOA->ODR, 2);
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

	//INIT RELAY control
	//Set GPIO A, Pin 2 to "General purpose output mode"
	bitset(GPIOA->MODER, 4);
	bitclear(GPIOA->MODER,5);
}

void InitGPIOC(){
	bitset(RCC->AHB2ENR, 2);	// Enable Clock to GPIOC

	//Set GPIO Port C, Pin 6 to "Input mode".
	bitclear(GPIOC->MODER, 13);
	bitclear(GPIOC->MODER, 12);

	//LED green PC07
	// Set GPIO port C, Pin 7 to "General purpose output mode"
	bitset(GPIOC->MODER, 14);
	bitclear(GPIOC->MODER, 15);

}

void InitGPIOB(){
	//Sets up GPIOB pins for I2C operation.
	//Setup to use internal pull-up for I2C SLC and SDA lines

	bitset(RCC->AHB2ENR, 1);	// Enable Clock to GPIOB

	//Set GPIO Port B, Pin 8 to "Alternate function mode" 0b10
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


	//LED blue, PB07
	bitset(GPIOB->MODER, 14);
	bitclear(GPIOB->MODER, 15);
}

void InitGPIOG(){
	//Sets GPIOG pins to work for LPUART

	bitset(RCC->AHB2ENR,   6);  	// Enable Clock to GPIOG
	bitset(RCC->APB1ENR1, 28);  	// Enable Clock to PWR Interface
	bitset(PWR->CR2, 9);        	// Enable GPIOG power

	//Set GPIOG.7 to AF8 (LPUART1_TX)
	bitset(GPIOG->MODER,    15);  	// Setting 0b10 (Alternate Function) in pin 7 two bit mode cfgs
	bitclear(GPIOG->MODER,  14);

	bitset(GPIOG->AFR[0],   31);  	// Programming 0b1000 (AF8 = LPUART1_TX)
	bitclear(GPIOG->AFR[0], 30);
	bitclear(GPIOG->AFR[0], 29);
	bitclear(GPIOG->AFR[0], 28);

	//Set GPIOG.8 to AF8 (LPUART1_RX)
	bitset(GPIOG->MODER,    17);  	// Setting 0b10 (Alternate Function) in pin 8 two bit mode cfgs
	bitclear(GPIOG->MODER,  16);

	bitset(GPIOG->AFR[1], 3);  		// Programming 0b1000 (AF8 = LPUART1_RX)
	bitclear(GPIOG->AFR[1], 2);
	bitclear(GPIOG->AFR[1], 1);
	bitclear(GPIOG->AFR[1], 0);

}

void InitGPIOE(){
	//Sets GPIOE pins for LED array

	bitset(RCC->AHB2ENR, 4);//Enable GPIOE clock

	//set as outputs (01) For each pin PE_2 through PE_11
	bitset(GPIOE->MODER, 4);
	bitclear(GPIOE->MODER, 5);

	bitset(GPIOE->MODER, 6);
	bitclear(GPIOE->MODER, 7);

	bitset(GPIOE->MODER, 8);
	bitclear(GPIOE->MODER, 9);

	bitset(GPIOE->MODER, 10);
	bitclear(GPIOE->MODER, 11);

	bitset(GPIOE->MODER, 12);
	bitclear(GPIOE->MODER, 13);

	bitset(GPIOE->MODER, 14);
	bitclear(GPIOE->MODER, 15);

	bitset(GPIOE->MODER, 16);
	bitclear(GPIOE->MODER, 17);

	bitset(GPIOE->MODER, 18);
	bitclear(GPIOE->MODER, 19);

	bitset(GPIOE->MODER, 20);
	bitclear(GPIOE->MODER, 21);

	bitset(GPIOE->MODER, 22);
	bitclear(GPIOE->MODER, 23);

	GPIOE->ODR = 0x0000;//Clear LED output first
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

void InitBtn(){
	RCC->APB2ENR   |= 1;  // Enable Clock to SYSCFG & EXTI
	RCC->AHB2ENR |= (1<<2); // Ensure Enable GPIOC

	bitclear(GPIOC->MODER, 26); // Clear bit 26 and 27, input mode
	bitclear(GPIOC->MODER, 27);

	bitset(EXTI->EXTICR[3], 9);  // Select PC13
	bitset(EXTI->RTSR1,13);     // Trigger on rising edge of PC13
	bitset(EXTI->IMR1,13);     // Interrupt mask disable for PC13

	NVIC_SetPriority(EXTI13_IRQn, 0); // 0 highest priority

	NVIC_EnableIRQ(EXTI13_IRQn);
}

void Error_Handler(void) {
}
