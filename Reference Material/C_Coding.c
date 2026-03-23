
#include "stm32l552xx.h"
#include "stdio.h"

// Preprocessor macros
#define MYAGE 38  // Cleaner code
#define MYAGE_MONTHS (MYAGE*12)


void delayMs(int n);

// Little Endian
// Data types
uint8_t  byte = 0;                    //Unsigned 8 bit int --range [0:255]
uint16_t halfword=0xFFFF;             //Unsigned 16 bit int
uint32_t word=0x76543210;             //Unsigned 32 bit int
unsigned int word2;                   //Identical to uint32_t
uint64_t longword=0xFEDCBA9876543210; //Unsigned 64 bit int

int8_t  sbyte;        // Signed 8 bits --range [-127:127]
int16_t shalfword;    // Signed 16 bits
int32_t sword;        // Signed 32 bits
int     sinteger;     // Signed 32 bits
signed int sinteger2; // Identical to int
int64_t slongword;    // Signed 64 bits

float   fnum=10.1; // 32-bits
double  dnum=10.1; // 64-bits

char mychar='A';   // 8 bit




//-----------------------------
//           |       bytes    |
//   32bit   |   3  2  1  0   |
//  Address  | ---------------|
//0x0000_0000|   00 00 00 00  |
//0x0000_0004|   00 00 00 00  |
//0x0000_0008|   00 00 00 00  |
//0x0000_000C|   00 00 00 00  |
//0x0000_0010|   00 00 00 00  |
//0x0000_0014|   00 00 00 00  |
//0x0000_0018|   00 00 00 00  |
//0x0000_001C|   00 00 00 00  |
// ...
// ...
//0xFFFF_FFF4|   00 00 00 00  |
//0xFFFF_FFF8|   00 00 00 00  |
//0xFFFF_FFFC|   00 00 00 00  |
//-----------------------------



int main(void) {

	int a, b, c;
	a=10;
	b=20;
	c=a+b; // Arithmetic ops:  +,-,*,/,>>,<<,%
	c=a&b; // Logic ops: &, |, ~, ^

	unsigned int myreg=0xDEEDBEEF;
    myreg    = myreg & 0xFFFFFFF0; // Clear the lower 4 bits
    myreg   &= 0xFFFFFFF0;

    myreg    = myreg | 0xF0000000; // Set the upper 4 bits
    myreg   |= 0xF0000000;

    myreg = ~myreg;      // Invert (toggle) all bits
    myreg = myreg^0xFFFFFFFF;

    // Set the 17th bit
    myreg    = myreg | (1<<17);

// Pointers
    uint32_t * myptr;     // some memory address has been reserved
    myptr=0x0000000C;     // The pointer is pointing at the word at memory address 0x0000000C
    a=*myptr;             // copy CONTENT of memory at address 0x0000000C to variable a
    *myptr = 0xDEADBEEF;  // CONTENT of memory at address 0x0000000C is changed to 0xDEADBEEF
    myptr+=4;             // The pointer is now pointing at memory address 0x00000010

// Conditions  (==, !=, >, <):
    if(a==10) printf("I am 10!\n");
    else      printf("I am not 10!\n");



    // For loops and array of characters
    char src_str []="Hello World\n";
    char dest_str [12];
	for (int i=0; i<12; i++){
		dest_str=src_str[i];
	}

    RCC->AHB2ENR |=  1;             /* enable GPIOA clock */
    RCC->CFGR    |=  12;


    GPIOA->MODER &= ~0x00000C00;    /* clear pin mode */
    GPIOA->MODER |=  0x00000400;    /* set pin to output mode */

    while(1) {
        GPIOA->ODR |=  0x00000020;  /* turn on LED */
        delayMs(500);
        GPIOA->ODR &= ~0x00000020;  /* turn off LED */
       delayMs(500);
    }
}

/* 16 MHz SYSCLK */
void delayMs(int n) {
    int i;
    //for (; n > 0; n--)
        for (i = 0; i < 254; i++)
        	GPIOA->ODR |= i&0xff;

}
