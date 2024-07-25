## -Lithion-Power---Assessment-
To measure 0 to 100V DC using a microcontroller like the ESP32, you can use a voltage divider circuit to step down the voltage to a range that the microcontroller's ADC (Analog-to-Digital Converter) can handle. The ESP32's ADC typically operates at a maximum input voltage of 3.3V.

## IDE MP LAB
## Compiler XC8

Here's a step-by-step guide on how to do this:

## Design the Voltage Divider:

A voltage divider uses two resistors to scale down the input voltage to a lower voltage. The voltage divider formula is:
Vout = Vin * R2 / R1 + R2

For example, to scale down 100V to 3.3V:
3.3V = 100V * R2/ R1 + R2
R2 / R1 + R2 = 3.3 / 100 = 0.033
## Choose Resistor Values:
Let's choose R2 = 3.3kΩ
then
      3.3kΩ / R1 + 3.3kΩ = 0.033
Soving for R1:
       R1 = 3.3kΩ / 0.033 - 3.3kΩ ≈96.7kΩ
    You can choose standard resistor values close to these calculated values. For instance R1 =  100kΩ and R2 =   3.3kΩ
  
## Connect the Voltage Divider:

-> Connect the high voltage (0 to 100V) to the input of the voltage divider.
-> Connect the junction of the two resistors to an ADC pin of the ESP32.
-> Ensure that the ground of the high voltage source is common with the ESP32 ground.
## Protect the Microcontroller:

-> To protect the ADC pin from over-voltage, you can add a Zener diode (e.g., 3.3V) across the ADC input and ground.
-> You can also add a small capacitor (e.g., 100nF) across the ADC input to filter out noise.

## Measure and Calculate the Voltage:
In your ESP32 code, read the ADC value and convert it back to the original voltage:

int adcValue = analogRead(ADC_PIN);
float voltage = (adcValue / 4095.0) * 3.3; // Assuming 12-bit ADC resolution
float inputVoltage = voltage * ((R1 + R2) / R2);

IDE and Compiler
For this example, we'll assume you're using MPLAB X IDE, a popular choice for programming PIC microcontrollers. MPLAB X IDE includes its own C compiler specifically designed for PIC microcontrollers.

## Algorithm
-> Initialization:
-> Set up system clock and I/O pins.
-> Configure the ADC module (enable, reference voltage, resolution).
Main Loop:
-> Read the ADC value from the pin connected to the voltage divider output.
-> Calculate the actual voltage based on the voltage divider ratio and ADC reference voltage.
Optionally, display or process the measured voltage (e.g., using LEDs, sending data to a computer).
 

## Flowchart
           +-------------------+
           |      Start       |
           +-------------------+
                    |
                    V
           +-------------------+
           |  Initialize ADC   |
           +-------------------+
                    |
                    V
        +-------------------+     +-------------------+
        |  Read ADC Value         Calculate Voltage |
        +-------------------+     +-------------------+
                    |                       
                    V (Yes)
        +-------------------+     +-------------------+
        |  Display/Process  | -->|  End of Loop?     |
        +-------------------+     +-------------------+
                    |
                    V (No)
                    |
           +-------------------+
           |       Loop        |
           +-------------------+

## Circuit Diagram:

+---------+     +---------+     +---------+
|         |-----| R1 (High) |-----|         | (Input: 0-100V DC)
+---------+     +---------+     +---------+
          |
          V (Voltage divider)
          |
+---------+     +---------+     +---------+
|         |-----| R2        |-----|         | (Output: Scaled down voltage)
+---------+     +---------+     +---------+
          |
          V
+---------+-----| Analog Input Pin |-----+---------+
| MCU     |     +---------+             |
+---------+                             +---------+
                                            | To Ground |
                                            +---------+


Program Code

## ADC.h

#ifndef ADC_H
#define ADC_H

#define CHANNEL0		0x00
#define CHANNEL1		0x01
#define CHANNEL2		0x02
#define CHANNEL3		0x03
#define CHANNEL4		0x04
#define CHANNEL5		0x05
#define CHANNEL6		0x06
#define CHANNEL7		0x07
#define CHANNEL8		0x08
#define CHANNEL9		0x09
#define CHANNEL10		0x0A

void init_adc(void);
unsigned short read_adc(unsigned char channel);

#endif

## CLCD.h

#ifndef LCD_H
#define LCD_H



#define CLCD_PORT			PORTD
#define CLCD_EN				RC2
#define CLCD_RS				RC1
#define CLCD_RW				RC0
#define CLCD_BUSY			RD7
#define PORT_DIR			TRISD7


#define HI												1
#define LO												0

#define INPUT											0xFF
#define OUTPUT											0x00

#define DATA_COMMAND									1
#define INSTRUCTION_COMMAND								0
#define _XTAL_FREQ                  20000000
#define LINE1(x)									(0x80 + (x))
#define LINE2(x)										(0xC0 + (x))

#define TWO_LINE_5x8_MATRIX_8_BIT					clcd_write(0x38, INSTRUCTION_COMMAND)
#define CLEAR_DISP_SCREEN				                clcd_write(0x01, INSTRUCTION_COMMAND)
#define CURSOR_HOME							clcd_write(0x02, INSTRUCTION_COMMAND)
#define DISP_ON_AND_CURSOR_OFF						clcd_write(0x0C, INSTRUCTION_COMMAND)
#define EIGHT_BIT_MODE   0x33
void init_clcd(void);
void clcd_print(const unsigned char *data, unsigned char addr);
void clcd_putch(const unsigned char data, unsigned char addr);
void clcd_write(unsigned char bit_values, unsigned char control_bit);

#endif

## MAIN.h

#ifndef MAIN_H
#define MAIN_H

#endif

## ADC.c

#include <xc.h>
#include "adc.h"

void init_adc(void)
{
	/* Selecting right justified ADRES Registers order */
	ADFM = 1;

	/* 
	 * Acqusition time selection bits 
	 * Set for 4 Tad
	 */
	ACQT2 = 0;
	ACQT1 = 1;
	ACQT0 = 0;

	/*
	 * Selecting the conversion clock of Fosc / 32 -> 1.6usecs -> 1Tad
	 * Our device frequency is 20 MHz
	 */
	ADCS0 = 0;
	ADCS1 = 1;
	ADCS2 = 0;

	/* Stop the conversion to start with */
	GODONE = 0;

	

	/* Voltage reference bit as VSS */
	VCFG1 = 0;
	/* Voltage reference bit as VDD */
	VCFG0 = 0;

	/* Just clearing the ADRESH & ADRESL registers, for time pass */
	ADRESH = 0;
	ADRESL = 0;

	/* Turn ON the ADC module */
	ADON = 1;
}

unsigned short read_adc(unsigned char channel)
{
	unsigned short reg_val;

	/*select the channel*/
	ADCON0 = (ADCON0 & 0xC3) | (channel << 2);

	/* Start the conversion */
	GO = 1;
	while (GO);
	reg_val = (ADRESH << 8) | ADRESL; 

	return reg_val;
}

## CLCD.c

#include <xc.h>
#include "clcd.h"

void clcd_write(unsigned char byte, unsigned char control_bit)
{
	CLCD_RS = control_bit;
	CLCD_PORT = byte;

	/* Should be atleast 200ns */
	CLCD_EN = HI;
	CLCD_EN = LO;

	PORT_DIR = INPUT;
	CLCD_RW = HI;
	CLCD_RS = INSTRUCTION_COMMAND;

	do
	{
		CLCD_EN = HI;
		CLCD_EN = LO;
	} while (CLCD_BUSY);

	CLCD_RW = LO;
	PORT_DIR = OUTPUT;
}

void init_clcd()
{
	/* Set PortD as output port for CLCD data */
	TRISD = 0x00;
	/* Set PortC as output port for CLCD control */
	TRISC = TRISC & 0xF8;

	CLCD_RW = LO;

	
     /* Startup Time for the CLCD controller */
    __delay_ms(30);
    
    /* The CLCD Startup Sequence */
    clcd_write(EIGHT_BIT_MODE, INSTRUCTION_COMMAND	);
    __delay_us(4100);
    clcd_write(EIGHT_BIT_MODE, INSTRUCTION_COMMAND	);
    __delay_us(100);
    clcd_write(EIGHT_BIT_MODE, INSTRUCTION_COMMAND	);
    __delay_us(1); 
    
    CURSOR_HOME;
    __delay_us(100);
    TWO_LINE_5x8_MATRIX_8_BIT;
    __delay_us(100);
    CLEAR_DISP_SCREEN;
    __delay_us(500);
    DISP_ON_AND_CURSOR_OFF;
    __delay_us(100);
}

void clcd_print(const unsigned char *data, unsigned char addr)
{
	clcd_write(addr, INSTRUCTION_COMMAND);
	while (*data != '\0')
	{
		clcd_write(*data++, DATA_COMMAND);
	}
}

void clcd_putch(const unsigned char data, unsigned char addr)
{
	clcd_write(addr, INSTRUCTION_COMMAND);
	clcd_write(data, DATA_COMMAND);
}


## MAIN.c

#include "main.h"
#include "clcd.h"
#include "adc.h"
#include <xc.h>

static void init_config(void)
{
    init_adc();
    init_clcd();
}

void main(void)
{
    unsigned short adc_reg_val;
    int catch;
    
    init_config();

    while (1)   
    {
        adc_reg_val = read_adc(CHANNEL4);
        catch = adc_reg_val/10.23; // Conversion to get values from 0 to 100V. 
        
        /*Printing in CLCD*/
        clcd_print("Measured VOLTAGE:",LINE1(0));
        clcd_putch('0'+ (catch%10) ,LINE2(2)); // For 1's number
        clcd_putch('0'+ (catch/10)%10 ,LINE2(1)); // For 2's number.
        clcd_putch('0'+ (catch/100) ,LINE2(0)); // For 3's number.
    }
}




 ## Calculate achievable theoretical accuracy

-> ADC Resolution: The ADC resolution determines the smallest voltage change the microcontroller can distinguish. For example, a 10-bit ADC with a 5V reference voltage can theoretically resolve voltage changes as small as:

Resolution = (Reference Voltage) / (2^Number of Bits - 1)
Resolution = (5V) / (2^10 - 1) ≈ 4.88 mV


-> Voltage Divider Ratio: The voltage divider scales down the input voltage. The accuracy of this scaling depends on the resistor tolerances. Let R1 be the high-resistance resistor connected to the input voltage and R2 be the resistor connected to the microcontroller's ADC pin. The ideal output voltage (Vout) is:

Vout = (Input Voltage) * (R2 / (R1 + R2))

-> Calculate Error Due to Voltage Divider: The error due to the voltage divider is the difference between the ideal output voltage (Vout) and the average of the minimum and maximum output voltages (Vout_avg).

Error_divider = abs(Vout - (Vout_min + Vout_max) / 2)

-> Quantize Error Due to ADC Resolution: The ADC can only represent discrete voltage values based on its resolution. The quantization error is half the resolution of the ADC.

Quantization_error = Resolution / 2

-> Total Theoretical Error: The total theoretical error is the sum of the error due to the voltage divider and the quantization error.

Total_error = Error_divider + Quantization_error

-> Express Error as Percentage: To express the total error as a percentage of the full-scale input voltage (100V in this case), calculate:

Percentage_error = (Total_error / 100V) * 100%





