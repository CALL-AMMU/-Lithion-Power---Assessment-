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













