#ifndef LCD_H_
#define LCD_H_

#include "I2C_LCD.h"

char buffer[16];

void print_distance(uint8_t distance)
{
    LCD_Clear();
    LCD_Set_Cursor(1, 1);
    
    // Check whether the result is valid or not
    if(distance >= 2 && distance <= 250)
    { 
        sprintf(buffer, "Dist.: %d cm", distance);
        LCD_Write_String(buffer);
        EUSART_Write((uint16_t) distance);
    }
    else 
        LCD_Write_String("Out of Range");
}

#endif // LCD_H_