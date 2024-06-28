 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.0
*/

/*
� [2024] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/
#include "mcc_generated_files/system/system.h"
#include "I2C_LCD.h"

/*
    Main application
*/

#define MOTOR_FRONT_RIGHT_AVANCER 0b00010000
#define MOTOR_FRONT_RIGHT_RECULER 0b00100000

#define MOTOR_BACK_RIGHT_AVANCER  0b10000000
#define MOTOR_BACK_RIGHT_RECULER  0b01000000

#define MOTOR_FRONT_LEFT_AVANCER  0b00000010
#define MOTOR_FRONT_LEFT_RECULER  0b00000001

#define MOTOR_BACK_LEFT_AVANCER   0b00000100
#define MOTOR_BACK_LEFT_RECULER   0b00001000

#define DRIVE_FORWARD(pwm) do { shift_out_to_motors(MOTOR_FRONT_RIGHT_AVANCER\
                                                    | MOTOR_FRONT_LEFT_AVANCER\
                                                    | MOTOR_BACK_RIGHT_AVANCER\
                                                    | MOTOR_BACK_LEFT_AVANCER);\
                                CCP2_LoadDutyValue(pwm);} while(0)

#define DRIVE_BACKWARDS(pwm) do { shift_out_to_motors(MOTOR_FRONT_RIGHT_RECULER\
                                                    | MOTOR_FRONT_LEFT_RECULER\
                                                    | MOTOR_BACK_RIGHT_RECULER\
                                                    | MOTOR_BACK_LEFT_RECULER);\
                                CCP2_LoadDutyValue(pwm);} while(0)

#define DRIVE_LEFTWARDS(pwm) do { shift_out_to_motors(MOTOR_FRONT_RIGHT_RECULER\
                                                    | MOTOR_FRONT_LEFT_AVANCER\
                                                    | MOTOR_BACK_RIGHT_RECULER\
                                                    | MOTOR_BACK_LEFT_AVANCER);\
                                CCP2_LoadDutyValue(pwm);} while(0)

#define DRIVE_RIGHTWARDS(pwm) do { shift_out_to_motors(MOTOR_FRONT_RIGHT_AVANCER\
                                                    | MOTOR_FRONT_LEFT_RECULER\
                                                    | MOTOR_BACK_RIGHT_AVANCER\
                                                    | MOTOR_BACK_LEFT_RECULER);\
                                CCP2_LoadDutyValue(pwm);} while(0)

#define TURN_LEFT(pwm) do { shift_out_to_motors(MOTOR_FRONT_RIGHT_AVANCER\
                                                    | MOTOR_FRONT_LEFT_RECULER\
                                                    | MOTOR_BACK_RIGHT_RECULER\
                                                    | MOTOR_BACK_LEFT_AVANCER);\
                                CCP2_LoadDutyValue(pwm);} while(0)

#define TURN_RIGHT(pwm) do { shift_out_to_motors(MOTOR_FRONT_RIGHT_RECULER\
                                                    | MOTOR_FRONT_LEFT_AVANCER\
                                                    | MOTOR_BACK_RIGHT_AVANCER\
                                                    | MOTOR_BACK_LEFT_RECULER);\
                                CCP2_LoadDutyValue(pwm);} while(0)

void TMR0_Custom_ISR(void);
void UART_Custom_ISR(uint8_t Rx_Code);
void shift_out_to_motors(uint8_t byte);

#define SPEED 300

typedef enum {
    DRIVE_FORWARD = 0x0,
    DRIVE_BACKWARDS = 0x1,
    DRIVE_RIGHTWARDS = 0x2,
    DRIVE_LEFTWARDS = 0x3,
    TURN_LEFT = 0x4,
    TURN_RIGHT = 0x5
} DIRECTIONS;

int main(void)
{
    SYSTEM_Initialize();

    // Disable I2C Interrupts
    PIE1bits.SSP1IE = 0; 
    PIE2bits.BCL1IE = 0; 

    // Enable the Global Interrupts 
    INTERRUPT_GlobalInterruptEnable(); 

    // Disable the Global Interrupts 
    //INTERRUPT_GlobalInterruptDisable(); 

    // Enable the Peripheral Interrupts 
    INTERRUPT_PeripheralInterruptEnable(); 

    // Disable the Peripheral Interrupts 
    //INTERRUPT_PeripheralInterruptDisable(); 
    PIE1bits.SSP1IE = 0; 
    PIE2bits.BCL1IE = 0;

    I2C_Master_Init();
    LCD_Init(0x4E); // Initialize LCD module with I2C address = 0x4E
    Backlight();
    LCD_Clear();
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("Master Camp 2024");
    
    INTCONbits.TMR0IE = 1;
    SR_DATA_SetLow();
    SR_SEND_SetLow();
    SR_CLOCK_SetLow();
    CCP2_LoadDutyValue(300);
    CCP1_LoadDutyValue(0);
    
    while(1)
    {    
        // //DRIVE_FORWARD(700);
        // DRIVE_FORWARD(SPEED);
        // __delay_ms(700);
        // DRIVE_BACKWARDS(SPEED);
        // __delay_ms(700);
        // DRIVE_RIGHTWARDS(SPEED);
        // __delay_ms(700);
        // DRIVE_LEFTWARDS(SPEED);
        // __delay_ms(700);
        // TURN_LEFT(SPEED);
        // __delay_ms(700);
        // TURN_RIGHT(SPEED);
        // __delay_ms(700);
    }
}


void control_motors_with_uart(DIRECTIONS dir)
{
    switch (dir)
    {
        case DRIVE_FORWARD:
            DRIVE_FORWARD(SPEED);
            break;
        case DRIVE_BACKWARDS:
            DRIVE_BACKWARDS(SPEED);
            break;
        case DRIVE_RIGHTWARDS:
            DRIVE_RIGHTWARDS(SPEED);
            break;
        case DRIVE_LEFTWARDS:
            DRIVE_LEFTWARDS(SPEED);
            break;
        case TURN_LEFT:
            TURN_LEFT(SPEED);
            break;
        case TURN_RIGHT:
            TURN_RIGHT(SPEED);
            break;
        default:
            break;
    }
}

void shift_out_to_motors(uint8_t byte)
{
    for (uint8_t i=0; i < 8; i++){
        //if (byte & (1 << i))
            //SR_DATA_SetHigh();
        //else
            //SR_DATA_SetLow();
        
        LATBbits.LATB4 = (byte & (1 << i)) >> i;
        
        __delay_ms(1);
        //SR_CLOCK_SetHigh();
        LATBbits.LATB6 = 1;
        __delay_ms(1);
        
        LATBbits.LATB4 = 1;
        LATBbits.LATB6 = 0;
        __delay_ms(1);
    }
    
    //SR_SEND_SetHigh();
    LATBbits.LATB5 = 1;
    __delay_ms(1);
    LATBbits.LATB5 = 0;
    //SR_SEND_SetLow();
    __delay_ms(1);
}

uint8_t get_distance_from_supersonic()
{
    TMR1H = 0x00;
    TMR1L = 0x00;
    // Generate a 10 microseconds pulse on Trig output
    Trig_SetHigh();
    __delay_us(10);
    Trig_SetLow();
    // Wait for Echo pin rising edge
    while(Echo_PORT == LOW);
    // Start measuring Echo pulse width in seconds
    TMR1_Start();
    // Wait for Echo pin falling edge
    while(Echo_PORT == HIGH);
    // Stop the timer 
    TMR1_Stop();
    // Estimate the distance in CM
    uint8_t distance_in_cm = (uint8_t)(((TMR1H<<8) + TMR1L)/58.82);
    
    return distance_in_cm;
}

float get_temperature()
{
    float temperature_in_c = ((float)(ADC_GetConversion(0) & 0x3FF) * 3.3/1023)*100;
    return temperature_in_c;
}

void UART_Custom_ISR(uint8_t Rx_Code)
{
    char buffer[16];
    LCD_Clear();
    LCD_Set_Cursor(1, 1);
    sprintf(buffer, "%d", Rx_Code);
    LCD_Write_String(buffer);
    control_motors_with_uart((DIRECTIONS) Rx_Code);
    __delay_ms(1000);
}


void TMR0_Custom_ISR(void)
{
    char buffer[16];

    uint8_t distance = get_distance_from_supersonic();

    if(distance >= 2 && distance <= 250) // Check whether the result is valid or not
    { 
        LCD_Clear();
        LCD_Set_Cursor(1, 1);
        sprintf(buffer, "Dist.: %d cm", distance);
        LCD_Write_String(buffer);  // Display the distance on the LCD
    }
    else 
    {
        LCD_Clear();
        LCD_Set_Cursor(1, 1);
        LCD_Write_String("Out of Range");
    }    
}