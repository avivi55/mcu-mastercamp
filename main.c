#include "main.h"

bool auto_mode = false;

int main(void)
{
    SYSTEM_Initialize();

    // Disable I2C Interrupts
    PIE1bits.SSP1IE = 0; 
    PIE2bits.BCL1IE = 0; 

    INTERRUPT_GlobalInterruptEnable(); 
    INTERRUPT_PeripheralInterruptEnable(); 

    I2C_Master_Init();
    LCD_Init(0x4E); // Initialize LCD module with I2C address = 0x4E
    Backlight();
    LCD_Clear();
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("Master Camp 2024");
    
    INTCONbits.TMR0IE = 0;

    SR_DATA_SetLow();
    SR_SEND_SetLow();
    SR_CLOCK_SetLow();

    CCP2_LoadDutyValue(500);
    CCP1_LoadDutyValue(42);

    STOP(SPEED);
    while(1)
    {
        if (auto_mode)
            auto_drive();
        else{
            print_distance(get_distance_from_supersonic());
            __delay_ms(150);
        }
    }
}

void auto_drive()
{
    __delay_ms(100);
    uint16_t distance = get_distance_from_supersonic();

    print_distance(distance);

    if (distance >= 30)
        return;
    
    STOP(SPEED);
    TURN_SERVO_LEFT();
    
    __delay_ms(1000);

    uint16_t distance_l = get_distance_from_supersonic();
    print_distance(distance_l);
    __delay_ms(500);

    TURN_SERVO_RIGHT();

    __delay_ms(1000);
    
    uint16_t distance_r = get_distance_from_supersonic();
    print_distance(distance_r);
    __delay_ms(500);

    TURN_SERVO_CENTER();

    if (distance_l < distance_r)
        TURN_RIGHT(SPEED);
    else
        TURN_LEFT(SPEED);

    __delay_ms(MS_250_TURN_TIME);
    STOP(SPEED);

    __delay_ms(700);
    DRIVE_FORWARD(SPEED);
    __delay_ms(10);
}

void UART_Custom_ISR(uint8_t Rx_Code)
{
    if (!verify_password(Rx_Code))
        return;
    
    print_distance(get_distance_from_supersonic());
    
    auto_mode = false;
    control_servo(Rx_Code);
    control_speed(Rx_Code);
    control_motors_with_uart(Rx_Code);
    
    if (Rx_Code == 0b11111111)
    {
        auto_mode = true;
        SPEED = AUTO_SPEED;
        TURN_SERVO_CENTER();
        DRIVE_FORWARD(SPEED);
    }
        
    __delay_ms(10);
}