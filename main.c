#include "main.h"

uint16_t SPEED = AUTO_SPEED;
uint8_t auto_mode = 1;
uint8_t password_validation_index = 0;

#define PASSWORD_LEN 5
uint8_t password[PASSWORD_LEN] = {
    0x6a,
    0x7c,
    0xee,
    0xe0,
    0xb0
};

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

    DRIVE_FORWARD(SPEED);
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

void print_distance(uint8_t distance)
{
    char buffer[16];
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
        TURN_LEFT(SPEED);
    else
        TURN_RIGHT(SPEED);

    __delay_ms(300);
    STOP(SPEED);

    __delay_ms(700);
    DRIVE_FORWARD(SPEED);
    __delay_ms(10);
}

void control_speed(uint8_t code)
{
    uint8_t speed = code >> 3;
    speed &= 0b00000111;
    SPEED = (uint16_t) (speed*128);
}

void control_motors_with_uart(uint8_t code)
{
    DIRECTIONS dir = (DIRECTIONS) (code & 0b111);
    
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
            STOP(SPEED);
            break;
    }
}

void control_servo(uint8_t code)
{
    uint8_t angle = code >> 6;

    switch (angle)
    {
        case 0b00:
            TURN_SERVO_LEFT();
            break;
        case 0b01:
            TURN_SERVO_CENTER();
            break;
        case 0b10:
            TURN_SERVO_RIGHT();
            break;
        default:
            break;
    }
}

void shift_out_to_motors(uint8_t byte)
{
    for (uint8_t i=0; i < 8; i++){    
        LATBbits.LATB4 = (byte & (1 << i)) >> i;
        
        __delay_ms(1);
        SR_CLOCK_SetHigh();
        __delay_ms(1);
        
        SR_DATA_SetHigh();
        SR_CLOCK_SetLow();
        __delay_ms(1);
    }
    
    SR_SEND_SetHigh();
    __delay_ms(1);
    SR_SEND_SetLow();
    __delay_ms(1);
}

uint8_t get_distance_from_supersonic()
{
    TMR1H = 0x00;
    TMR1L = 0x00;

    // Trigger impulse generation
    Trig_SetHigh();
    __delay_us(10);
    Trig_SetLow();

    // Echo delay measurement
    while(Echo_PORT == LOW);
    TMR1_Start();
    while(Echo_PORT == HIGH);
    TMR1_Stop();

    uint16_t distance_in_cm = (uint16_t)(((TMR1H<<8) + TMR1L)/58.82);
    
    // to make sure the out of rage engages
    if (distance_in_cm < 2 || distance_in_cm > 250)
        distance_in_cm = 700;
    
    return distance_in_cm;
}

uint8_t verify_password(uint8_t password_part)
{
    if (password_validation_index == PASSWORD_LEN)
    {
        password_validation_index = 0;
        return 1;
    }
    
    if (password_part == password[password_validation_index])
        password_validation_index++;
    else
        password_validation_index = 0;
    
    return 0;
}

void UART_Custom_ISR(uint8_t Rx_Code)
{
    if (!verify_password(Rx_Code))
        return;
    
    print_distance(get_distance_from_supersonic());
    
    auto_mode = 0;
    control_servo(Rx_Code);
    control_speed(Rx_Code);
    control_motors_with_uart(Rx_Code);
    
    if (Rx_Code == 0b11111111)
    {
        auto_mode = 1;
        SPEED = AUTO_SPEED;
        TURN_SERVO_CENTER();
        DRIVE_FORWARD(SPEED);
    }
        
    __delay_ms(10);
}