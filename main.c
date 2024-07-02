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

#define STOP(pwm) do { shift_out_to_motors(MOTOR_FRONT_RIGHT_AVANCER | MOTOR_FRONT_RIGHT_RECULER\
                                            | MOTOR_FRONT_LEFT_AVANCER | MOTOR_FRONT_LEFT_RECULER\
                                            | MOTOR_BACK_RIGHT_AVANCER | MOTOR_BACK_RIGHT_RECULER\
                                            | MOTOR_BACK_LEFT_AVANCER | MOTOR_BACK_LEFT_RECULER\
                                            );\
                                CCP2_LoadDutyValue(pwm);} while(0)
#define SERVO_LEFT do {CCP1_LoadDutyValue(68);} while(0)
#define SERVO_RIGHT do {CCP1_LoadDutyValue(20);} while(0)
#define SERVO_FRONT do {CCP1_LoadDutyValue(42);} while(0)

void TMR0_Custom_ISR(void);
void auto_drive();
void UART_Custom_ISR(uint8_t Rx_Code);
void shift_out_to_motors(uint8_t byte);
uint8_t get_distance_from_supersonic();

#define AUTO_SPEED 350
int SPEED = AUTO_SPEED;

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

typedef enum {
    DRIVE_FORWARD = 0,
    DRIVE_BACKWARDS = 1,
    DRIVE_RIGHTWARDS = 2,
    DRIVE_LEFTWARDS = 3,
    TURN_LEFT = 4,
    TURN_RIGHT = 5,
    STOP = 6
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
            get_distance_from_supersonic();
            __delay_ms(10);
        }
    }
}


void speed_motor(uint8_t speed){
    SPEED = (uint16_t) (speed*128);
}

void auto_drive(){
    uint16_t distance = get_distance_from_supersonic();
    if (distance < 30){
        STOP(SPEED);
        SERVO_LEFT;
        __delay_ms(1500);
        uint16_t distance_l = get_distance_from_supersonic();
        __delay_ms(100);
        SERVO_RIGHT;
        __delay_ms(1500);
        uint16_t distance_r = get_distance_from_supersonic();
        __delay_ms(100);
        SERVO_FRONT;
        if (distance_l < distance_r){
            TURN_LEFT(SPEED);
            __delay_ms(300);
            STOP(SPEED);
        }else{
            TURN_RIGHT(SPEED);
            __delay_ms(300);
            STOP(SPEED);
        }
        __delay_ms(700);
        DRIVE_FORWARD(SPEED);
    }
    __delay_ms(10);
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

void control_speed(uint8_t code)
{
    uint8_t speed = code >> 3;
    speed &= 0b00000111;
    speed_motor(speed);
}

void control_servo(uint8_t code)
{
    auto angle = code >> 6;
    switch (angle)
    {
        case 0b00:
            SERVO_LEFT;
            break;
        case 0b01:
            SERVO_FRONT;
            break;
        case 0b10:
            SERVO_RIGHT;
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
    uint16_t distance_in_cm = (uint16_t)(((TMR1H<<8) + TMR1L)/58.82);
    char buffer[16];
    if(distance_in_cm >= 2 && distance_in_cm <= 250) // Check whether the result is valid or not
    { 
        LCD_Clear();
        LCD_Set_Cursor(1, 1);
        sprintf(buffer, "Dist.: %d cm", distance_in_cm);
        LCD_Write_String(buffer);  // Display the distance on the LCD
        EUSART_Write((uint16_t) distance_in_cm);
    }
    else 
    {
        distance_in_cm = 700;
        LCD_Clear();
        LCD_Set_Cursor(1, 1);
        LCD_Write_String("Out of Range");
    }
    return distance_in_cm;
}

float get_temperature()
{
    float temperature_in_c = ((float)(ADC_GetConversion(0) & 0x3FF) * 3.3/1023)*100;
    return temperature_in_c;
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
    
    auto_mode = 0;
    control_speed(Rx_Code);
    control_motors_with_uart(Rx_Code);
    control_servo(Rx_Code);
    
    if (Rx_Code == 0b11111111)
    {
        auto_mode = 1;
        SPEED = AUTO_SPEED;
        SERVO_FRONT;
        DRIVE_FORWARD(SPEED);
    }
        
    __delay_ms(5);
}

void TMR0_Custom_ISR(void)
{
    return;
}