#ifndef MOTORS_H_
#define MOTORS_H_

#define AUTO_SPEED 250

#define DRIVE_FORWARD(pwm) do {                      \
    shift_out_to_motors(FRONT_RIGHT_FORWARD    \
                        | FRONT_LEFT_FORWARD   \
                        | BACK_RIGHT_FORWARD   \
                        | BACK_LEFT_FORWARD);  \
    CCP2_LoadDutyValue(pwm);} while(0)

#define DRIVE_BACKWARDS(pwm) do {                   \
    shift_out_to_motors(FRONT_RIGHT_BACKWARD   \
                        | FRONT_LEFT_BACKWARD  \
                        | BACK_RIGHT_BACKWARD  \
                        | BACK_LEFT_BACKWARD); \
    CCP2_LoadDutyValue(pwm);} while(0)

#define DRIVE_LEFTWARDS(pwm) do {                   \
    shift_out_to_motors(FRONT_RIGHT_BACKWARD   \
                        | FRONT_LEFT_FORWARD  \
                        | BACK_RIGHT_BACKWARD  \
                        | BACK_LEFT_FORWARD); \
    CCP2_LoadDutyValue(pwm);} while(0)

#define DRIVE_RIGHTWARDS(pwm) do {                  \
    shift_out_to_motors(FRONT_RIGHT_FORWARD   \
                        | FRONT_LEFT_BACKWARD  \
                        | BACK_RIGHT_FORWARD  \
                        | BACK_LEFT_BACKWARD); \
    CCP2_LoadDutyValue(pwm);} while(0)

#define TURN_RIGHT(pwm) do {                        \
    shift_out_to_motors(FRONT_RIGHT_FORWARD   \
                        | FRONT_LEFT_BACKWARD  \
                        | BACK_RIGHT_BACKWARD  \
                        | BACK_LEFT_FORWARD); \
    CCP2_LoadDutyValue(pwm);} while(0)

#define TURN_LEFT(pwm) do {                         \
    shift_out_to_motors(FRONT_RIGHT_BACKWARD   \
                        | FRONT_LEFT_FORWARD  \
                        | BACK_RIGHT_FORWARD  \
                        | BACK_LEFT_BACKWARD); \
    CCP2_LoadDutyValue(pwm);} while(0)

#define STOP(pwm) do {\
    shift_out_to_motors(FRONT_RIGHT_FORWARD | FRONT_RIGHT_BACKWARD\
                | FRONT_LEFT_FORWARD | FRONT_LEFT_BACKWARD\
                | BACK_RIGHT_FORWARD | BACK_RIGHT_BACKWARD\
                | BACK_LEFT_FORWARD | BACK_LEFT_BACKWARD\
                );\
    CCP2_LoadDutyValue(pwm);} while(0)



typedef enum {
    FRONT_RIGHT_FORWARD  = 0b00010000,
    FRONT_RIGHT_BACKWARD = 0b00100000,

    BACK_RIGHT_FORWARD   = 0b10000000,
    BACK_RIGHT_BACKWARD  = 0b01000000,

    FRONT_LEFT_FORWARD   = 0b00000010,
    FRONT_LEFT_BACKWARD  = 0b00000001,

    BACK_LEFT_FORWARD    = 0b00000100,
    BACK_LEFT_BACKWARD   = 0b00001000
} MOTORS;

typedef enum {
    DRIVE_FORWARD    = 0,
    DRIVE_BACKWARDS  = 1,
    DRIVE_RIGHTWARDS = 2,
    DRIVE_LEFTWARDS  = 3,
    TURN_LEFT        = 4,
    TURN_RIGHT       = 5,
    STOP             = 6
} DIRECTIONS;

uint16_t SPEED = AUTO_SPEED;

void shift_out_to_motors(uint8_t byte)
{
    for (uint8_t i=0; i < 8; i++){

        uint8_t bit_mask = 1 << i;
        if (byte & bit_mask)
            SR_DATA_SetHigh();
        else
            SR_DATA_SetLow();
        
        __delay_ms(1);
        SR_CLOCK_SetHigh();
        __delay_ms(1);
        SR_CLOCK_SetLow();
        __delay_ms(1);
    }
    
    SR_SEND_SetHigh();
    __delay_ms(1);
    SR_SEND_SetLow();
    __delay_ms(1);
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
    SPEED = (uint16_t) (speed*128);
}

#endif // MOTORS_H_