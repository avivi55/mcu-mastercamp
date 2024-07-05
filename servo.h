#ifndef SERVO_H_
#define SERVO_H_

#define TURN_SERVO_LEFT() CCP1_LoadDutyValue(68)
#define TURN_SERVO_RIGHT() CCP1_LoadDutyValue(20)
#define TURN_SERVO_CENTER() CCP1_LoadDutyValue(42)

typedef enum {
    LEFT    = 0,
    CENTER  = 1,
    RIGHT   = 2
} SERVO_ANGLE;


void control_servo(uint8_t code)
{
    uint8_t angle = (SERVO_ANGLE) (code >> 6);

    switch (angle)
    {
        case LEFT:
            TURN_SERVO_LEFT();
            break;
        case CENTER:
            TURN_SERVO_CENTER();
            break;
        case RIGHT:
            TURN_SERVO_RIGHT();
            break;
        default:
            break;
    }
}

#endif // SERVO_H_
