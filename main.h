#ifndef MAIN_H_
#define MAIN_H_

#include "mcc_generated_files/system/system.h"
#include "I2C_LCD.h"

void TMR0_Custom_ISR(void);
void UART_Custom_ISR(uint8_t Rx_Code);
void auto_drive();
void shift_out_to_motors(uint8_t byte);
uint8_t get_distance_from_supersonic();
void print_distance(uint8_t distance);
void control_motors_with_uart(uint8_t code);
void control_speed(uint8_t code);
void control_servo(uint8_t code);
uint8_t verify_password(uint8_t password_part);

#define MOTOR_FRONT_RIGHT_AVANCER 0b00010000
#define MOTOR_FRONT_RIGHT_RECULER 0b00100000

#define MOTOR_BACK_RIGHT_AVANCER  0b10000000
#define MOTOR_BACK_RIGHT_RECULER  0b01000000

#define MOTOR_FRONT_LEFT_AVANCER  0b00000010
#define MOTOR_FRONT_LEFT_RECULER  0b00000001

#define MOTOR_BACK_LEFT_AVANCER   0b00000100
#define MOTOR_BACK_LEFT_RECULER   0b00001000

#define AUTO_SPEED 350


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

#define TURN_RIGHT(pwm) do { shift_out_to_motors(MOTOR_FRONT_RIGHT_AVANCER\
                                                    | MOTOR_FRONT_LEFT_RECULER\
                                                    | MOTOR_BACK_RIGHT_RECULER\
                                                    | MOTOR_BACK_LEFT_AVANCER);\
                                CCP2_LoadDutyValue(pwm);} while(0)

#define TURN_LEFT(pwm) do { shift_out_to_motors(MOTOR_FRONT_RIGHT_RECULER\
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

#define TURN_SERVO_LEFT() CCP1_LoadDutyValue(68);
#define TURN_SERVO_RIGHT() CCP1_LoadDutyValue(20);
#define TURN_SERVO_CENTER() CCP1_LoadDutyValue(42);

typedef enum {
    DRIVE_FORWARD = 0,
    DRIVE_BACKWARDS = 1,
    DRIVE_RIGHTWARDS = 2,
    DRIVE_LEFTWARDS = 3,
    TURN_LEFT = 4,
    TURN_RIGHT = 5,
    STOP = 6
} DIRECTIONS;


// float get_temperature()
// {
//     float temperature_in_c = ((float)(ADC_GetConversion(0) & 0x3FF) * 3.3/1023)*100;
//     return temperature_in_c;
// }

#endif // MAIN_H_