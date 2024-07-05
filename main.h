#ifndef MAIN_H_
#define MAIN_H_

#include "mcc_generated_files/system/system.h"
#include "lcd.h"
#include "motors.h"
#include "servo.h"
#include "ultrasonic.h"
#include "password.h"

void UART_Custom_ISR(uint8_t Rx_Code);
void auto_drive();
uint8_t verify_password(uint8_t password_part);

#define MS_250_TURN_TIME 320

#endif // MAIN_H_