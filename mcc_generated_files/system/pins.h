/**
 * Generated Pins header File
 * 
 * @file pins.h
 * 
 * @defgroup  pinsdriver Pins Driver
 * 
 * @brief This is generated driver header for pins. 
 *        This header file provides APIs for all pins selected in the GUI.
 *
 * @version Driver Version  3.0.0
*/

/*
© [2024] Microchip Technology Inc. and its subsidiaries.

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

#ifndef PINS_H
#define PINS_H

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set IO_RA0 aliases
#define IO_RA0_TRIS                 TRISAbits.TRISA0
#define IO_RA0_LAT                  LATAbits.LATA0
#define IO_RA0_PORT                 PORTAbits.RA0
#define IO_RA0_WPU                  WPUAbits.WPUA0
#define IO_RA0_OD                   ODCONAbits.ODA0
#define IO_RA0_ANS                  ANSELAbits.ANSA0
#define IO_RA0_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define IO_RA0_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define IO_RA0_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define IO_RA0_GetValue()           PORTAbits.RA0
#define IO_RA0_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define IO_RA0_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define IO_RA0_SetPullup()          do { WPUAbits.WPUA0 = 1; } while(0)
#define IO_RA0_ResetPullup()        do { WPUAbits.WPUA0 = 0; } while(0)
#define IO_RA0_SetPushPull()        do { ODCONAbits.ODA0 = 0; } while(0)
#define IO_RA0_SetOpenDrain()       do { ODCONAbits.ODA0 = 1; } while(0)
#define IO_RA0_SetAnalogMode()      do { ANSELAbits.ANSA0 = 1; } while(0)
#define IO_RA0_SetDigitalMode()     do { ANSELAbits.ANSA0 = 0; } while(0)
// get/set IO_RA1 aliases
#define LEDG_TRIS                 TRISAbits.TRISA1
#define LEDG_LAT                  LATAbits.LATA1
#define LEDG_PORT                 PORTAbits.RA1
#define LEDG_WPU                  WPUAbits.WPUA1
#define LEDG_OD                   ODCONAbits.ODA1
#define LEDG_ANS                  ANSELAbits.ANSA1
#define LEDG_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define LEDG_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define LEDG_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define LEDG_GetValue()           PORTAbits.RA1
#define LEDG_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define LEDG_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define LEDG_SetPullup()          do { WPUAbits.WPUA1 = 1; } while(0)
#define LEDG_ResetPullup()        do { WPUAbits.WPUA1 = 0; } while(0)
#define LEDG_SetPushPull()        do { ODCONAbits.ODA1 = 0; } while(0)
#define LEDG_SetOpenDrain()       do { ODCONAbits.ODA1 = 1; } while(0)
#define LEDG_SetAnalogMode()      do { ANSELAbits.ANSA1 = 1; } while(0)
#define LEDG_SetDigitalMode()     do { ANSELAbits.ANSA1 = 0; } while(0)
// get/set IO_RA2 aliases
#define Servo_TRIS                 TRISAbits.TRISA2
#define Servo_LAT                  LATAbits.LATA2
#define Servo_PORT                 PORTAbits.RA2
#define Servo_WPU                  WPUAbits.WPUA2
#define Servo_OD                   ODCONAbits.ODA2
#define Servo_ANS                  ANSELAbits.ANSA2
#define Servo_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define Servo_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define Servo_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define Servo_GetValue()           PORTAbits.RA2
#define Servo_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define Servo_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define Servo_SetPullup()          do { WPUAbits.WPUA2 = 1; } while(0)
#define Servo_ResetPullup()        do { WPUAbits.WPUA2 = 0; } while(0)
#define Servo_SetPushPull()        do { ODCONAbits.ODA2 = 0; } while(0)
#define Servo_SetOpenDrain()       do { ODCONAbits.ODA2 = 1; } while(0)
#define Servo_SetAnalogMode()      do { ANSELAbits.ANSA2 = 1; } while(0)
#define Servo_SetDigitalMode()     do { ANSELAbits.ANSA2 = 0; } while(0)
// get/set IO_RA5 aliases
#define LEDR_TRIS                 TRISAbits.TRISA5
#define LEDR_LAT                  LATAbits.LATA5
#define LEDR_PORT                 PORTAbits.RA5
#define LEDR_WPU                  WPUAbits.WPUA5
#define LEDR_OD                   ODCONAbits.ODA5
#define LEDR_ANS                  ANSELAbits.
#define LEDR_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define LEDR_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define LEDR_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define LEDR_GetValue()           PORTAbits.RA5
#define LEDR_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define LEDR_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define LEDR_SetPullup()          do { WPUAbits.WPUA5 = 1; } while(0)
#define LEDR_ResetPullup()        do { WPUAbits.WPUA5 = 0; } while(0)
#define LEDR_SetPushPull()        do { ODCONAbits.ODA5 = 0; } while(0)
#define LEDR_SetOpenDrain()       do { ODCONAbits.ODA5 = 1; } while(0)
#define LEDR_SetAnalogMode()      do { ANSELAbits. = 1; } while(0)
#define LEDR_SetDigitalMode()     do { ANSELAbits. = 0; } while(0)
// get/set IO_RB4 aliases
#define SR_DATA_TRIS                 TRISBbits.TRISB4
#define SR_DATA_LAT                  LATBbits.LATB4
#define SR_DATA_PORT                 PORTBbits.RB4
#define SR_DATA_WPU                  WPUBbits.WPUB4
#define SR_DATA_OD                   ODCONBbits.ODB4
#define SR_DATA_ANS                  ANSELBbits.ANSB4
#define SR_DATA_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define SR_DATA_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define SR_DATA_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define SR_DATA_GetValue()           PORTBbits.RB4
#define SR_DATA_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define SR_DATA_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define SR_DATA_SetPullup()          do { WPUBbits.WPUB4 = 1; } while(0)
#define SR_DATA_ResetPullup()        do { WPUBbits.WPUB4 = 0; } while(0)
#define SR_DATA_SetPushPull()        do { ODCONBbits.ODB4 = 0; } while(0)
#define SR_DATA_SetOpenDrain()       do { ODCONBbits.ODB4 = 1; } while(0)
#define SR_DATA_SetAnalogMode()      do { ANSELBbits.ANSB4 = 1; } while(0)
#define SR_DATA_SetDigitalMode()     do { ANSELBbits.ANSB4 = 0; } while(0)
// get/set IO_RB5 aliases
#define SR_SEND_TRIS                 TRISBbits.TRISB5
#define SR_SEND_LAT                  LATBbits.LATB5
#define SR_SEND_PORT                 PORTBbits.RB5
#define SR_SEND_WPU                  WPUBbits.WPUB5
#define SR_SEND_OD                   ODCONBbits.ODB5
#define SR_SEND_ANS                  ANSELBbits.ANSB5
#define SR_SEND_SetHigh()            do { LATBbits.LATB5 = 1; } while(0)
#define SR_SEND_SetLow()             do { LATBbits.LATB5 = 0; } while(0)
#define SR_SEND_Toggle()             do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define SR_SEND_GetValue()           PORTBbits.RB5
#define SR_SEND_SetDigitalInput()    do { TRISBbits.TRISB5 = 1; } while(0)
#define SR_SEND_SetDigitalOutput()   do { TRISBbits.TRISB5 = 0; } while(0)
#define SR_SEND_SetPullup()          do { WPUBbits.WPUB5 = 1; } while(0)
#define SR_SEND_ResetPullup()        do { WPUBbits.WPUB5 = 0; } while(0)
#define SR_SEND_SetPushPull()        do { ODCONBbits.ODB5 = 0; } while(0)
#define SR_SEND_SetOpenDrain()       do { ODCONBbits.ODB5 = 1; } while(0)
#define SR_SEND_SetAnalogMode()      do { ANSELBbits.ANSB5 = 1; } while(0)
#define SR_SEND_SetDigitalMode()     do { ANSELBbits.ANSB5 = 0; } while(0)
// get/set IO_RB6 aliases
#define SR_CLOCK_TRIS                 TRISBbits.TRISB6
#define SR_CLOCK_LAT                  LATBbits.LATB6
#define SR_CLOCK_PORT                 PORTBbits.RB6
#define SR_CLOCK_WPU                  WPUBbits.WPUB6
#define SR_CLOCK_OD                   ODCONBbits.ODB6
#define SR_CLOCK_ANS                  ANSELBbits.ANSB6
#define SR_CLOCK_SetHigh()            do { LATBbits.LATB6 = 1; } while(0)
#define SR_CLOCK_SetLow()             do { LATBbits.LATB6 = 0; } while(0)
#define SR_CLOCK_Toggle()             do { LATBbits.LATB6 = ~LATBbits.LATB6; } while(0)
#define SR_CLOCK_GetValue()           PORTBbits.RB6
#define SR_CLOCK_SetDigitalInput()    do { TRISBbits.TRISB6 = 1; } while(0)
#define SR_CLOCK_SetDigitalOutput()   do { TRISBbits.TRISB6 = 0; } while(0)
#define SR_CLOCK_SetPullup()          do { WPUBbits.WPUB6 = 1; } while(0)
#define SR_CLOCK_ResetPullup()        do { WPUBbits.WPUB6 = 0; } while(0)
#define SR_CLOCK_SetPushPull()        do { ODCONBbits.ODB6 = 0; } while(0)
#define SR_CLOCK_SetOpenDrain()       do { ODCONBbits.ODB6 = 1; } while(0)
#define SR_CLOCK_SetAnalogMode()      do { ANSELBbits.ANSB6 = 1; } while(0)
#define SR_CLOCK_SetDigitalMode()     do { ANSELBbits.ANSB6 = 0; } while(0)
// get/set IO_RC1 aliases
#define Echo_TRIS                 TRISCbits.TRISC1
#define Echo_LAT                  LATCbits.LATC1
#define Echo_PORT                 PORTCbits.RC1
#define Echo_WPU                  WPUCbits.WPUC1
#define Echo_OD                   ODCONCbits.ODC1
#define Echo_ANS                  ANSELCbits.ANSC1
#define Echo_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define Echo_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define Echo_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define Echo_GetValue()           PORTCbits.RC1
#define Echo_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define Echo_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define Echo_SetPullup()          do { WPUCbits.WPUC1 = 1; } while(0)
#define Echo_ResetPullup()        do { WPUCbits.WPUC1 = 0; } while(0)
#define Echo_SetPushPull()        do { ODCONCbits.ODC1 = 0; } while(0)
#define Echo_SetOpenDrain()       do { ODCONCbits.ODC1 = 1; } while(0)
#define Echo_SetAnalogMode()      do { ANSELCbits.ANSC1 = 1; } while(0)
#define Echo_SetDigitalMode()     do { ANSELCbits.ANSC1 = 0; } while(0)
// get/set IO_RC2 aliases
#define Trig_TRIS                 TRISCbits.TRISC2
#define Trig_LAT                  LATCbits.LATC2
#define Trig_PORT                 PORTCbits.RC2
#define Trig_WPU                  WPUCbits.WPUC2
#define Trig_OD                   ODCONCbits.ODC2
#define Trig_ANS                  ANSELCbits.ANSC2
#define Trig_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define Trig_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define Trig_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define Trig_GetValue()           PORTCbits.RC2
#define Trig_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define Trig_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define Trig_SetPullup()          do { WPUCbits.WPUC2 = 1; } while(0)
#define Trig_ResetPullup()        do { WPUCbits.WPUC2 = 0; } while(0)
#define Trig_SetPushPull()        do { ODCONCbits.ODC2 = 0; } while(0)
#define Trig_SetOpenDrain()       do { ODCONCbits.ODC2 = 1; } while(0)
#define Trig_SetAnalogMode()      do { ANSELCbits.ANSC2 = 1; } while(0)
#define Trig_SetDigitalMode()     do { ANSELCbits.ANSC2 = 0; } while(0)
// get/set IO_RC3 aliases
#define SDA_TRIS                 TRISCbits.TRISC3
#define SDA_LAT                  LATCbits.LATC3
#define SDA_PORT                 PORTCbits.RC3
#define SDA_WPU                  WPUCbits.WPUC3
#define SDA_OD                   ODCONCbits.ODC3
#define SDA_ANS                  ANSELCbits.ANSC3
#define SDA_SetHigh()            do { LATCbits.LATC3 = 1; } while(0)
#define SDA_SetLow()             do { LATCbits.LATC3 = 0; } while(0)
#define SDA_Toggle()             do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define SDA_GetValue()           PORTCbits.RC3
#define SDA_SetDigitalInput()    do { TRISCbits.TRISC3 = 1; } while(0)
#define SDA_SetDigitalOutput()   do { TRISCbits.TRISC3 = 0; } while(0)
#define SDA_SetPullup()          do { WPUCbits.WPUC3 = 1; } while(0)
#define SDA_ResetPullup()        do { WPUCbits.WPUC3 = 0; } while(0)
#define SDA_SetPushPull()        do { ODCONCbits.ODC3 = 0; } while(0)
#define SDA_SetOpenDrain()       do { ODCONCbits.ODC3 = 1; } while(0)
#define SDA_SetAnalogMode()      do { ANSELCbits.ANSC3 = 1; } while(0)
#define SDA_SetDigitalMode()     do { ANSELCbits.ANSC3 = 0; } while(0)
// get/set IO_RC4 aliases
#define SCL_pin_TRIS                 TRISCbits.TRISC4
#define SCL_pin_LAT                  LATCbits.LATC4
#define SCL_pin_PORT                 PORTCbits.RC4
#define SCL_pin_WPU                  WPUCbits.WPUC4
#define SCL_pin_OD                   ODCONCbits.ODC4
#define SCL_pin_ANS                  ANSELCbits.
#define SCL_pin_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define SCL_pin_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define SCL_pin_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define SCL_pin_GetValue()           PORTCbits.RC4
#define SCL_pin_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define SCL_pin_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define SCL_pin_SetPullup()          do { WPUCbits.WPUC4 = 1; } while(0)
#define SCL_pin_ResetPullup()        do { WPUCbits.WPUC4 = 0; } while(0)
#define SCL_pin_SetPushPull()        do { ODCONCbits.ODC4 = 0; } while(0)
#define SCL_pin_SetOpenDrain()       do { ODCONCbits.ODC4 = 1; } while(0)
#define SCL_pin_SetAnalogMode()      do { ANSELCbits. = 1; } while(0)
#define SCL_pin_SetDigitalMode()     do { ANSELCbits. = 0; } while(0)
// get/set IO_RC5 aliases
#define LEDY_TRIS                 TRISCbits.TRISC5
#define LEDY_LAT                  LATCbits.LATC5
#define LEDY_PORT                 PORTCbits.RC5
#define LEDY_WPU                  WPUCbits.WPUC5
#define LEDY_OD                   ODCONCbits.ODC5
#define LEDY_ANS                  ANSELCbits.
#define LEDY_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define LEDY_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define LEDY_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define LEDY_GetValue()           PORTCbits.RC5
#define LEDY_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define LEDY_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define LEDY_SetPullup()          do { WPUCbits.WPUC5 = 1; } while(0)
#define LEDY_ResetPullup()        do { WPUCbits.WPUC5 = 0; } while(0)
#define LEDY_SetPushPull()        do { ODCONCbits.ODC5 = 0; } while(0)
#define LEDY_SetOpenDrain()       do { ODCONCbits.ODC5 = 1; } while(0)
#define LEDY_SetAnalogMode()      do { ANSELCbits. = 1; } while(0)
#define LEDY_SetDigitalMode()     do { ANSELCbits. = 0; } while(0)
// get/set IO_RC6 aliases
#define TX_pin_TRIS                 TRISCbits.TRISC6
#define TX_pin_LAT                  LATCbits.LATC6
#define TX_pin_PORT                 PORTCbits.RC6
#define TX_pin_WPU                  WPUCbits.WPUC6
#define TX_pin_OD                   ODCONCbits.ODC6
#define TX_pin_ANS                  ANSELCbits.ANSC6
#define TX_pin_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define TX_pin_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define TX_pin_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define TX_pin_GetValue()           PORTCbits.RC6
#define TX_pin_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define TX_pin_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
#define TX_pin_SetPullup()          do { WPUCbits.WPUC6 = 1; } while(0)
#define TX_pin_ResetPullup()        do { WPUCbits.WPUC6 = 0; } while(0)
#define TX_pin_SetPushPull()        do { ODCONCbits.ODC6 = 0; } while(0)
#define TX_pin_SetOpenDrain()       do { ODCONCbits.ODC6 = 1; } while(0)
#define TX_pin_SetAnalogMode()      do { ANSELCbits.ANSC6 = 1; } while(0)
#define TX_pin_SetDigitalMode()     do { ANSELCbits.ANSC6 = 0; } while(0)
// get/set IO_RC7 aliases
#define RX_pin_TRIS                 TRISCbits.TRISC7
#define RX_pin_LAT                  LATCbits.LATC7
#define RX_pin_PORT                 PORTCbits.RC7
#define RX_pin_WPU                  WPUCbits.WPUC7
#define RX_pin_OD                   ODCONCbits.ODC7
#define RX_pin_ANS                  ANSELCbits.ANSC7
#define RX_pin_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define RX_pin_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define RX_pin_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define RX_pin_GetValue()           PORTCbits.RC7
#define RX_pin_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define RX_pin_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
#define RX_pin_SetPullup()          do { WPUCbits.WPUC7 = 1; } while(0)
#define RX_pin_ResetPullup()        do { WPUCbits.WPUC7 = 0; } while(0)
#define RX_pin_SetPushPull()        do { ODCONCbits.ODC7 = 0; } while(0)
#define RX_pin_SetOpenDrain()       do { ODCONCbits.ODC7 = 1; } while(0)
#define RX_pin_SetAnalogMode()      do { ANSELCbits.ANSC7 = 1; } while(0)
#define RX_pin_SetDigitalMode()     do { ANSELCbits.ANSC7 = 0; } while(0)
/**
 * @ingroup  pinsdriver
 * @brief GPIO and peripheral I/O initialization
 * @param none
 * @return none
 */
void PIN_MANAGER_Initialize (void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt on Change Handling routine
 * @param none
 * @return none
 */
void PIN_MANAGER_IOC(void);


#endif // PINS_H
/**
 End of File
*/