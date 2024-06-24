/**
 * Generated Driver File
 * 
 * @file pins.c
 * 
 * @ingroup  pinsdriver
 * 
 * @brief This is generated driver implementation for pins. 
 *        This file provides implementations for pin APIs for all pins selected in the GUI.
 *
 * @version Driver Version 3.0.0
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

#include "../pins.h"


void PIN_MANAGER_Initialize(void)
{
   /**
    LATx registers
    */
    LATA = 0x0;
    LATB = 0x0;
    LATC = 0x18;

    /**
    TRISx registers
    */
    TRISA = 0x19;
    TRISB = 0x80;
    TRISC = 0x9B;

    /**
    ANSELx registers
    */
    ANSELA = 0x15;
    ANSELB = 0x0;
    ANSELC = 0x1;

    /**
    WPUx registers
    */
    WPUA = 0x3A;
    WPUB = 0xF0;
    WPUC = 0xDF;
    OPTION_REGbits.nWPUEN = 0x0;
  
    /**
    ODx registers
    */
   
    ODCONA = 0x0;
    ODCONB = 0x0;
    ODCONC = 0x0;
    /**
    SLRCONx registers
    */
    SLRCONA = 0x36;
    SLRCONB = 0xF0;
    SLRCONC = 0xFF;
    /**
    INLVLx registers
    */
    INLVLA = 0x18;
    INLVLB = 0xF0;
    INLVLC = 0x7;

    /**
    PPS registers
    */
    RXPPS = 0x17; //RC7->EUSART:RX;
    RC6PPS = 18;  //RC6->EUSART:TX;
    SSPCLKPPS = 0x14;  //RC4->MSSP:SCL;
    RC4PPS = 16;  //RC4->MSSP:SCL;
    SSPDATPPS = 0x13;  //RC3->MSSP:SDA;
    RC3PPS = 17;  //RC3->MSSP:SDA;
    CCP2PPS = 0x2;  //RA2->CCP2:CCP2;
    RA2PPS = 13;  //RA2->CCP2:CCP2;
    CCP1PPS = 0x15;  //RC5->CCP1:CCP1;
    RC5PPS = 12;  //RC5->CCP1:CCP1;

    /**
    APFCON registers
    */

   /**
    IOCx registers 
    */
    IOCAP = 0x0;
    IOCAN = 0x0;
    IOCAF = 0x0;
    IOCBP = 0x0;
    IOCBN = 0x0;
    IOCBF = 0x0;
    IOCCP = 0x0;
    IOCCN = 0x0;
    IOCCF = 0x0;


}
  
void PIN_MANAGER_IOC(void)
{
}
/**
 End of File
*/