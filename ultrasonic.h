#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

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

#endif // ULTRASONIC_H_
