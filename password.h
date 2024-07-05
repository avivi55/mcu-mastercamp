#ifndef PASSWORD_H_
#define PASSWORD_H_

uint8_t password_validation_index = 0;

#define PASSWORD_LEN 5
uint8_t password[PASSWORD_LEN] = {
    0x6a,
    0x7c,
    0xee,
    0xe0,
    0xb0
};

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

#endif // PASSWORD_H_
