#ifndef _BUTTON_H_
#define _BUTTON_H_

#include <Arduino.h>

class BUTTON {
public:
    BUTTON(byte pin, byte mode);
    bool pressed();
private:
    byte _pin;
    byte _mode;
    bool _pressed;
};

#endif