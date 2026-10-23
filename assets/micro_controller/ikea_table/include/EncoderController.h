#ifndef ENCODER_CONTROLLER_H
#define ENCODER_CONTROLLER_H

#include <ESP32Encoder.h>

class EncoderController {
public:
    EncoderController(int leftA, int leftB, int rightA, int rightB);

    void begin();

    long getLeftCount();
    long getRightCount();

private:
    ESP32Encoder encoderLeft;
    ESP32Encoder encoderRight;

    int pinLeftA;
    int pinLeftB;
    int pinRightA;
    int pinRightB;
};

#endif
