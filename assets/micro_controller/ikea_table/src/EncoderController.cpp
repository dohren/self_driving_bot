#include <Arduino.h>
#include "EncoderController.h"

EncoderController::EncoderController(int leftA, int leftB, int rightA, int rightB)
    : pinLeftA(leftA), pinLeftB(leftB), pinRightA(rightA), pinRightB(rightB)
{
}

void EncoderController::begin() {
    // Interne Pullups aktivieren (ggf. anpassen)
    ESP32Encoder::useInternalWeakPullResistors = UP;

    // Encoder an Pins anhängen (Halb-Quadratur)
    encoderLeft.attachHalfQuad(pinLeftA, pinLeftB);
    encoderRight.attachHalfQuad(pinRightA, pinRightB);

    // Zähler auf 0 setzen
    encoderLeft.clearCount();
    encoderRight.clearCount();

    Serial.println("EncoderController initialisiert.");
    Serial.print("Linker Encoder: ");
    Serial.println(encoderLeft.getCount());
    Serial.print("Rechter Encoder: ");
    Serial.println(encoderRight.getCount());
}

long EncoderController::getLeftCount() {
    return encoderLeft.getCount();
}

long EncoderController::getRightCount() {
    return encoderRight.getCount();
}

