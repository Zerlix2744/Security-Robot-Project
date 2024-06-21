#include "PinZone.h"

void setupPinOUTPUT(){
  pinMode(pinML_INA, OUTPUT);
  pinMode(pinML_INB, OUTPUT);

  pinMode(pinMR_INA, OUTPUT);
  pinMode(pinMR_INB, OUTPUT);
}

void motorL(int pwm) {
  if (pwm == 0) {
    analogWrite(pinML_INA, 255);
    analogWrite(pinML_INB, 255);
  } else if (pwm > 0) {
    analogWrite(pinML_INA, pwm);
    analogWrite(pinML_INB, 0);
  } else if (pwm < 0) {
    analogWrite(pinML_INA, 0);
    analogWrite(pinML_INB, abs(pwm));
  }
}
void motorR(int pwm) {
  if (pwm == 0) {
    analogWrite(pinMR_INA, 255);
    analogWrite(pinMR_INB, 255);
  } else if (pwm > 0) {
    analogWrite(pinMR_INA, 0);
    analogWrite(pinMR_INB, pwm);
  } else if (pwm < 0) {
    analogWrite(pinMR_INA, abs(pwm));
    analogWrite(pinMR_INB, 0);
  }
}