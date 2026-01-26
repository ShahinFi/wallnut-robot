#include "encoder.h"

static const int ENC_L_A_PIN = 2;
static const int ENC_R_A_PIN = 3;

volatile long encLeft = 0;
volatile long encRight = 0;

static void isrLeft()  { encLeft++; }
static void isrRight() { encRight++; }

void encoderInit() {
  pinMode(ENC_L_A_PIN, INPUT_PULLUP);
  pinMode(ENC_R_A_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A_PIN), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A_PIN), isrRight, RISING);
}

long encoderGetLeft() {
  noInterrupts();
  long v = encLeft;
  interrupts();
  return v;
}

long encoderGetRight() {
  noInterrupts();
  long v = encRight;
  interrupts();
  return v;
}

void encoderReset() {
  noInterrupts();
  encLeft = 0;
  encRight = 0;
  interrupts();
}
