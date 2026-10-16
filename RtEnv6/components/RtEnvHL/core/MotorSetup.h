#pragma once

#include "RTEnvHL.h"
#include "EspMotor.h"
#include "Encoder2.h"

const int PWA = 25, AI1 = 26, AI2 = 27;
const int ENC_A = 32, ENC_B = 33;

Motor mot(PWA, AI1, AI2);
Encoder2 enc(ENC_A, ENC_B);

void InitMotors()
{
  Motor::InitTimer();
  mot.Init();
  Motor::StartTimer();
  mot.setPow2(0.0);

  enc.Init();

  mot.inv = false;
  enc.inv = false;
}