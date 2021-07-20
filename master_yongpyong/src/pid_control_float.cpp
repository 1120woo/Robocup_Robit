#include "../include/master_yongpyong/pid_control_float.h"
#include <iostream>

using namespace std;

void PID_Control_init(PID* temp,double TP, double TI, double TD ,double ELIMIT, double OLIMIT)
{
  temp->kP = TP;
  temp->kI = TI;
  temp->kD = TD;
  temp->errorSumLimit = ELIMIT;
  temp->outputLimit = OLIMIT;
  temp->underOfPoint = 100.0;
}

void PID_Control_Float(PID* dst, double target, double input)
{
  dst->nowValue = input;
  dst->target = target;

  dst->nowError = dst->nowValue - dst->target;
  dst->errorSum += dst->nowError;
  dst->errorDiff = dst->nowError - dst->pastError;
  if(dst->errorSumLimit !=0)
  {
    if(dst->errorSum > dst->errorSumLimit)
      dst->errorSum = dst->errorSumLimit;
    else if(dst->errorSum < -dst->errorSumLimit)
      dst->errorSum = -dst->errorSumLimit;
  }
    dst->nowOutput = (double)(dst->kP * dst->nowError + dst->kI * dst->errorSum + dst->kD * dst->errorDiff);

    if(dst->underOfPoint == 0) return;

  dst->nowOutput /= dst -> underOfPoint;

  dst->pastError = dst->nowError;

  //cout<< "nowOutput = " << dst->nowOutput << endl;

  if(dst->nowOutput > dst->outputLimit) dst->nowOutput = dst->outputLimit;
  else if(dst->nowOutput < -dst->outputLimit) dst->nowOutput = -dst->outputLimit;

}
