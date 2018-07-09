#include "PID.h"

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {
}

PID::~PID() {
}

// 初始化PID参数
void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

// 更新误差
void PID::UpdateError(double cte) {
  p_error = cte;
  i_error = i_error + cte;
  d_error = cte - d_error;
}

// 计算总误差
double PID::TotalError() {
  return Kp * p_error + Ki * i_error + Kd * d_error;
}

