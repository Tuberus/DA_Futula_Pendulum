#pragma once
#include <algorithm>

class PendulumPID {
public:
    float kp = 0.002f;
    float ki = 0.0f;
    float kd = 0.0f;
    float setpoint = 0.0f;
    float outMax = 0.35f;

    void reset() {
        _integral = 0.0f;
        _lastError = 0.0f;
    }

    float compute(float angle, float dt) {
        float error = setpoint - angle;

        _integral += error * dt;
        _integral = std::clamp(_integral, -outMax, outMax);

        float deriv = (error - _lastError) / dt;
        _lastError = error;

        float out = kp * error + ki * _integral + kd * deriv;
        return std::clamp(out, -outMax, outMax);
    }

private:
    float _integral = 0.0f;
    float _lastError = 0.0f;
};