#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "Config.h"

// Класс ПИД-регулятора
class PIDController {
private:
    float kp;
    float ki;
    float kd;
    float previousError;
    float integral;
    
public:
    PIDController(float p = DEFAULT_KP, float i = DEFAULT_KI, float d = DEFAULT_KD);
    
    // Вычислить корректировку на основе ошибки
    float calculate(float error);
    
    // Сбросить интеграл и предыдущую ошибку
    void reset();
    
    // Установить коэффициенты
    void setGains(float p, float i, float d);
    
    // Получить коэффициенты
    void getGains(float &p, float &i, float &d) const {
        p = kp; i = ki; d = kd;
    }
    
    // Получить предыдущую ошибку (для агрессивного PID)
    float getPreviousError() const { return previousError; }
};

#endif // PID_CONTROLLER_H
