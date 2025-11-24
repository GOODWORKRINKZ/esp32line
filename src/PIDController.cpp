#include "PIDController.h"

PIDController::PIDController(float p, float i, float d) 
    : kp(p), ki(i), kd(d), previousError(0.0), integral(0.0) {
}

float PIDController::calculate(float error) {
    /*
     * Вычисляет корректировку рулевого управления на основе ошибки
     * error: -2.0 (линия слева) до +2.0 (линия справа)
     * Возврат: корректировка для моторов
     */
    
    // P - пропорциональная составляющая
    float P = error;
    
    // I - интегральная составляющая (накопленная ошибка)
    integral += error;
    // Анти-windup: ограничиваем интеграл
    if (integral > 100) integral = 100;
    if (integral < -100) integral = -100;
    float I = integral;
    
    // D - дифференциальная составляющая (скорость изменения ошибки)
    float D = error - previousError;
    previousError = error;
    
    // Итоговая корректировка
    return kp * P + ki * I + kd * D;
}

void PIDController::reset() {
    previousError = 0.0;
    integral = 0.0;
}

void PIDController::setGains(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
}
