#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>
#include "Config.h"

// Класс для работы с энкодерами
class Encoders {
private:
    static volatile long leftTicks;
    static volatile long rightTicks;
    static portMUX_TYPE timerMux;
    
    unsigned long lastUpdateTime;
    float leftSpeed;   // мм/сек
    float rightSpeed;  // мм/сек
    
    // ISR функции должны быть static
    static void IRAM_ATTR leftISR();
    static void IRAM_ATTR rightISR();
    
public:
    Encoders();
    
    // Инициализация энкодеров
    void begin();
    
    // Обновление скоростей
    void update();
    
    // Получить скорости
    float getLeftSpeed() const;
    float getRightSpeed() const;
    
    // Получить количество тиков
    long getLeftTicks();
    long getRightTicks();
    
    // Сбросить счетчики
    void resetTicks();
};

#endif // ENCODERS_H
