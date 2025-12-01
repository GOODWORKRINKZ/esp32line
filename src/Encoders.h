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
    
    // Накопительные счётчики тиков (не сбрасываются в update())
    long totalLeftTicks;
    long totalRightTicks;
    
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
    
    // Получить НАКОПЛЕННОЕ количество тиков (для контроля поворота)
    long getLeftTicks() const;
    long getRightTicks() const;
    
    // Сбросить накопительные счетчики
    void resetTicks();
    
    // Полный сброс всех данных (для перезапуска)
    void resetAll();
};

#endif // ENCODERS_H
