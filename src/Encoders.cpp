#include "Encoders.h"

#ifdef USE_ENCODERS

// Инициализация статических переменных
volatile long Encoders::leftTicks = 0;
volatile long Encoders::rightTicks = 0;
portMUX_TYPE Encoders::timerMux = portMUX_INITIALIZER_UNLOCKED;

Encoders::Encoders() : lastUpdateTime(0), leftSpeed(0.0), rightSpeed(0.0) {
}

void Encoders::begin() {
    pinMode(ENCODER_LEFT, INPUT);
    pinMode(ENCODER_RIGHT, INPUT);
    
    // ESP32 поддерживает прерывания на всех GPIO
    attachInterrupt(ENCODER_LEFT, leftISR, RISING);
    attachInterrupt(ENCODER_RIGHT, rightISR, RISING);
}

void Encoders::update() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastUpdateTime >= 100) {  // Обновление каждые 100мс
        unsigned long deltaTime = currentTime - lastUpdateTime;
        
        // Безопасное чтение volatile переменных
        long leftTicksLocal, rightTicksLocal;
        portENTER_CRITICAL(&timerMux);
        leftTicksLocal = leftTicks;
        rightTicksLocal = rightTicks;
        leftTicks = 0;
        rightTicks = 0;
        portEXIT_CRITICAL(&timerMux);
        
        // Вычисляем пройденное расстояние
        float leftDistance = leftTicksLocal * MM_PER_TICK;
        float rightDistance = rightTicksLocal * MM_PER_TICK;
        
        // Вычисляем скорости (мм/сек)
        leftSpeed = (leftDistance / deltaTime) * 1000.0;
        rightSpeed = (rightDistance / deltaTime) * 1000.0;
        
        lastUpdateTime = currentTime;
    }
}

long Encoders::getLeftTicks() {
    portENTER_CRITICAL(&timerMux);
    long ticks = leftTicks;
    portEXIT_CRITICAL(&timerMux);
    return ticks;
}

long Encoders::getRightTicks() {
    portENTER_CRITICAL(&timerMux);
    long ticks = rightTicks;
    portEXIT_CRITICAL(&timerMux);
    return ticks;
}

void Encoders::resetTicks() {
    portENTER_CRITICAL(&timerMux);
    leftTicks = 0;
    rightTicks = 0;
    portEXIT_CRITICAL(&timerMux);
}

void IRAM_ATTR Encoders::leftISR() {
    leftTicks++;
}

void IRAM_ATTR Encoders::rightISR() {
    rightTicks++;
}

#endif // USE_ENCODERS
