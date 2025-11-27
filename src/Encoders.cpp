#include "Encoders.h"

// Инициализация статических переменных
volatile long Encoders::leftTicks = 0;
volatile long Encoders::rightTicks = 0;
portMUX_TYPE Encoders::timerMux = portMUX_INITIALIZER_UNLOCKED;

Encoders::Encoders() : lastUpdateTime(0), leftSpeed(0.0), rightSpeed(0.0),
                       totalLeftTicks(0), totalRightTicks(0) {
}

void Encoders::begin() {
#ifdef USE_ENCODERS
    pinMode(ENCODER_LEFT, INPUT);
    pinMode(ENCODER_RIGHT, INPUT);
    
    // ESP32 поддерживает прерывания на всех GPIO
    attachInterrupt(ENCODER_LEFT, leftISR, RISING);
    attachInterrupt(ENCODER_RIGHT, rightISR, RISING);
#endif
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
        
        // НАКАПЛИВАЕМ тики для контроля поворота
        totalLeftTicks += leftTicksLocal;
        totalRightTicks += rightTicksLocal;
        
        // Вычисляем пройденное расстояние
        float leftDistance = leftTicksLocal * MM_PER_TICK;
        float rightDistance = rightTicksLocal * MM_PER_TICK;
        
        // Вычисляем скорости (мм/сек)
        leftSpeed = (leftDistance / deltaTime) * 1000.0;
        rightSpeed = (rightDistance / deltaTime) * 1000.0;
        
        lastUpdateTime = currentTime;
    }
}

float Encoders::getLeftSpeed() const {
    return leftSpeed;
}

float Encoders::getRightSpeed() const {
    return rightSpeed;
}

long Encoders::getLeftTicks() const {
    // Возвращаем НАКОПЛЕННЫЕ тики
    return totalLeftTicks;
}

long Encoders::getRightTicks() const {
    // Возвращаем НАКОПЛЕННЫЕ тики
    return totalRightTicks;
}

void Encoders::resetTicks() {
    portENTER_CRITICAL(&timerMux);
    leftTicks = 0;
    rightTicks = 0;
    portEXIT_CRITICAL(&timerMux);
    
    // Сбрасываем накопительные счётчики
    totalLeftTicks = 0;
    totalRightTicks = 0;
}

void IRAM_ATTR Encoders::leftISR() {
    leftTicks++;
}

void IRAM_ATTR Encoders::rightISR() {
    rightTicks++;
}
