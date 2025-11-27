#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <Arduino.h>
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "PIDController.h"

// Forward declaration
class Encoders;

// Состояния робота
enum RobotState {
    IDLE,              // Ожидание
    CALIBRATING,       // Калибровка датчиков
    FOLLOWING,         // Следование по линии
    SEARCHING_LEFT,    // Поиск линии влево
    SEARCHING_RIGHT,   // Поиск линии вправо
    LOST,              // Линия потеряна
    STOPPED            // Остановлен
};

// Класс для управления роботом, следующим по линии
class LineFollower {
private:
    LineSensors& sensors;
    Motors& motors;
    PIDController& pid;
    Encoders* encoders;  // Всегда указатель, может быть nullptr
    
    RobotState currentState;
    int baseSpeed;
    unsigned long searchStartTime;
    
public:
    // Конструктор с опциональным параметром энкодеров
    LineFollower(LineSensors& s, Motors& m, PIDController& p, Encoders* e = nullptr);
    
    // Инициализация
    void begin();
    
    // Основной цикл обработки
    void update();
    
    // Управление состояниями
    void start();
    void pause();
    void stop();
    void calibrate();
    
    // Получить текущее состояние
    RobotState getState() const { return currentState; }
    
    // Управление скоростью
    void increaseSpeed();
    void decreaseSpeed();
    int getBaseSpeed() const { return baseSpeed; }
    
private:
    // Внутренние методы
    void followLine();
    void searchLine();
};

#endif // LINE_FOLLOWER_H
