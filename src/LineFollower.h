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
    WAITING_FOR_TURN,  // Ожидание обновления энкодеров перед поворотом
    TURNING,           // Поворот на месте (с контролем энкодеров)
    SEARCHING_LEFT,    // Поиск линии влево
    SEARCHING_RIGHT,   // Поиск линии вправо
    LOST,              // Линия потеряна
    STOPPED            // Остановлен
};

// Направление поворота
enum TurnDirection {
    TURN_NONE,
    TURN_LEFT,
    TURN_RIGHT
};

// Класс для управления роботом, следующим по линии
class LineFollower {
private:
    LineSensors& sensors;
    Motors& motors;
    PIDController& pid;
    Encoders* encoders;  // Указатель на энкодеры (может быть nullptr)
    
    RobotState currentState;
    int baseSpeed;
    unsigned long searchStartTime;
    
    // Переменные для поворота с энкодерами
    TurnDirection turnDirection;
    long turnStartTicksLeft;
    long turnStartTicksRight;
    float targetTurnDegrees;
    unsigned long waitStartTime;  // Время начала ожидания перед поворотом
    
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
    void executeTurn();  // Выполнение поворота с контролем энкодеров
    void startTurn(TurnDirection dir, float degrees);  // Начать поворот
};

#endif // LINE_FOLLOWER_H
