#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "Config.h"

// Класс для управления моторами
class Motors {
public:
    Motors();
    
    // Инициализация моторов
    void begin();
    
    // Установить скорость моторов (от -255 до +255)
    void setSpeed(int leftSpeed, int rightSpeed);
    
    // Остановка моторов
    void stop();
    
    // Движение вперед
    void moveForward(int speed);
    
    // Движение назад
    void moveBackward(int speed);
    
    // Поворот влево
    void turnLeft(int speed);
    
    // Поворот вправо
    void turnRight(int speed);
};

#endif // MOTORS_H
