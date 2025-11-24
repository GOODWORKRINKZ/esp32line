#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "Config.h"

// Класс для работы с датчиками линии
class LineSensors {
private:
    int sensorMin[5];
    int sensorMax[5];
    
public:
    LineSensors();
    
    // Инициализация датчиков
    void begin();
    
    // Чтение значений с датчиков
    void read(int sensors[5]);
    
    // Вычисление позиции линии (-2.0 до +2.0, или -999 если не найдена)
    float calculatePosition(int sensors[5]);
    
    // Калибровка датчиков
    void calibrate();
    
    // Получить минимальные значения
    void getMin(int min[5]) { 
        for(int i = 0; i < 5; i++) min[i] = sensorMin[i]; 
    }
    
    // Получить максимальные значения
    void getMax(int max[5]) { 
        for(int i = 0; i < 5; i++) max[i] = sensorMax[i]; 
    }
};

#endif // SENSORS_H
