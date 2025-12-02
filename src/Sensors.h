#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "Config.h"

// Класс для работы с датчиками линии
class LineSensors {
private:
    int sensorMin[5];
    int sensorMax[5];
    
    // Последняя известная позиция линии (для случаев когда линия между датчиками)
    float lastKnownPosition;
    unsigned long lastPositionTime;  // Время последнего обнаружения линии
    
    // История позиций для предсказания направления движения линии
    static const int HISTORY_SIZE = 5;
    float positionHistory[5];     // Кольцевой буфер позиций
    int historyIndex;              // Текущий индекс в буфере
    int historyCount;              // Количество записей в истории
    
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
    
    // Получить последнюю известную позицию линии
    float getLastKnownPosition() const { return lastKnownPosition; }
    
    // Получить время последнего обнаружения линии (мс)
    unsigned long getLastPositionTime() const { return lastPositionTime; }
    
    // Сбросить память позиции
    void resetPositionMemory();
    
    // Получить направление движения линии (тренд)
    // Возвращает: <0 линия уходит влево, >0 линия уходит вправо, 0 стабильно
    float getPositionTrend() const;
    
    // Получить среднюю позицию за последние N измерений
    float getAveragePosition() const;
    
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
