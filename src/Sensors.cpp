#include "Sensors.h"

LineSensors::LineSensors() : lastKnownPosition(-999), lastPositionTime(0) {
    // Инициализация массивов калибровки
    for(int i = 0; i < 5; i++) {
        sensorMin[i] = 0;
        sensorMax[i] = 1023;
    }
}

void LineSensors::begin() {
    pinMode(SENSOR_1, INPUT);
    pinMode(SENSOR_2, INPUT);
    pinMode(SENSOR_3, INPUT);
    pinMode(SENSOR_4, INPUT);
    pinMode(SENSOR_5, INPUT);
}

void LineSensors::read(int sensors[5]) {
    // Чтение цифровых значений с датчиков
    // 0 = черная линия (LOW), 1 = белое поле (HIGH)
    sensors[0] = digitalRead(SENSOR_1);
    sensors[1] = digitalRead(SENSOR_2);
    sensors[2] = digitalRead(SENSOR_3);
    sensors[3] = digitalRead(SENSOR_4);
    sensors[4] = digitalRead(SENSOR_5);
}

float LineSensors::calculatePosition(int sensors[5]) {
    /*
     * Вычисляет позицию линии относительно центра робота
     * Веса датчиков: -2, -1, 0, +1, +2 (центр в нуле)
     * Возврат: -2.0 ... +2.0 - позиция линии, -999 - линия не найдена
     */
    
    int weights[5] = {-2, -1, 0, 1, 2};
    int lineValues[5];
    
    // Инвертируем значения: 0→1 (линия), 1→0 (поле)
    for (int i = 0; i < 5; i++) {
        lineValues[i] = (sensors[i] == 0) ? 1 : 0;
    }
    
    // Взвешенная сумма
    float weightedSum = 0.0;
    int totalActiveSensors = 0;
    
    for (int i = 0; i < 5; i++) {
        weightedSum += lineValues[i] * weights[i];
        totalActiveSensors += lineValues[i];
    }
    
    // Если ни один датчик не видит линию
    if (totalActiveSensors == 0) {
        return -999;  // Линия не найдена
    }
    
    // Нормализованная позиция
    float position = weightedSum / totalActiveSensors;
    
    // Сохраняем последнюю известную позицию
    lastKnownPosition = position;
    lastPositionTime = millis();
    
    return position;
}

void LineSensors::calibrate() {
    // Сброс мин/макс значений
    for (int i = 0; i < 5; i++) {
        sensorMin[i] = 1023;
        sensorMax[i] = 0;
    }
    
    Serial.println("Калибровка датчиков началась...");
    Serial.println("Водите робота над линией 5 секунд");
    
    unsigned long startTime = millis();
    int sensors[5];
    
    while (millis() - startTime < 5000) {
        read(sensors);
        
        for (int i = 0; i < 5; i++) {
            if (sensors[i] < sensorMin[i]) sensorMin[i] = sensors[i];
            if (sensors[i] > sensorMax[i]) sensorMax[i] = sensors[i];
        }
        
        delay(50);
    }
    
    Serial.println("✓ Калибровка завершена!");
    Serial.println("Результаты:");
    for (int i = 0; i < 5; i++) {
        Serial.printf("  Датчик %d: min=%d, max=%d\n", i + 1, sensorMin[i], sensorMax[i]);
    }
}

void LineSensors::resetPositionMemory() {
    lastKnownPosition = -999;
    lastPositionTime = 0;
}
