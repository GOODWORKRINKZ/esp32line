#include "Sensors.h"

LineSensors::LineSensors() : lastKnownPosition(-999), lastPositionTime(0),
                             historyIndex(0), historyCount(0) {
    // Инициализация массивов калибровки
    for(int i = 0; i < 5; i++) {
        sensorMin[i] = 0;
        sensorMax[i] = 1023;
        positionHistory[i] = 0.0;  // Инициализация истории
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
     * 
     * Экспоненциальные веса датчиков (как в профессиональных LFR):
     * Крайние датчики имеют больший вес для острой реакции на резкие повороты
     * Веса: [-3.0, -1.0, 0, +1.0, +3.0] (экспоненциальное распределение)
     * 
     * ПОДДЕРЖКА ШИРОКОЙ ЛИНИИ:
     * Если два соседних центральных датчика (2+3 или 3+4) видят линию,
     * считаем что робот на прямой (позиция = 0)
     * 
     * Возврат: примерно -3.0 ... +3.0 - позиция линии, -999 - линия не найдена
     */
    
    // Экспоненциальные веса: крайние датчики важнее для детекции поворотов
    float weights[5] = {-3.0, -1.0, 0.0, 1.0, 3.0};
    int lineValues[5];
    
    // Инвертируем значения: 0→1 (линия), 1→0 (поле)
    for (int i = 0; i < 5; i++) {
        lineValues[i] = (sensors[i] == 0) ? 1 : 0;
    }
    
    // Подсчёт активных датчиков
    int totalActiveSensors = 0;
    for (int i = 0; i < 5; i++) {
        totalActiveSensors += lineValues[i];
    }
    
    // Если ни один датчик не видит линию
    if (totalActiveSensors == 0) {
        return -999;  // Линия не найдена
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // ПОДДЕРЖКА ШИРОКОЙ ЛИНИИ
    // Если ровно 2 соседних центральных датчика активны - едем прямо
    // ═══════════════════════════════════════════════════════════════════════
    if (totalActiveSensors == 2) {
        // Индексы [1] и [2] (SENSOR_2 левый + SENSOR_3 центральный)
        if (lineValues[1] && lineValues[2]) {
            lastKnownPosition = 0.0;  // Считаем что на прямой
            lastPositionTime = millis();
            return 0.0;
        }
        // Индексы [2] и [3] (SENSOR_3 центральный + SENSOR_4 правый)
        if (lineValues[2] && lineValues[3]) {
            lastKnownPosition = 0.0;  // Считаем что на прямой
            lastPositionTime = millis();
            return 0.0;
        }
    }
    
    // Стандартный расчёт через взвешенную сумму
    float weightedSum = 0.0;
    for (int i = 0; i < 5; i++) {
        weightedSum += lineValues[i] * weights[i];
    }
    
    // Нормализованная позиция
    float position = weightedSum / totalActiveSensors;
    
    // Сохраняем в историю (кольцевой буфер)
    positionHistory[historyIndex] = position;
    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    if (historyCount < HISTORY_SIZE) historyCount++;
    
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
    historyIndex = 0;
    historyCount = 0;
    for (int i = 0; i < HISTORY_SIZE; i++) {
        positionHistory[i] = 0.0;
    }
}

float LineSensors::getPositionTrend() const {
    // Возвращает направление движения линии (тренд)
    // >0 = линия уходит вправо, <0 = линия уходит влево
    if (historyCount < 2) return 0.0;
    
    // Вычисляем разницу между новыми и старыми позициями
    int newest = (historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE;
    int oldest = (historyIndex - historyCount + HISTORY_SIZE) % HISTORY_SIZE;
    
    return positionHistory[newest] - positionHistory[oldest];
}

float LineSensors::getAveragePosition() const {
    // Возвращает среднюю позицию за последние измерения
    if (historyCount == 0) return lastKnownPosition;
    
    float sum = 0.0;
    for (int i = 0; i < historyCount; i++) {
        sum += positionHistory[i];
    }
    return sum / historyCount;
}
