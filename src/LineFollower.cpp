#include "LineFollower.h"
#include "Encoders.h"

// Конструктор
LineFollower::LineFollower(LineSensors& s, Motors& m, PIDController& p, Encoders* e)
    : sensors(s), motors(m), pid(p), encoders(e),
      currentState(IDLE), baseSpeed(BASE_SPEED), searchStartTime(0) {
}

void LineFollower::begin() {
    sensors.begin();
    motors.begin();
    
    if (encoders) {
        encoders->begin();
    }
    
    currentState = IDLE;
    Serial.println("[OK] LineFollower инициализирован");
}

void LineFollower::update() {
    // Обновление энкодеров
    if (encoders) {
        encoders->update();
    }
    
    // Обработка текущего состояния
    switch (currentState) {
        case IDLE:
        case STOPPED:
            // Гарантируем, что моторы остановлены
            motors.stop();
            break;
            
        case CALIBRATING:
            sensors.calibrate();
            currentState = IDLE;
            break;
            
        case FOLLOWING:
            followLine();
            break;
            
        case SEARCHING_LEFT:
        case SEARCHING_RIGHT:
            searchLine();
            break;
            
        case LOST:
            motors.stop();
            Serial.println("⚠ ЛИНИЯ ПОТЕРЯНА! Отправьте 's' для повторного поиска");
            currentState = IDLE;
            break;
    }
}

void LineFollower::start() {
    Serial.println("▶ СТАРТ - Начинаю следование по линии");
    currentState = FOLLOWING;
    pid.reset();
}

void LineFollower::pause() {
    Serial.println("⏸ ПАУЗА - Остановка");
    currentState = STOPPED;
    motors.stop();
}

void LineFollower::stop() {
    currentState = STOPPED;
    motors.stop();
}

void LineFollower::calibrate() {
    Serial.println("⚙ Запуск калибровки датчиков");
    currentState = CALIBRATING;
}

void LineFollower::increaseSpeed() {
    baseSpeed = constrain(baseSpeed + 10, MIN_SPEED, MAX_SPEED);
    Serial.printf("Скорость увеличена: %d\n", baseSpeed);
}

void LineFollower::decreaseSpeed() {
    baseSpeed = constrain(baseSpeed - 10, MIN_SPEED, MAX_SPEED);
    Serial.printf("Скорость уменьшена: %d\n", baseSpeed);
}

void LineFollower::followLine() {
    int sensorValues[5];
    sensors.read(sensorValues);
    
    float position = sensors.calculatePosition(sensorValues);
    
    // Проверка: линия найдена?
    if (position == -999) {
        // Линия потеряна - начинаем поиск
        Serial.println("⚠ Линия потеряна! Начинаю поиск...");
        currentState = SEARCHING_LEFT;
        searchStartTime = millis();
        return;
    }
    
    // Вычисляем ошибку (отклонение от центра)
    float error = position;
    
    // ПИД-регулятор
    float correction = pid.calculate(error);
    
    // Применяем корректировку к скоростям моторов
    int leftSpeed = baseSpeed + correction;
    int rightSpeed = baseSpeed - correction;
    
    // Ограничиваем скорости
    leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
    
    // Устанавливаем скорости моторов
    motors.setSpeed(leftSpeed, rightSpeed);
    
    // Отладочный вывод
#ifdef DEBUG_MODE
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 200) {  // Каждые 200 мс
        Serial.print("Датчики: ");
        for (int i = 0; i < 5; i++) {
            Serial.print(sensorValues[i]);
            Serial.print(" ");
        }
        Serial.printf("| Позиция: %.2f | Ошибка: %.2f | Коррекция: %.1f | Моторы: L=%d R=%d\n",
                      position, error, correction, leftSpeed, rightSpeed);
        lastDebugTime = millis();
    }
#endif
}

void LineFollower::searchLine() {
    int sensorValues[5];
    sensors.read(sensorValues);
    
    float position = sensors.calculatePosition(sensorValues);
    
    // Проверяем, нашли ли линию
    if (position != -999) {
        Serial.println("✓ Линия найдена! Продолжаю движение");
        currentState = FOLLOWING;
        pid.reset();
        return;
    }
    
    // Проверяем таймаут
    if (millis() - searchStartTime > SEARCH_TIMEOUT) {
        Serial.println("✗ Таймаут поиска. Линия не найдена.");
        currentState = LOST;
        return;
    }
    
    // Выполняем поиск (поворот на месте)
    if (currentState == SEARCHING_LEFT) {
        motors.turnLeft(TURN_SPEED);
        
        // Переключаемся на поиск вправо через половину времени
        if (millis() - searchStartTime > SEARCH_TIMEOUT / 2) {
            Serial.println("→ Переключаюсь на поиск вправо");
            currentState = SEARCHING_RIGHT;
        }
    } else {
        motors.turnRight(TURN_SPEED);
    }
}
