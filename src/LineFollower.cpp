#include "LineFollower.h"
#include "Encoders.h"

// Конструктор
LineFollower::LineFollower(LineSensors& s, Motors& m, PIDController& p, Encoders* e)
    : sensors(s), motors(m), pid(p), encoders(e),
      currentState(IDLE), baseSpeed(BASE_SPEED), searchStartTime(0),
      turnDirection(TURN_NONE), turnStartTicksLeft(0), turnStartTicksRight(0),
      targetTurnDegrees(0) {
}

void LineFollower::begin() {
    sensors.begin();
    motors.begin();
    
    if (encoders) {
        encoders->begin();
        Serial.println("[OK] Энкодеры инициализированы");
        Serial.printf("[INFO] Кинематика: %.2f тиков/градус, %.2f мм/тик\n", 
                      TICKS_PER_DEGREE, MM_PER_TICK);
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
            
        case TURNING:
            executeTurn();
            break;
            
        case SEARCHING_LEFT:
        case SEARCHING_RIGHT:
            searchLine();
            break;
            
        case LOST:
            motors.stop();
            Serial.println("⚠ ЛИНИЯ ПОТЕРЯНА! Нажмите кнопку для повторного поиска");
            currentState = IDLE;
            break;
    }
}

void LineFollower::start() {
    Serial.printf("[%lu] ▶ СТАРТ → СЛЕДОВАНИЕ ПО ЛИНИИ\n", millis());
    currentState = FOLLOWING;
    pid.reset();
    sensors.resetPositionMemory();
    turnDirection = TURN_NONE;
}

void LineFollower::pause() {
    Serial.println("⏸ ПАУЗА - Остановка");
    currentState = STOPPED;
    motors.stop();
}

void LineFollower::stop() {
    Serial.printf("[%lu] ⏹ СТОП\n", millis());
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

// ═══════════════════════════════════════════════════════════════════════════
// НАЧАТЬ ПОВОРОТ НА МЕСТЕ (с контролем энкодеров)
// ═══════════════════════════════════════════════════════════════════════════
void LineFollower::startTurn(TurnDirection dir, float degrees) {
    turnDirection = dir;
    targetTurnDegrees = degrees;
    
    if (encoders) {
        // Сбрасываем и запоминаем начальные тики
        encoders->resetTicks();
        turnStartTicksLeft = 0;
        turnStartTicksRight = 0;
    }
    
    currentState = TURNING;
    
#ifdef DEBUG_MODE
    const char* dirStr = (dir == TURN_LEFT) ? "ВЛЕВО" : "ВПРАВО";
    Serial.printf("[%lu] 🔄 ПОВОРОТ %s на %.1f° (цель: %.1f тиков)\n", 
                  millis(), dirStr, degrees, degrees * TICKS_PER_DEGREE);
#endif
}

// ═══════════════════════════════════════════════════════════════════════════
// ВЫПОЛНЕНИЕ ПОВОРОТА (с контролем энкодеров)
// ═══════════════════════════════════════════════════════════════════════════
void LineFollower::executeTurn() {
    // Сначала проверяем датчики - может линия уже под центром?
    int sensorValues[5];
    sensors.read(sensorValues);
    float position = sensors.calculatePosition(sensorValues);
    
    // Если линия вернулась в центр - завершаем поворот
    if (position != -999 && abs(position) < 0.5) {
        motors.stop();
        currentState = FOLLOWING;
        pid.reset();
        if (encoders) {
            long leftTicks = abs(encoders->getLeftTicks());
            long rightTicks = abs(encoders->getRightTicks());
            Serial.printf("[%lu] ✓ ЛИНИЯ В ЦЕНТРЕ (повернули L=%ld R=%ld тиков) → ЕДЕМ\n", 
                          millis(), leftTicks, rightTicks);
        } else {
            Serial.printf("[%lu] ✓ ЛИНИЯ В ЦЕНТРЕ → ЕДЕМ\n", millis());
        }
        return;
    }
    
    // Проверяем энкодеры - не повернули ли достаточно?
    if (encoders) {
        long leftTicks = abs(encoders->getLeftTicks());
        long rightTicks = abs(encoders->getRightTicks());
        // Используем СРЕДНЕЕ из двух энкодеров для точного контроля угла
        long avgTicks = (leftTicks + rightTicks) / 2;
        
        // Сколько тиков нужно для целевого угла
        float targetTicks = targetTurnDegrees * TICKS_PER_DEGREE;
        
#ifdef DEBUG_MODE
        static unsigned long lastTurnDebug = 0;
        if (millis() - lastTurnDebug > 100) {
            Serial.printf("[%lu]   └─ Тики: L=%ld R=%ld (ср=%ld/%.1f) | Поз=%.2f\n", 
                          millis(), leftTicks, rightTicks, avgTicks, targetTicks, position);
            lastTurnDebug = millis();
        }
#endif
        
        // Если повернули достаточно - ищем линию
        if (avgTicks >= targetTicks) {
            motors.stop();
            Serial.printf("[%lu] ⚠ Повернули %.1f° (L=%ld R=%ld тиков), линия не найдена → ПОИСК\n", 
                          millis(), targetTurnDegrees, leftTicks, rightTicks);
            currentState = (turnDirection == TURN_LEFT) ? SEARCHING_RIGHT : SEARCHING_LEFT;
            searchStartTime = millis();
            return;
        }
    }
    
    // Выполняем поворот
    int leftCmd, rightCmd;
    if (turnDirection == TURN_RIGHT) {
        // Поворот ВПРАВО: левое вперёд, правое назад
        leftCmd = TURN_SPEED;
        rightCmd = -TURN_SPEED;
    } else {
        // Поворот ВЛЕВО: левое назад, правое вперёд
        leftCmd = -TURN_SPEED;
        rightCmd = TURN_SPEED;
    }
    
    motors.setSpeed(leftCmd, rightCmd);
}

// ═══════════════════════════════════════════════════════════════════════════
// СЛЕДОВАНИЕ ПО ЛИНИИ
// ═══════════════════════════════════════════════════════════════════════════
void LineFollower::followLine() {
    int sensorValues[5];
    sensors.read(sensorValues);
    
    float position = sensors.calculatePosition(sensorValues);
    bool usingMemory = false;  // Флаг: используем память позиции
    
    // Проверка: линия найдена?
    if (position == -999) {
        // Линия не видна датчиками - проверяем память позиции
        unsigned long timeSinceLine = millis() - sensors.getLastPositionTime();
        float lastPosition = sensors.getLastKnownPosition();
        
        // Проверяем что есть валидная сохранённая позиция и она не устарела
        if (lastPosition != -999 && timeSinceLine < LINE_MEMORY_TIMEOUT) {
            // Используем последнюю известную позицию
            // ВАЖНО: при использовании памяти НЕ делаем резких поворотов!
            position = lastPosition;
            usingMemory = true;
        } else {
            // Линия действительно потеряна - начинаем поиск
            Serial.printf("[%lu] ⚠ ЛИНИЯ ПОТЕРЯНА! → ПОИСК\n", millis());
            currentState = SEARCHING_LEFT;
            searchStartTime = millis();
            return;
        }
    }
    
    // Вычисляем ошибку (отклонение от центра)
    float error = position;
    
    // ПИД-регулятор
    float correction = pid.calculate(error);
    
    // ═══════════════════════════════════════════════════════════════════════
    // ДИФФЕРЕНЦИАЛЬНОЕ УПРАВЛЕНИЕ С ПОВОРОТОМ НА МЕСТЕ
    // ═══════════════════════════════════════════════════════════════════════
    
    int leftSpeed, rightSpeed;
    float absError = abs(error);
    
    // Пороги для разных режимов управления
    // Позиция ±2.0 = только ОДИН крайний датчик видит линию (резкий поворот 90°)
    // Позиция ±1.0 = один из боковых датчиков (плавный поворот)
    // Позиция ±0.5 = центр или рядом
    const float TURN_IN_PLACE_THRESHOLD = 2.0;  // Поворот на месте (только крайние датчики!)
    const float AGGRESSIVE_THRESHOLD = 1.0;      // Агрессивная коррекция
    
    // Определяем режим
    const char* mode;
    
    // ВАЖНО: НЕ поворачиваем на месте если используем память позиции!
    // Память = линия между датчиками, нужна только плавная коррекция
    if (absError >= TURN_IN_PLACE_THRESHOLD && !usingMemory) {
        // ═══════════════════════════════════════════════════════════════════
        // РЕЖИМ: ПОВОРОТ НА МЕСТЕ
        // Только крайний датчик видит линию - реальный резкий поворот!
        // ═══════════════════════════════════════════════════════════════════
        mode = "ПОВОРОТ";
        
        if (encoders) {
            // С энкодерами - используем контролируемый поворот
            // Оцениваем угол по формуле из Config.h
            float estimatedAngle = (absError - MIN_TURN_ERROR) * DEGREES_PER_ERROR + BASE_TURN_ANGLE;
            TurnDirection dir = (error > 0) ? TURN_RIGHT : TURN_LEFT;
            startTurn(dir, estimatedAngle);
            return;
        } else {
            // Без энкодеров - простой поворот на месте
            if (error > 0) {
                leftSpeed = TURN_SPEED;
                rightSpeed = -TURN_SPEED;
            } else {
                leftSpeed = -TURN_SPEED;
                rightSpeed = TURN_SPEED;
            }
        }
    } else if (absError >= AGGRESSIVE_THRESHOLD) {
        // ═══════════════════════════════════════════════════════════════════
        // РЕЖИМ: АГРЕССИВНАЯ КОРРЕКЦИЯ
        // Линия немного сбоку - одно колесо почти стоит, другое едет
        // ═══════════════════════════════════════════════════════════════════
        mode = "АГРЕСС";
        
        if (error > 0) {
            leftSpeed = baseSpeed;
            rightSpeed = MIN_SPEED;
        } else {
            leftSpeed = MIN_SPEED;
            rightSpeed = baseSpeed;
        }
    } else {
        // ═══════════════════════════════════════════════════════════════════
        // РЕЖИМ: ПЛАВНАЯ КОРРЕКЦИЯ
        // Линия почти по центру - едем прямо с небольшой коррекцией
        // ═══════════════════════════════════════════════════════════════════
        mode = "ПЛАВНО";
        
        leftSpeed = baseSpeed + correction;
        rightSpeed = baseSpeed - correction;
        
        // Ограничиваем скорости
        leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
        rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
    }
    
    // Устанавливаем скорости моторов
    motors.setSpeed(leftSpeed, rightSpeed);
    
    // Отладочный вывод
#ifdef DEBUG_MODE
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 200) {
        unsigned long now = millis();
        Serial.printf("[%lu] ", now);
        
        // Датчики компактно
        for (int i = 0; i < 5; i++) {
            Serial.print(sensorValues[i]);
        }
        
        Serial.printf(" | Поз=%+.2f | %6s | M: L=%3d R=%3d", 
                      position, mode, leftSpeed, rightSpeed);
        
        // Показываем скорости с энкодеров
        if (encoders) {
            Serial.printf(" | Энк: L=%4.0f R=%4.0f мм/с", 
                          encoders->getLeftSpeed(), encoders->getRightSpeed());
        }
        
        // Флаг использования памяти
        if (usingMemory) {
            unsigned long timeSinceLine = now - sensors.getLastPositionTime();
            Serial.printf(" [МЕМ:%lu мс]", timeSinceLine);
        }
        
        Serial.println();
        lastDebugTime = now;
    }
#endif
}

// ═══════════════════════════════════════════════════════════════════════════
// ПОИСК ЛИНИИ
// ═══════════════════════════════════════════════════════════════════════════
void LineFollower::searchLine() {
    int sensorValues[5];
    sensors.read(sensorValues);
    
    float position = sensors.calculatePosition(sensorValues);
    
    unsigned long now = millis();
    unsigned long searchTime = now - searchStartTime;
    
    // Проверяем, нашли ли линию
    if (position != -999) {
        Serial.printf("[%lu] ✓ ЛИНИЯ НАЙДЕНА (поиск %lu мс) → ЕДЕМ\n", now, searchTime);
        currentState = FOLLOWING;
        pid.reset();
        return;
    }
    
    // Проверяем таймаут
    if (searchTime > SEARCH_TIMEOUT) {
        Serial.printf("[%lu] ✗ ТАЙМАУТ ПОИСКА (%lu мс) → ПОТЕРЯНА\n", now, searchTime);
        currentState = LOST;
        return;
    }
    
    // Выполняем поиск (поворот на месте)
    if (currentState == SEARCHING_LEFT) {
        motors.setSpeed(-TURN_SPEED, TURN_SPEED);  // Поворот влево на месте
        
        // Переключаемся на поиск вправо через половину времени
        if (searchTime > SEARCH_TIMEOUT / 2) {
            Serial.printf("[%lu] → ПОИСК ВПРАВО (прошло %lu мс)\n", now, searchTime);
            currentState = SEARCHING_RIGHT;
        }
    } else {
        motors.setSpeed(TURN_SPEED, -TURN_SPEED);  // Поворот вправо на месте
    }
}
