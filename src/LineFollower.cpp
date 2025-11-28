#include "LineFollower.h"
#include "Encoders.h"

// Конструктор
LineFollower::LineFollower(LineSensors& s, Motors& m, PIDController& p, Encoders* e)
    : sensors(s), motors(m), pid(p), encoders(e),
      currentState(IDLE), baseSpeed(BASE_SPEED), searchStartTime(0),
      turnDirection(TURN_NONE), turnStartTicksLeft(0), turnStartTicksRight(0),
      targetTurnDegrees(0), overshoot(false), lastValidPosition(0.0) {
}

void LineFollower::begin() {
    sensors.begin();
    motors.begin();
    
    if (encoders) {
        encoders->begin();
#ifdef DEBUG_MODE
        Serial.println("[OK] Энкодеры инициализированы");
        Serial.printf("[INFO] Кинематика: %.2f тиков/градус, %.2f мм/тик\n", 
                      TICKS_PER_DEGREE, MM_PER_TICK);
#endif
    }
    
    currentState = IDLE;
#ifdef DEBUG_MODE
    Serial.println("[OK] LineFollower инициализирован");
#endif
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
            
        case WAITING_FOR_TURN:
            // Неблокирующее ожидание обновления энкодеров
            if (millis() - waitStartTime >= 200) {
                // Прошло 200мс - обнуляем энкодеры и начинаем поворот
                if (encoders) {
                    encoders->resetTicks();
                }
#ifdef DEBUG_MODE
                Serial.printf("[%lu] ✓ Энкодеры обнулены → ПОИСК\n", millis());
#endif
                
                // FIXED: Линия СЛЕВА (turn=LEFT) → ищем ВЛЕВО (SEARCHING_LEFT)
                //        Линия СПРАВА (turn=RIGHT) → ищем ВПРАВО (SEARCHING_RIGHT)
                currentState = (turnDirection == TURN_LEFT) ? SEARCHING_LEFT : SEARCHING_RIGHT;
                searchStartTime = millis();
            }
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
#ifdef DEBUG_MODE
            Serial.println("⚠ ЛИНИЯ ПОТЕРЯНА! Нажмите кнопку для повторного поиска");
#endif
            currentState = IDLE;
            break;
    }
}

void LineFollower::start() {
#ifdef DEBUG_MODE
    Serial.printf("[%lu] ▶ СТАРТ → СЛЕДОВАНИЕ ПО ЛИНИИ\n", millis());
#endif
    currentState = FOLLOWING;
    pid.reset();
    sensors.resetPositionMemory();
    turnDirection = TURN_NONE;
    overshoot = false;
    lastValidPosition = 0.0;
}

void LineFollower::pause() {
#ifdef DEBUG_MODE
    Serial.println("⏸ ПАУЗА - Остановка");
#endif
    currentState = STOPPED;
    motors.stop();
}

void LineFollower::stop() {
#ifdef DEBUG_MODE
    Serial.printf("[%lu] ⏹ СТОП\n", millis());
#endif
    currentState = STOPPED;
    motors.stop();
}

void LineFollower::calibrate() {
#ifdef DEBUG_MODE
    Serial.println("⚙ Запуск калибровки датчиков");
#endif
    currentState = CALIBRATING;
}

void LineFollower::increaseSpeed() {
    baseSpeed = constrain(baseSpeed + 10, MIN_SPEED, MAX_SPEED);
#ifdef DEBUG_MODE
    Serial.printf("Скорость увеличена: %d\n", baseSpeed);
#endif
}

void LineFollower::decreaseSpeed() {
    baseSpeed = constrain(baseSpeed - 10, MIN_SPEED, MAX_SPEED);
#ifdef DEBUG_MODE
    Serial.printf("Скорость уменьшена: %d\n", baseSpeed);
#endif
}

// ═══════════════════════════════════════════════════════════════════════════
// НАЧАТЬ ПОВОРОТ НА МЕСТЕ (с контролем энкодеров)
// ═══════════════════════════════════════════════════════════════════════════
void LineFollower::startTurn(TurnDirection dir, float degrees) {
    turnDirection = dir;
    targetTurnDegrees = degrees;
    
    // ВАЖНО: Сначала ОСТАНАВЛИВАЕМ моторы чтобы сбросить инерцию!
    motors.stop();
    delay(20);  // Даём время остановиться
    
    if (encoders) {
        // Сбрасываем тики ПОСЛЕ остановки
        encoders->resetTicks();
        turnStartTicksLeft = 0;
        turnStartTicksRight = 0;
    }
    
    currentState = TURNING;
    
    // СРАЗУ даём команду на поворот (не ждём executeTurn!)
    int leftCmd, rightCmd;
    if (dir == TURN_RIGHT) {
        leftCmd = TURN_SPEED;
        rightCmd = -TURN_SPEED;
    } else {
        leftCmd = -TURN_SPEED;
        rightCmd = TURN_SPEED;
    }
    motors.setSpeed(leftCmd, rightCmd);
    
#ifdef DEBUG_MODE
    const char* dirStr = (dir == TURN_LEFT) ? "ВЛЕВО" : "ВПРАВО";
    Serial.printf("[%lu] 🔄 ПОВОРОТ %s на %.1f° (цель: %.1f тиков) | M: L=%d R=%d\n", 
                  millis(), dirStr, degrees, degrees * TICKS_PER_DEGREE, leftCmd, rightCmd);
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
#ifdef DEBUG_MODE
        if (encoders) {
            long leftTicks = abs(encoders->getLeftTicks());
            long rightTicks = abs(encoders->getRightTicks());
            Serial.printf("[%lu] ✓ ЛИНИЯ В ЦЕНТРЕ (повернули L=%ld R=%ld тиков) → ЕДЕМ\n", 
                          millis(), leftTicks, rightTicks);
        } else {
            Serial.printf("[%lu] ✓ ЛИНИЯ В ЦЕНТРЕ → ЕДЕМ\n", millis());
        }
#endif
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
#ifdef DEBUG_MODE
            Serial.printf("[%lu] ⚠ Повернули %.1f° (L=%ld R=%ld тиков), линия не найдена → ПОИСК\n", 
                          millis(), targetTurnDegrees, leftTicks, rightTicks);
#endif
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
// СЛЕДОВАНИЕ ПО ЛИНИИ (как у профессиональных LFR - без поворота на месте!)
// ═══════════════════════════════════════════════════════════════════════════
void LineFollower::followLine() {
    int sensorValues[5];
    sensors.read(sensorValues);
    
    float position = sensors.calculatePosition(sensorValues);
    
    // Проверка: линия найдена?
    if (position == -999) {
        // Линия не видна датчиками - проверяем память позиции
        unsigned long timeSinceLine = millis() - sensors.getLastPositionTime();
        float lastPosition = sensors.getLastKnownPosition();
        
        // ДЕТЕКЦИЯ OVERSHOOT (идея Wright Hobbies + roboforum.ru)
        // Если потеряли линию в повороте - устанавливаем флаг
        if (lastValidPosition != 0.0 && timeSinceLine >= OVERSHOOT_DETECT_TIME) {
            overshoot = true;
        }
        
        // Проверяем что есть валидная сохранённая позиция и она не устарела
        if (lastPosition != -999 && timeSinceLine < LINE_MEMORY_TIMEOUT) {
            // ═══════════════════════════════════════════════════════════════
            // ЛИНИЯ ПОТЕРЯНА НО НЕДАВНО БЫЛА ВИДНА
            // Резкий поворот? ОСТАНАВЛИВАЕМСЯ и ждём обновления энкодеров!
            // ═══════════════════════════════════════════════════════════════
            // Требуем сильное отклонение (2.3+) И потерю линии более 80мс
            // Смягчено после анализа roboforum.ru - не надо слишком рано останавливаться
            if (abs(lastPosition) >= 2.5 && timeSinceLine >= 100) {
                // Резкое отклонение - это поворот трассы!
                motors.stop();
                
                // FIXED: Определяем направление по ПОЗИЦИИ:
                // lastPosition < 0 (линия СЛЕВА) → TURN_LEFT (ищем влево)
                // lastPosition > 0 (линия СПРАВА) → TURN_RIGHT (ищем вправо)
                turnDirection = (lastPosition < 0) ? TURN_LEFT : TURN_RIGHT;
                
                // Переходим в режим ожидания (неблокирующий!)
                waitStartTime = millis();
                currentState = WAITING_FOR_TURN;
                
#ifdef DEBUG_MODE
                Serial.printf("[%lu] ⏸ СТОП перед поворотом (поз=%.1f) → ждём 200мс\n",
                              millis(), lastPosition);
#endif
                return;
            }
            
            // Адаптивная скорость поиска (идея Wright Hobbies)
            // После 150мс поиска - снижаем скорость на 30% для точности
            int searchSpeed = baseSpeed;
            if (timeSinceLine > 150) {
                searchSpeed = baseSpeed * 0.7;
            }
            
            int leftSpeed, rightSpeed;
            
            if (lastPosition > 0) {
                // Линия была справа - крутим вправо
                leftSpeed = searchSpeed;
                rightSpeed = -MIN_SPEED;
            } else {
                // Линия была слева - крутим влево
                leftSpeed = -MIN_SPEED;
                rightSpeed = searchSpeed;
            }
            
            motors.setSpeed(leftSpeed, rightSpeed);
            
#ifdef DEBUG_MODE
            static unsigned long lastLostDebug = 0;
            if (millis() - lastLostDebug > 200) {
                Serial.printf("[%lu] 🔍 ПОИСК (память: %.1f, %lu мс) | M: L=%d R=%d\n",
                              millis(), lastPosition, timeSinceLine, leftSpeed, rightSpeed);
                lastLostDebug = millis();
            }
#endif
            return;
        } else {
            // Линия давно потеряна - начинаем полноценный поиск
#ifdef DEBUG_MODE
            Serial.printf("[%lu] ⚠ ЛИНИЯ ПОТЕРЯНА! → ПОИСК\n", millis());
#endif
            currentState = SEARCHING_LEFT;
            searchStartTime = millis();
            return;
        }
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // ЛИНИЯ ВИДНА - ЧИСТЫЙ PID БЕЗ ПОВОРОТА НА МЕСТЕ!
    // (как у amjed-ali-k и Aarushraj-Puduchery)
    // ═══════════════════════════════════════════════════════════════════════
    
    // Сохраняем валидную позицию для детекции overshoot
    lastValidPosition = position;
    
    float error = position;
    float absError = abs(error);
    
    // Пороги для PID режимов (улучшенная градация по Wright Hobbies)
    const float AGGRESSIVE_THRESHOLD = 2.0;  // Для резких отклонений
    const float OVERSHOOT_THRESHOLD = 2.5;   // Критическое отклонение = вероятный overshoot
    
    // Определяем режим
    const char* mode;
    int leftSpeed, rightSpeed;
    
    // Вычисляем PID коррекцию
    float correction;
    if (absError >= AGGRESSIVE_THRESHOLD) {
        // ═══════════════════════════════════════════════════════════════════
        // АГРЕССИВНЫЙ PID - для сильных отклонений
        // С учётом OVERSHOOT (идея Wright Hobbies)
        // ═══════════════════════════════════════════════════════════════════
        
        // Если overshoot - смягчаем коррекцию (не делаем слишком резких движений)
        if (overshoot && absError >= OVERSHOOT_THRESHOLD) {
            mode = "ВОССТ";
            // Смягчённая коррекция для восстановления после overshoot
            correction = (AGGRESSIVE_KP * 0.7) * error + (AGGRESSIVE_KD * 0.8) * (error - pid.getPreviousError());
            overshoot = false;  // Сбрасываем флаг после применения
        } else {
            mode = "АГРЕСС";
            correction = AGGRESSIVE_KP * error + AGGRESSIVE_KD * (error - pid.getPreviousError());
        }
        
        // Применяем коррекцию с возможностью отрицательной скорости
        leftSpeed = baseSpeed + correction;
        rightSpeed = baseSpeed - correction;
        
        // Ограничиваем: минимум может быть отрицательным для резкого поворота
        leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
        rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
        
    } else {
        // ═══════════════════════════════════════════════════════════════════
        // ПЛАВНЫЙ PID - стандартная коррекция
        // ═══════════════════════════════════════════════════════════════════
        mode = "ПЛАВНО";
        correction = pid.calculate(error);
        overshoot = false;  // Сбрасываем флаг при нормальном движении
        
        leftSpeed = baseSpeed + correction;
        rightSpeed = baseSpeed - correction;
        
        // Ограничиваем скорости (минимум всегда положительный)
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
#ifdef DEBUG_MODE
        Serial.printf("[%lu] ✓ ЛИНИЯ НАЙДЕНА (поиск %lu мс) → ЕДЕМ\n", now, searchTime);
#endif
        currentState = FOLLOWING;
        pid.reset();
        return;
    }
    
    // Проверяем таймаут
    if (searchTime > SEARCH_TIMEOUT) {
#ifdef DEBUG_MODE
        Serial.printf("[%lu] ✗ ТАЙМАУТ ПОИСКА (%lu мс) → ПОТЕРЯНА\n", now, searchTime);
#endif
        currentState = LOST;
        return;
    }
    
    // Выполняем поиск (поворот на месте)
    if (currentState == SEARCHING_LEFT) {
        motors.setSpeed(-TURN_SPEED, TURN_SPEED);  // Поворот влево на месте
        
        // Переключаемся на поиск вправо через половину времени
        if (searchTime > SEARCH_TIMEOUT / 2) {
#ifdef DEBUG_MODE
            Serial.printf("[%lu] → ПОИСК ВПРАВО (прошло %lu мс)\n", now, searchTime);
#endif
            currentState = SEARCHING_RIGHT;
        }
    } else {
        motors.setSpeed(TURN_SPEED, -TURN_SPEED);  // Поворот вправо на месте
    }
}
