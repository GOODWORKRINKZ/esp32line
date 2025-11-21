/*
 * ═══════════════════════════════════════════════════════════════════════════
 * РОБОТ СЛЕДУЮЩИЙ ПО ЛИНИИ - ESP32 + 5 ДАТЧИКОВ TCRT5000 + L298N + ЭНКОДЕРЫ
 * ═══════════════════════════════════════════════════════════════════════════
 * 
 * Описание:
 * Оптимизированная прошивка для робота следующего по черной линии (20мм)
 * с использованием ПИД-регулятора и опциональной поддержкой энкодеров.
 * 
 * Железо:
 * - ESP32 DevKit
 * - 5× TCRT5000 (датчики линии, цифровые)
 * - 2× DC моторы с редуктором (диаметр колес 65мм)
 * - L298N драйвер моторов (ENA/ENB припаяны к HIGH)
 * - 2× FC-03 оптические энкодеры (опционально)
 * - Батарея 7.4V Li-Po 2S
 * 
 * Геометрия:
 * - Колесная база: 125 мм
 * - Расстояние между датчиками: 15 мм (общая ширина массива 60 мм)
 * - Датчики впереди колес: 35 мм
 * - Высота датчиков над землей: 4-5 мм
 * 
 * Автор: GOODWORKRINKZ
 * Дата: 2025
 * Лицензия: MIT
 * ═══════════════════════════════════════════════════════════════════════════
 */

// ═══════════════════════════════════════════════════════════════════════════
// НАСТРОЙКИ РЕЖИМА РАБОТЫ
// ═══════════════════════════════════════════════════════════════════════════

// Раскомментируйте для использования энкодеров FC-03
// #define USE_ENCODERS

// Режим отладки - выводит подробную информацию в Serial
#define DEBUG_MODE

// ═══════════════════════════════════════════════════════════════════════════
// ПИНЫ ПОДКЛЮЧЕНИЯ
// ═══════════════════════════════════════════════════════════════════════════

// Датчики линии TCRT5000 (цифровые входы)
#define SENSOR_1  32  // Крайний левый
#define SENSOR_2  33  // Левый
#define SENSOR_3  25  // Центральный
#define SENSOR_4  26  // Правый
#define SENSOR_5  27  // Крайний правый

// Управление моторами L298N (без PWM на ENA/ENB, управление через INx)
#define MOTOR_LEFT_FWD   12  // IN1 - левый мотор вперед
#define MOTOR_LEFT_BWD   13  // IN2 - левый мотор назад
#define MOTOR_RIGHT_FWD  14  // IN3 - правый мотор вперед
#define MOTOR_RIGHT_BWD  15  // IN4 - правый мотор назад

// Энкодеры FC-03 (опционально, прерывания)
#define ENCODER_LEFT   16  // Левый энкодер
#define ENCODER_RIGHT  17  // Правый энкодер

// ═══════════════════════════════════════════════════════════════════════════
// ПАРАМЕТРЫ РОБОТА
// ═══════════════════════════════════════════════════════════════════════════

// Физические параметры
#define WHEEL_DIAMETER      65.0    // Диаметр колеса в мм
#define WHEEL_BASE          125.0   // Расстояние между колесами в мм
#define ENCODER_SLOTS       20      // Количество прорезей в диске FC-03

// Вычисляемые константы
#define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER)  // Длина окружности колеса
#define MM_PER_TICK         (WHEEL_CIRCUMFERENCE / ENCODER_SLOTS)  // мм на импульс

// ═══════════════════════════════════════════════════════════════════════════
// ПИД-РЕГУЛЯТОР - ПАРАМЕТРЫ
// ═══════════════════════════════════════════════════════════════════════════

// Коэффициенты ПИД для следования по линии
float Kp = 25.0;   // Пропорциональный (основной) - отклик на текущую ошибку
float Ki = 0.0;    // Интегральный - обычно не нужен для линии
float Kd = 15.0;   // Дифференциальный - предсказание, сглаживание

// Переменные ПИД
float previousError = 0.0;
float integral = 0.0;

// Базовая скорость робота (0-255)
int baseSpeed = 130;        // Крейсерская скорость
int maxSpeed = 200;         // Максимальная скорость
int minSpeed = 60;          // Минимальная скорость (чтобы не стоять)
int turnSpeed = 100;        // Скорость при поиске линии

// ═══════════════════════════════════════════════════════════════════════════
// ЭНКОДЕРЫ - ПЕРЕМЕННЫЕ
// ═══════════════════════════════════════════════════════════════════════════

#ifdef USE_ENCODERS
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;
unsigned long lastEncoderTime = 0;
float leftSpeed = 0.0;    // Текущая скорость левого колеса (мм/сек)
float rightSpeed = 0.0;   // Текущая скорость правого колеса (мм/сек)

// ПИД для управления скоростью моторов
float speedKp = 2.0;
float speedKi = 0.5;
float speedKd = 0.1;
float leftSpeedError = 0.0;
float rightSpeedError = 0.0;
#endif

// ═══════════════════════════════════════════════════════════════════════════
// КАЛИБРОВКА ДАТЧИКОВ
// ═══════════════════════════════════════════════════════════════════════════

// Пороговые значения для датчиков (если нужна аналоговая калибровка)
int sensorMin[5] = {0, 0, 0, 0, 0};       // Минимум (черный)
int sensorMax[5] = {1023, 1023, 1023, 1023, 1023};  // Максимум (белый)

// ═══════════════════════════════════════════════════════════════════════════
// СОСТОЯНИЯ РОБОТА
// ═══════════════════════════════════════════════════════════════════════════

enum RobotState {
  IDLE,              // Ожидание
  CALIBRATING,       // Калибровка датчиков
  FOLLOWING,         // Следование по линии
  SEARCHING_LEFT,    // Поиск линии влево
  SEARCHING_RIGHT,   // Поиск линии вправо
  LOST,              // Линия потеряна
  STOPPED            // Остановлен
};

RobotState currentState = IDLE;
unsigned long searchStartTime = 0;
const unsigned long SEARCH_TIMEOUT = 2000;  // 2 секунды на поиск

// ═══════════════════════════════════════════════════════════════════════════
// ПРЕРЫВАНИЯ ДЛЯ ЭНКОДЕРОВ
// ═══════════════════════════════════════════════════════════════════════════

#ifdef USE_ENCODERS
// Прерывание для левого энкодера
void IRAM_ATTR leftEncoderISR() {
  leftEncoderTicks++;
}

// Прерывание для правого энкодера
void IRAM_ATTR rightEncoderISR() {
  rightEncoderTicks++;
}
#endif

// ═══════════════════════════════════════════════════════════════════════════
// SETUP - ИНИЦИАЛИЗАЦИЯ
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  // Инициализация Serial для отладки
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║  РОБОТ СЛЕДУЮЩИЙ ПО ЛИНИИ - ESP32        ║");
  Serial.println("║  5 датчиков TCRT5000 + ПИД-регулятор     ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
  
  // Настройка пинов датчиков линии
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_4, INPUT);
  pinMode(SENSOR_5, INPUT);
  Serial.println("[OK] Датчики линии инициализированы");
  
  // Настройка пинов моторов
  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);
  Serial.println("[OK] Моторы инициализированы");
  
  // Начальная остановка моторов
  stopMotors();
  
  #ifdef USE_ENCODERS
  // Настройка энкодеров
  pinMode(ENCODER_LEFT, INPUT);
  pinMode(ENCODER_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), rightEncoderISR, RISING);
  Serial.println("[OK] Энкодеры инициализированы (режим с энкодерами)");
  #else
  Serial.println("[INFO] Режим БЕЗ энкодеров");
  #endif
  
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║  Настройки:                               ║");
  Serial.println("╠════════════════════════════════════════════╣");
  Serial.printf("║  PID: Kp=%.1f Ki=%.1f Kd=%.1f        ║\n", Kp, Ki, Kd);
  Serial.printf("║  Скорость: базовая=%d макс=%d         ║\n", baseSpeed, maxSpeed);
  Serial.println("╚════════════════════════════════════════════╝\n");
  
  Serial.println("Робот готов к работе!");
  Serial.println("Поместите робота на линию и отправьте 's' для старта");
  Serial.println("Команды: s=старт, p=стоп, c=калибровка\n");
  
  currentState = IDLE;
}

// ═══════════════════════════════════════════════════════════════════════════
// LOOP - ОСНОВНОЙ ЦИКЛ
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  // Обработка команд из Serial
  handleSerialCommands();
  
  // Основной алгоритм работы
  switch (currentState) {
    case IDLE:
      // Ожидание команды старта
      stopMotors();
      break;
      
    case CALIBRATING:
      calibrateSensors();
      break;
      
    case FOLLOWING:
      followLine();
      break;
      
    case SEARCHING_LEFT:
    case SEARCHING_RIGHT:
      searchLine();
      break;
      
    case LOST:
      stopMotors();
      Serial.println("⚠ ЛИНИЯ ПОТЕРЯНА! Отправьте 's' для повторного поиска");
      currentState = IDLE;
      break;
      
    case STOPPED:
      stopMotors();
      break;
  }
  
  // Обновление скоростей по энкодерам
  #ifdef USE_ENCODERS
  updateEncoderSpeeds();
  #endif
  
  delay(10);  // Небольшая задержка для стабильности
}

// ═══════════════════════════════════════════════════════════════════════════
// ЧТЕНИЕ ДАТЧИКОВ ЛИНИИ
// ═══════════════════════════════════════════════════════════════════════════

void readSensors(int sensors[5]) {
  // Чтение цифровых значений с датчиков
  // 0 = черная линия (LOW), 1 = белое поле (HIGH)
  sensors[0] = digitalRead(SENSOR_1);
  sensors[1] = digitalRead(SENSOR_2);
  sensors[2] = digitalRead(SENSOR_3);
  sensors[3] = digitalRead(SENSOR_4);
  sensors[4] = digitalRead(SENSOR_5);
}

// ═══════════════════════════════════════════════════════════════════════════
// РАСЧЕТ ПОЗИЦИИ ЛИНИИ (ВЗВЕШЕННЫЙ МЕТОД)
// ═══════════════════════════════════════════════════════════════════════════

float calculateLinePosition(int sensors[5]) {
  /*
   * Вычисляет позицию линии относительно центра робота
   * 
   * Веса датчиков: -2, -1, 0, +1, +2 (центр в нуле)
   * Черная линия = 0, Белое поле = 1
   * Инвертируем: линия должна давать вес, поле - нет
   * 
   * Возврат:
   *   -2.0 ... +2.0 - позиция линии
   *   -999 - линия не найдена (все датчики на белом)
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
  
  return position;
}

// ═══════════════════════════════════════════════════════════════════════════
// ПИД-РЕГУЛЯТОР
// ═══════════════════════════════════════════════════════════════════════════

float calculatePID(float error) {
  /*
   * Вычисляет корректировку рулевого управления на основе ошибки
   * 
   * error: -2.0 (линия слева) до +2.0 (линия справа)
   * Возврат: корректировка для моторов
   */
  
  // P - пропорциональная составляющая
  float P = error;
  
  // I - интегральная составляющая (накопленная ошибка)
  integral += error;
  // Анти-windup: ограничиваем интеграл
  if (integral > 100) integral = 100;
  if (integral < -100) integral = -100;
  float I = integral;
  
  // D - дифференциальная составляющая (скорость изменения ошибки)
  float D = error - previousError;
  previousError = error;
  
  // Итоговая корректировка
  float correction = Kp * P + Ki * I + Kd * D;
  
  return correction;
}

// ═══════════════════════════════════════════════════════════════════════════
// СЛЕДОВАНИЕ ПО ЛИНИИ
// ═══════════════════════════════════════════════════════════════════════════

void followLine() {
  int sensors[5];
  readSensors(sensors);
  
  float position = calculateLinePosition(sensors);
  
  // Проверка: линия найдена?
  if (position == -999) {
    // Линия потеряна - начинаем поиск
    Serial.println("⚠ Линия потеряна! Начинаю поиск...");
    currentState = SEARCHING_LEFT;
    searchStartTime = millis();
    return;
  }
  
  // Вычисляем ошибку (отклонение от центра)
  float error = position;  // position уже в диапазоне -2..+2
  
  // ПИД-регулятор
  float correction = calculatePID(error);
  
  // Применяем корректировку к скоростям моторов
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;
  
  // Ограничиваем скорости
  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);
  
  // Устанавливаем скорости моторов
  #ifdef USE_ENCODERS
  setMotorSpeedWithEncoders(leftSpeed, rightSpeed);
  #else
  setMotorSpeed(leftSpeed, rightSpeed);
  #endif
  
  // Отладочный вывод
  #ifdef DEBUG_MODE
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 200) {  // Каждые 200 мс
    Serial.print("Датчики: ");
    for (int i = 0; i < 5; i++) {
      Serial.print(sensors[i]);
      Serial.print(" ");
    }
    Serial.printf("| Позиция: %.2f | Ошибка: %.2f | Коррекция: %.1f | Моторы: L=%d R=%d\n",
                  position, error, correction, leftSpeed, rightSpeed);
    lastDebugTime = millis();
  }
  #endif
}

// ═══════════════════════════════════════════════════════════════════════════
// ПОИСК ЛИНИИ
// ═══════════════════════════════════════════════════════════════════════════

void searchLine() {
  int sensors[5];
  readSensors(sensors);
  
  float position = calculateLinePosition(sensors);
  
  // Проверяем, нашли ли линию
  if (position != -999) {
    Serial.println("✓ Линия найдена! Продолжаю движение");
    currentState = FOLLOWING;
    integral = 0;  // Сбрасываем интеграл
    previousError = 0;
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
    turnLeft(turnSpeed);
    
    // Переключаемся на поиск вправо через половину времени
    if (millis() - searchStartTime > SEARCH_TIMEOUT / 2) {
      Serial.println("→ Переключаюсь на поиск вправо");
      currentState = SEARCHING_RIGHT;
    }
  } else {
    turnRight(turnSpeed);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// УПРАВЛЕНИЕ МОТОРАМИ (БЕЗ ЭНКОДЕРОВ)
// ═══════════════════════════════════════════════════════════════════════════

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  /*
   * Устанавливает скорость моторов через ШИМ на пинах INx
   * 
   * Так как ENA/ENB припаяны к HIGH, используем ШИМ на IN1/IN3 для скорости
   */
  
  // Левый мотор
  if (leftSpeed >= 0) {
    analogWrite(MOTOR_LEFT_FWD, leftSpeed);
    digitalWrite(MOTOR_LEFT_BWD, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_FWD, LOW);
    analogWrite(MOTOR_LEFT_BWD, -leftSpeed);
  }
  
  // Правый мотор
  if (rightSpeed >= 0) {
    analogWrite(MOTOR_RIGHT_FWD, rightSpeed);
    digitalWrite(MOTOR_RIGHT_BWD, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_FWD, LOW);
    analogWrite(MOTOR_RIGHT_BWD, -rightSpeed);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// УПРАВЛЕНИЕ МОТОРАМИ (С ЭНКОДЕРАМИ)
// ═══════════════════════════════════════════════════════════════════════════

#ifdef USE_ENCODERS
void setMotorSpeedWithEncoders(int targetLeft, int targetRight) {
  /*
   * Устанавливает скорость с учетом обратной связи от энкодеров
   * Это обеспечивает синхронизацию колес и компенсацию проскальзывания
   */
  
  // Здесь можно добавить ПИД для каждого мотора отдельно
  // Для простоты используем прямое управление
  setMotorSpeed(targetLeft, targetRight);
}

void updateEncoderSpeeds() {
  /*
   * Обновляет скорости колес на основе энкодеров
   */
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastUpdateTime >= 100) {  // Обновление каждые 100мс
    unsigned long deltaTime = currentTime - lastUpdateTime;
    
    // Вычисляем пройденное расстояние
    float leftDistance = leftEncoderTicks * MM_PER_TICK;
    float rightDistance = rightEncoderTicks * MM_PER_TICK;
    
    // Вычисляем скорости (мм/сек)
    leftSpeed = (leftDistance / deltaTime) * 1000.0;
    rightSpeed = (rightDistance / deltaTime) * 1000.0;
    
    // Сбрасываем счетчики
    leftEncoderTicks = 0;
    rightEncoderTicks = 0;
    lastUpdateTime = currentTime;
    
    #ifdef DEBUG_MODE
    // Serial.printf("Скорости: Левое=%.1f мм/с, Правое=%.1f мм/с\n", leftSpeed, rightSpeed);
    #endif
  }
}
#endif

// ═══════════════════════════════════════════════════════════════════════════
// БАЗОВЫЕ ФУНКЦИИ УПРАВЛЕНИЯ МОТОРАМИ
// ═══════════════════════════════════════════════════════════════════════════

void stopMotors() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void turnLeft(int speed) {
  // Поворот на месте влево: левый назад, правый вперед
  analogWrite(MOTOR_LEFT_FWD, 0);
  analogWrite(MOTOR_LEFT_BWD, speed);
  analogWrite(MOTOR_RIGHT_FWD, speed);
  analogWrite(MOTOR_RIGHT_BWD, 0);
}

void turnRight(int speed) {
  // Поворот на месте вправо: левый вперед, правый назад
  analogWrite(MOTOR_LEFT_FWD, speed);
  analogWrite(MOTOR_LEFT_BWD, 0);
  analogWrite(MOTOR_RIGHT_FWD, 0);
  analogWrite(MOTOR_RIGHT_BWD, speed);
}

void moveForward(int speed) {
  analogWrite(MOTOR_LEFT_FWD, speed);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  analogWrite(MOTOR_RIGHT_FWD, speed);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void moveBackward(int speed) {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  analogWrite(MOTOR_LEFT_BWD, speed);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  analogWrite(MOTOR_RIGHT_BWD, speed);
}

// ═══════════════════════════════════════════════════════════════════════════
// КАЛИБРОВКА ДАТЧИКОВ
// ═══════════════════════════════════════════════════════════════════════════

void calibrateSensors() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║         КАЛИБРОВКА ДАТЧИКОВ               ║");
  Serial.println("╠════════════════════════════════════════════╣");
  Serial.println("║ 1. Медленно водите робота над линией      ║");
  Serial.println("║ 2. Убедитесь что все датчики попадают     ║");
  Serial.println("║    на белое поле и черную линию           ║");
  Serial.println("║ 3. Калибровка займет 5 секунд             ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
  
  delay(2000);
  Serial.println("Начинаю калибровку...");
  
  // Сброс мин/макс значений
  for (int i = 0; i < 5; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }
  
  // Медленное вращение и сбор данных
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    int sensors[5];
    readSensors(sensors);
    
    for (int i = 0; i < 5; i++) {
      if (sensors[i] < sensorMin[i]) sensorMin[i] = sensors[i];
      if (sensors[i] > sensorMax[i]) sensorMax[i] = sensors[i];
    }
    
    // Медленное вращение на месте для калибровки
    turnLeft(80);
    delay(50);
  }
  
  stopMotors();
  
  Serial.println("\n✓ Калибровка завершена!");
  Serial.println("Результаты:");
  for (int i = 0; i < 5; i++) {
    Serial.printf("  Датчик %d: min=%d, max=%d\n", i + 1, sensorMin[i], sensorMax[i]);
  }
  
  Serial.println("\nОтправьте 's' для начала движения");
  currentState = IDLE;
}

// ═══════════════════════════════════════════════════════════════════════════
// ОБРАБОТКА КОМАНД ИЗ SERIAL
// ═══════════════════════════════════════════════════════════════════════════

void handleSerialCommands() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch (command) {
      case 's':
      case 'S':
        Serial.println("▶ СТАРТ - Начинаю следование по линии");
        currentState = FOLLOWING;
        integral = 0;
        previousError = 0;
        break;
        
      case 'p':
      case 'P':
        Serial.println("⏸ ПАУЗА - Остановка");
        currentState = STOPPED;
        stopMotors();
        break;
        
      case 'c':
      case 'C':
        Serial.println("⚙ Запуск калибровки датчиков");
        currentState = CALIBRATING;
        break;
        
      case '+':
        baseSpeed = constrain(baseSpeed + 10, minSpeed, maxSpeed);
        Serial.printf("Скорость увеличена: %d\n", baseSpeed);
        break;
        
      case '-':
        baseSpeed = constrain(baseSpeed - 10, minSpeed, maxSpeed);
        Serial.printf("Скорость уменьшена: %d\n", baseSpeed);
        break;
        
      case 'h':
      case 'H':
      case '?':
        printHelp();
        break;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// СПРАВКА
// ═══════════════════════════════════════════════════════════════════════════

void printHelp() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║           КОМАНДЫ УПРАВЛЕНИЯ              ║");
  Serial.println("╠════════════════════════════════════════════╣");
  Serial.println("║  s  - Старт (начать следование)          ║");
  Serial.println("║  p  - Пауза (остановить)                  ║");
  Serial.println("║  c  - Калибровка датчиков                 ║");
  Serial.println("║  +  - Увеличить скорость                  ║");
  Serial.println("║  -  - Уменьшить скорость                  ║");
  Serial.println("║  h  - Показать эту справку                ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
}
