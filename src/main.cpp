#include <Arduino.h>
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "PIDController.h"
#include "Encoders.h"
#include "LineFollower.h"

// Forward declarations
void printHelp();

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

// Создание объектов компонентов
LineSensors sensors;
Motors motors;
PIDController pid;

#ifdef USE_ENCODERS
Encoders encoders;
LineFollower robot(sensors, motors, pid, &encoders);
#else
LineFollower robot(sensors, motors, pid, nullptr);
#endif

// Переменные для обработки кнопки
volatile bool buttonInterrupt = false;
unsigned long lastButtonPress = 0;

// ═══════════════════════════════════════════════════════════════════════════
// ОБРАБОТКА КНОПКИ СТАРТ/СТОП
// ═══════════════════════════════════════════════════════════════════════════

// Обработчик прерывания кнопки
void IRAM_ATTR buttonISR() {
    buttonInterrupt = true;
}

// Обработка нажатия кнопки (переключение старт/стоп)
void handleButton() {
    if (buttonInterrupt) {
        buttonInterrupt = false;
        
        // Антидребезг в основном цикле
        unsigned long currentTime = millis();
        if (currentTime - lastButtonPress < BUTTON_DEBOUNCE_MS) {
            return;
        }
        lastButtonPress = currentTime;
        
        RobotState state = robot.getState();
        if (state == IDLE || state == STOPPED || state == LOST) {
            // Робот стоит - запускаем
            robot.start();
        } else {
            // Робот едет - останавливаем
            robot.pause();
        }
    }
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
                robot.start();
                break;
                
            case 'p':
            case 'P':
                robot.pause();
                break;
                
            case 'c':
            case 'C':
                robot.calibrate();
                break;
                
            case '+':
                robot.increaseSpeed();
                break;
                
            case '-':
                robot.decreaseSpeed();
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
    Serial.println("║  КНОПКА - Старт/Стоп (переключение)       ║");
    Serial.println("║  s  - Старт (начать следование)           ║");
    Serial.println("║  p  - Пауза (остановить)                  ║");
    Serial.println("║  c  - Калибровка датчиков                 ║");
    Serial.println("║  +  - Увеличить скорость                  ║");
    Serial.println("║  -  - Уменьшить скорость                  ║");
    Serial.println("║  h  - Показать эту справку                ║");
    Serial.println("╚════════════════════════════════════════════╝\n");
}

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
    
    // Инициализация кнопки старт/стоп
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
    Serial.println("[OK] Кнопка старт/стоп инициализирована");
    
    // Инициализация робота
    robot.begin();
    
    // Вывод параметров ПИД
    float kp, ki, kd;
    pid.getGains(kp, ki, kd);
    
    Serial.println("╔════════════════════════════════════════════╗");
    Serial.println("║  Настройки:                               ║");
    Serial.println("╠════════════════════════════════════════════╣");
    Serial.printf("║  PID: Kp=%.1f Ki=%.1f Kd=%.1f        ║\n", kp, ki, kd);
    Serial.printf("║  Скорость: базовая=%d макс=%d         ║\n", robot.getBaseSpeed(), MAX_SPEED);
    
#ifdef USE_ENCODERS
    Serial.println("║  Энкодеры: ВКЛЮЧЕНЫ                       ║");
#else
    Serial.println("║  Энкодеры: ОТКЛЮЧЕНЫ                      ║");
#endif
    
    Serial.println("╚════════════════════════════════════════════╝\n");
    
    Serial.println("Робот готов к работе!");
    Serial.println("Поместите робота на линию и нажмите кнопку для старта");
    Serial.println("Повторное нажатие кнопки остановит робота");
    Serial.println("Команды Serial: s=старт, p=стоп, c=калибровка, h=справка\n");
}

// ═══════════════════════════════════════════════════════════════════════════
// LOOP - ОСНОВНОЙ ЦИКЛ
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
    // Обработка кнопки старт/стоп
    handleButton();
    
    // Обработка команд из Serial
    handleSerialCommands();
    
    // Обновление состояния робота
    robot.update();
    
    // Небольшая задержка для стабильности
    delay(10);
}
