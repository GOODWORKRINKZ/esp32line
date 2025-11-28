#include <Arduino.h>
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "PIDController.h"
#include "Encoders.h"
#include "LineFollower.h"
#include "ButtonHandler.h"

// Forward declarations
void robotTask(void* parameter);

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

// Обработчик кнопки (адаптировано из примера release-mechanism для ESP32)
// Кнопка: пин 4 → резистор 10кОм → GND, при нажатии замыкается на 3.3V (Active HIGH)
ButtonHandler button(BUTTON_PIN, false); // false = кнопка к VCC (Active HIGH)

// ═══════════════════════════════════════════════════════════════════════════
// ЗАДАЧА РОБОТА (FreeRTOS Task)
// ═══════════════════════════════════════════════════════════════════════════

void robotTask(void* parameter) {
    Serial.println("[TASK] Задача робота запущена на Core 1");
    
    while (true) {
        // Опрос кнопки (читаем и сбрасываем флаг)
        if (button.wasPressed()) {
            RobotState state = robot.getState();
            if (state == IDLE || state == STOPPED || state == LOST) {
                robot.start();
                Serial.println("[BUTTON] Старт!");
            } else {
                robot.stop();
                Serial.println("[BUTTON] Стоп!");
            }
        }
        
        // Обновление состояния робота
        robot.update();
        
        // Небольшая задержка для стабильности
        vTaskDelay(1);  // 1 тик FreeRTOS (~1ms)
    }
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
    
    // Инициализация кнопки старт/стоп с использованием ButtonHandler
    button.init();
    Serial.println("[OK] Кнопка старт/стоп инициализирована (ButtonHandler + ISR)");
    
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
    Serial.println("Повторное нажатие кнопки остановит робота\n");
    
    // Создаём задачу FreeRTOS для робота на ядре 1 (ядро 0 для WiFi/BT)
    xTaskCreatePinnedToCore(
        robotTask,        // Функция задачи
        "RobotTask",      // Название задачи
        10000,            // Размер стека (байты)
        NULL,             // Параметры
        1,                // Приоритет (1 = нормальный)
        NULL,             // Дескриптор задачи
        1                 // Ядро процессора (0 или 1)
    );
    
    Serial.println("[OK] Задача робота создана на Core 1\n");
}

// ═══════════════════════════════════════════════════════════════════════════
// LOOP - ОСНОВНОЙ ЦИКЛ (минимальная загрузка для кнопки)
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
    
}
