#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include <Arduino.h>

// Таймиги кнопки в миллисекундах
#define BUTTON_DEBOUNCE_TIME 100      // Антидребезг (для CHANGE режима нужен больше)
#define BUTTON_MIN_PRESS_TIME 50      // Минимальное время удержания для валидного нажатия

/**
 * @brief Класс для обработки нажатий кнопки с моделью press-release
 * 
 * Действие срабатывает только когда пользователь:
 * 1. Нажал кнопку (зафиксировали)
 * 2. Отпустил кнопку (только тогда wasPressed() вернёт true)
 * 
 * Это защищает от случайных срабатываний и делает управление предсказуемым.
 */
class ButtonHandler
{
public:
    /**
     * @brief Конструктор обработчика кнопки
     * @param buttonPin Номер пина, к которому подключена кнопка
     * @param activeLow true если кнопка подключена к GND (по умолчанию)
     */
    ButtonHandler(int buttonPin, bool activeLow = true);
    
    /**
     * @brief Инициализация обработчика (без callback)
     */
    void init();
    
    /**
     * @brief Проверить и сбросить флаг нажатия
     * @return true если кнопка была нажата с последней проверки
     */
    bool wasPressed();
    
    /**
     * @brief Получить текущее состояние кнопки
     * @return true если кнопка нажата в данный момент
     */
    bool isPressed();
    
    /**
     * @brief Получить количество нажатий с момента последнего сброса
     * @return Счетчик нажатий
     */
    unsigned long getPressCount() { return mPressCount; }
    
    /**
     * @brief Сбросить счетчик нажатий
     */
    void resetPressCount() { mPressCount = 0; }

private:
    /**
     * @brief Статический обработчик прерывания (ISR)
     */
    static void IRAM_ATTR handleInterruptStatic();
    
    /**
     * @brief Обработчик прерывания (ISR) для конкретного экземпляра
     */
    void IRAM_ATTR handleInterrupt();
    
    int mButtonPin;              // Номер пина кнопки
    bool mActiveLow;             // true если кнопка подключена к GND
    unsigned long mLastDebounceTime; // Время последнего изменения
    unsigned long mPressCount;   // Счетчик нажатий
    
    // Состояния для модели press-release
    volatile bool mIsHeld;       // Кнопка сейчас удерживается
    volatile bool mWasReleased;  // Флаг: кнопка была отпущена (готово к обработке)
    volatile unsigned long mPressStartTime;  // Когда начали нажатие
    
    // Статический указатель для доступа из ISR
    static ButtonHandler* instance;
};

#endif // BUTTON_HANDLER_H
