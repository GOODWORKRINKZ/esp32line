#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include <Arduino.h>

// Время антидребезга кнопки в миллисекундах
#define BUTTON_DEBOUNCE_TIME 150

// Тип функции обратного вызова
typedef void (*ButtonCallback)();

/**
 * @brief Класс для обработки нажатий кнопки с использованием прерываний
 * 
 * Адаптировано из примера release-mechanism для ESP32
 * Использует прерывания для надежного определения нажатий
 * с программным антидребезгом
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
     * @brief Инициализация обработчика с функцией обратного вызова
     * @param callback Функция, которая будет вызвана при нажатии кнопки
     */
    void init(ButtonCallback callback);
    
    /**
     * @brief Получить текущее состояние кнопки
     * @return true если кнопка нажата
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
    bool mLastState;             // Последнее стабильное состояние
    unsigned long mLastDebounceTime; // Время последнего изменения
    unsigned long mPressCount;   // Счетчик нажатий
    ButtonCallback mCallback;    // Функция обратного вызова
    volatile bool mButtonState;  // Текущее состояние (используется в ISR)
    volatile bool mPendingCallback; // Флаг ожидающего вызова callback
    
    // Статический указатель для доступа из ISR
    static ButtonHandler* instance;
};

#endif // BUTTON_HANDLER_H
