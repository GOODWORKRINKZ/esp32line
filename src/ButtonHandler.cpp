#include "ButtonHandler.h"

// Статический указатель на экземпляр для доступа из ISR
ButtonHandler* ButtonHandler::instance = nullptr;

ButtonHandler::ButtonHandler(int buttonPin, bool activeLow)
    : mButtonPin(buttonPin),
      mActiveLow(activeLow),
      mLastState(false),
      mLastDebounceTime(0),
      mPressCount(0),
      mCallback(nullptr),
      mButtonState(false),
      mPendingCallback(false)
{
    // Устанавливаем ссылку на текущий экземпляр
    instance = this;
}

void ButtonHandler::init(ButtonCallback callback)
{
    mCallback = callback;
    
    // Настройка пина в зависимости от типа подключения
    if (mActiveLow) {
        pinMode(mButtonPin, INPUT_PULLUP); // Кнопка к GND
    } else {
        pinMode(mButtonPin, INPUT); // Кнопка к VCC
    }
    
    // Начальное состояние
    mButtonState = digitalRead(mButtonPin);
    mLastState = mButtonState;
    
    // Подключение прерывания через статический метод
    attachInterrupt(
        digitalPinToInterrupt(mButtonPin),
        handleInterruptStatic,
        CHANGE  // Реагируем на любое изменение
    );
}

void IRAM_ATTR ButtonHandler::handleInterruptStatic()
{
    // Статический метод вызывает метод экземпляра
    if (instance != nullptr) {
        instance->handleInterrupt();
    }
}

void IRAM_ATTR ButtonHandler::handleInterrupt()
{
    // ISR должна быть максимально быстрой
    unsigned long now = millis();
    bool currentState = digitalRead(mButtonPin);
    
    // Инвертируем состояние если кнопка подключена к GND
    bool pressed = mActiveLow ? !currentState : currentState;
    
    // Простой антидребезг - игнорируем изменения быстрее DEBOUNCE_TIME
    if (now - mLastDebounceTime < BUTTON_DEBOUNCE_TIME) {
        return;
    }
    
    mLastDebounceTime = now;
    mButtonState = currentState;
    
    // Определяем нажатие (переход из отпущенного в нажатое)
    if (pressed && !mLastState) {
        mPressCount++;
        mPendingCallback = true;
        mLastState = true;
    } else if (!pressed && mLastState) {
        mLastState = false;
    }
    
    // Вызываем callback если есть ожидающий вызов
    // В ESP32 callback можно вызывать из ISR, но он должен быть помечен IRAM_ATTR
    if (mPendingCallback && mCallback != nullptr) {
        mPendingCallback = false;
        mCallback();
    }
}

bool ButtonHandler::isPressed()
{
    bool currentState = digitalRead(mButtonPin);
    return mActiveLow ? !currentState : currentState;
}
