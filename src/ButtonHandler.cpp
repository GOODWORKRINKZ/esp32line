#include "ButtonHandler.h"

// Статический указатель на экземпляр для доступа из ISR
ButtonHandler* ButtonHandler::instance = nullptr;

ButtonHandler::ButtonHandler(int buttonPin, bool activeLow)
    : mButtonPin(buttonPin),
      mActiveLow(activeLow),
      mLastState(false),
      mLastDebounceTime(0),
      mPressCount(0),
      mButtonState(false),
      mPressedFlag(false)
{
    // Устанавливаем ссылку на текущий экземпляр
    instance = this;
}

void ButtonHandler::init()
{
    // Настройка пина - просто INPUT как в Opener
    // Внешняя схема определяет уровень (pullup/pulldown)
    pinMode(mButtonPin, INPUT);
    
    // Начальное состояние
    mButtonState = digitalRead(mButtonPin);
    mLastState = mButtonState;
    
    // Подключение прерывания на FALLING (нажатие = переход HIGH→LOW)
    // Если кнопка подтянута к VCC и замыкает на GND
    attachInterrupt(
        digitalPinToInterrupt(mButtonPin),
        handleInterruptStatic,
        FALLING  // Срабатываем на спад сигнала (нажатие кнопки)
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
    // ISR должна быть максимально быстрой (как в Opener)
    unsigned long now = millis();
    
    // Простой антидребезг - игнорируем изменения быстрее DEBOUNCE_TIME
    if (now - mLastDebounceTime < BUTTON_DEBOUNCE_TIME) {
        return;
    }
    
    mLastDebounceTime = now;
    mPressCount++;
    mPressedFlag = true;  // Устанавливаем флаг нажатия
}

bool ButtonHandler::wasPressed()
{
    // Атомарное чтение и сброс флага
    bool pressed = mPressedFlag;
    if (pressed) {
        mPressedFlag = false;
    }
    return pressed;
}

bool ButtonHandler::isPressed()
{
    bool currentState = digitalRead(mButtonPin);
    return mActiveLow ? !currentState : currentState;
}
