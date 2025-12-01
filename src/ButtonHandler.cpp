#include "ButtonHandler.h"

// Статический указатель на экземпляр для доступа из ISR
ButtonHandler* ButtonHandler::instance = nullptr;

ButtonHandler::ButtonHandler(int buttonPin, bool activeLow)
    : mButtonPin(buttonPin),
      mActiveLow(activeLow),
      mLastDebounceTime(0),
      mPressCount(0),
      mIsHeld(false),
      mWasReleased(false),
      mPressStartTime(0)
{
    // Устанавливаем ссылку на текущий экземпляр
    instance = this;
}

void ButtonHandler::init()
{
    // Настройка пина - просто INPUT
    // Внешняя схема определяет уровень (pullup/pulldown)
    pinMode(mButtonPin, INPUT);
    
    // Начальное состояние
    mIsHeld = false;
    mWasReleased = false;
    mPressStartTime = 0;
    
    // Подключение прерывания на CHANGE (оба фронта - и нажатие и отпускание)
    attachInterrupt(
        digitalPinToInterrupt(mButtonPin),
        handleInterruptStatic,
        CHANGE  // Срабатываем на любое изменение
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
    
    // Антидребезг - увеличен для надёжности при CHANGE
    if (now - mLastDebounceTime < BUTTON_DEBOUNCE_TIME) {
        return;
    }
    
    // Читаем текущее состояние пина
    bool pinState = digitalRead(mButtonPin);
    bool isPressed = mActiveLow ? !pinState : pinState;
    
    if (isPressed) {
        // Кнопка НАЖАТА
        if (!mIsHeld) {
            // Первое нажатие - запоминаем время
            mIsHeld = true;
            mPressStartTime = now;
            mLastDebounceTime = now;
        }
        // Если уже удерживается - игнорируем (дребезг)
    }
    else {
        // Кнопка ОТПУЩЕНА
        if (mIsHeld) {
            // Было нажатие - проверяем валидность
            mIsHeld = false;
            mLastDebounceTime = now;
            
            // Проверяем минимальное время удержания
            if (now - mPressStartTime >= BUTTON_MIN_PRESS_TIME) {
                mPressCount++;
                mWasReleased = true;  // Флаг для wasPressed()
            }
        }
        // Если не было нажатия - игнорируем (дребезг или начальное состояние)
    }
}

bool ButtonHandler::wasPressed()
{
    // Атомарное чтение и сброс флага
    // Возвращает true только после полного цикла: нажал → отпустил
    bool released = mWasReleased;
    if (released) {
        mWasReleased = false;
    }
    return released;
}

bool ButtonHandler::isPressed()
{
    // Возвращает true если кнопка СЕЙЧАС удерживается
    return mIsHeld;
}
