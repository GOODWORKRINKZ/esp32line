# ButtonHandler - Обработчик кнопки с прерываниями

## Описание

Класс `ButtonHandler` реализует надежную обработку нажатий кнопки для ESP32 с использованием аппаратных прерываний и программного антидребезга.

**Адаптировано из примера** `release-mechanism` (STM32) для ESP32.

## Ключевые особенности

✅ **Прерывания** - использует аппаратные прерывания для мгновенной реакции  
✅ **Антидребезг** - программный антидребезг 50 мс  
✅ **Callback** - вызывает пользовательскую функцию при нажатии  
✅ **Счетчик** - подсчитывает количество нажатий  
✅ **Универсальность** - поддержка Active Low (к GND) и Active High (к VCC)

## Принцип работы

### Из примера release-mechanism

В примере использовался класс `PwmHandler` для обработки PWM-сигнала кнопки:

```cpp
// STM32 (оригинал)
class PwmHandler {
    void Init(callback_function_t callback) {
        attachInterrupt(
            digitalPinToInterrupt(mPwmPin), 
            [this]() { this->ISR_pwm(); },
            CHANGE
        );
    }
};
```

### Адаптация для ESP32

ESP32 Arduino 2.0.14 не поддерживает лямбда-функции в `attachInterrupt`, поэтому использован **статический метод**:

```cpp
// ESP32 (адаптировано)
class ButtonHandler {
    static void IRAM_ATTR handleInterruptStatic() {
        if (instance != nullptr) {
            instance->handleInterrupt();
        }
    }
};
```

### Алгоритм работы

1. **Прерывание на CHANGE** - срабатывает при любом изменении уровня
2. **Антидребезг** - игнорируем изменения быстрее 50 мс
3. **Определение фронта** - детектируем переход 0→1 (нажатие)
4. **Callback** - вызываем пользовательскую функцию из ISR
5. **Счетчик** - увеличиваем счетчик нажатий

## Использование

### Пример из main.cpp

```cpp
#include "ButtonHandler.h"

// Создание обработчика (пин 4, кнопка к GND)
ButtonHandler button(BUTTON_PIN, true);

// Callback-функция (вызывается из прерывания!)
void IRAM_ATTR onButtonPressed()
{
    // Ваша логика обработки нажатия
    robot.toggleState();
    Serial.println("[BUTTON] Нажата!");
}

void setup()
{
    // Инициализация с callback
    button.init(onButtonPressed);
}

void loop()
{
    // Обработка идет в прерывании!
    // В loop() ничего делать не нужно
}
```

### API

#### Конструктор

```cpp
ButtonHandler(int buttonPin, bool activeLow = true)
```

- `buttonPin` - номер GPIO пина
- `activeLow` - `true` если кнопка подключена к GND (по умолчанию)

#### Методы

```cpp
void init(ButtonCallback callback)
```
Инициализация с функцией обратного вызова.

```cpp
bool isPressed()
```
Получить текущее состояние кнопки.

```cpp
unsigned long getPressCount()
```
Получить количество нажатий с момента последнего сброса.

```cpp
void resetPressCount()
```
Сбросить счетчик нажатий.

## Подключение кнопки

### Схема (Active Low - к GND)

```
ESP32 GPIO 4 ────┐
                 │
              [BUTTON]
                 │
                GND
```

Pull-up резистор включается автоматически (`INPUT_PULLUP`).

### Схема (Active High - к VCC)

```
        3.3V ────┐
                 │
              [BUTTON]
                 │
ESP32 GPIO 4 ────┴──── (10kΩ to GND)
```

Требуется внешний pull-down резистор 10 кΩ.

## Технические детали

### Антидребезг

Механические кнопки "дребезжат" при нажатии - контакт замыкается-размыкается несколько раз за 10-50 мс. ButtonHandler игнорирует изменения быстрее `BUTTON_DEBOUNCE_TIME` (50 мс).

### Прерывания ESP32

- `IRAM_ATTR` - размещает функцию в RAM для быстрого выполнения
- ISR должна быть максимально быстрой
- Можно вызывать `Serial.println()` (но не рекомендуется)
- Callback вызывается непосредственно из ISR

### Статический экземпляр

Из-за ограничений ESP32 Arduino используется **singleton pattern**:

```cpp
static ButtonHandler* instance = nullptr;  // Глобальный указатель
```

⚠️ **Ограничение**: можно использовать только **один** экземпляр ButtonHandler.

Для нескольких кнопок нужно модифицировать класс (массив экземпляров или map).

## Отличия от оригинала (release-mechanism)

| Параметр | STM32 (оригинал) | ESP32 (адаптация) |
|----------|------------------|-------------------|
| Название класса | `PwmHandler` | `ButtonHandler` |
| Сигнал | PWM (1000-2000 мкс) | Цифровой (0/1) |
| Лямбда в ISR | ✅ Поддерживается | ❌ Не поддерживается |
| Статический метод | ❌ Не нужен | ✅ Обязателен |
| Антидребезг | ✅ По толерантности | ✅ По времени |
| Callback из ISR | ✅ Да | ✅ Да |

## Преимущества перед старым кодом

### Старый код (main.cpp)

```cpp
volatile bool buttonInterrupt = false;
unsigned long lastButtonPress = 0;

void IRAM_ATTR buttonISR() {
    buttonInterrupt = true;
}

void handleButton() {
    if (buttonInterrupt) {
        // Антидребезг в loop()
        if (millis() - lastButtonPress < DEBOUNCE) return;
        // Обработка...
        buttonInterrupt = false;
    }
}

void loop() {
    handleButton();  // Нужно вызывать в loop()!
}
```

### Новый код (ButtonHandler)

```cpp
ButtonHandler button(BUTTON_PIN, true);

void IRAM_ATTR onButtonPressed() {
    robot.toggleState();  // Сразу обработка!
}

void setup() {
    button.init(onButtonPressed);
}

void loop() {
    // Ничего не нужно!
}
```

**Преимущества:**
- ✅ Меньше глобальных переменных
- ✅ Не нужно вызывать в `loop()`
- ✅ Инкапсуляция логики в класс
- ✅ Антидребезг встроен
- ✅ Счетчик нажатий бесплатно
- ✅ Легко переносить в другие проекты

## Настройка

### Изменение времени антидребезга

В файле `ButtonHandler.h`:

```cpp
#define BUTTON_DEBOUNCE_TIME 50  // Измените на нужное значение (мс)
```

Рекомендуемые значения:
- **20-30 мс** - качественные кнопки
- **50 мс** - обычные кнопки (по умолчанию)
- **100+ мс** - плохие/старые кнопки

## Отладка

### Проверка работы

```cpp
void IRAM_ATTR onButtonPressed()
{
    Serial.printf("[BTN] Count: %lu\n", button.getPressCount());
}
```

### Просмотр состояния

```cpp
void loop()
{
    if (button.isPressed()) {
        Serial.println("Кнопка НАЖАТА");
    }
    delay(100);
}
```

## Лицензия

Адаптировано из примера `release-mechanism`.  
MIT License

---

**Автор адаптации**: GOODWORKRINKZ  
**Дата**: Ноябрь 2025
