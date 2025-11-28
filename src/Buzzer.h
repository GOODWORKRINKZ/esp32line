#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

// Мелодии (индексы)
enum BuzzerMelody {
    MELODY_READY = 0,      // Готовность (при старте системы)
    MELODY_START,          // Старт движения
    MELODY_STOP,           // Остановка
    MELODY_LINE_LOST,      // Линия потеряна
    MELODY_LINE_FOUND,     // Линия найдена
    MELODY_ERROR,          // Ошибка
    MELODY_BEEP            // Простой бип
};

// Структура для ноты
struct Note {
    uint16_t frequency;    // Частота в Гц (0 = пауза)
    uint16_t duration;     // Длительность в мс
};

/**
 * @brief Неблокирующий класс для управления бузером
 * 
 * Использует ESP32 LEDC для генерации тонов
 * Проигрывает мелодии в фоне, не блокируя основной цикл
 */
class Buzzer {
public:
    Buzzer(int pin, int channel = 0);
    
    /**
     * @brief Инициализация бузера
     */
    void begin();
    
    /**
     * @brief Обновление состояния (вызывать в loop)
     */
    void update();
    
    /**
     * @brief Воспроизвести мелодию
     * @param melody Индекс мелодии
     */
    void play(BuzzerMelody melody);
    
    /**
     * @brief Воспроизвести одиночный тон
     * @param frequency Частота в Гц
     * @param duration Длительность в мс
     */
    void tone(uint16_t frequency, uint16_t duration);
    
    /**
     * @brief Остановить воспроизведение
     */
    void stop();
    
    /**
     * @brief Проверить, воспроизводится ли что-то
     */
    bool isPlaying() const { return mPlaying; }
    
    /**
     * @brief Включить/выключить звук
     */
    void setEnabled(bool enabled) { mEnabled = enabled; }
    bool isEnabled() const { return mEnabled; }

private:
    void playNote(uint16_t frequency);
    void stopNote();
    
    int mPin;
    int mChannel;
    bool mEnabled;
    bool mPlaying;
    
    // Текущая мелодия
    const Note* mCurrentMelody;
    uint8_t mMelodyLength;
    uint8_t mCurrentNote;
    unsigned long mNoteStartTime;
};

#endif // BUZZER_H
