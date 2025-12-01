#include "Buzzer.h"
#include "Config.h"

// ═══════════════════════════════════════════════════════════════════════════
// МЕЛОДИИ
// ═══════════════════════════════════════════════════════════════════════════

// Ноты (частоты в Гц)
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_G5  784
#define NOTE_A5  880

// Мелодия готовности: весёлая трель вверх
static const Note melodyReady[] = {
    {NOTE_C5, 100},
    {NOTE_E5, 100},
    {NOTE_G5, 150}
};

// Мелодия старта: два коротких бипа
static const Note melodyStart[] = {
    {NOTE_A5, 80},
    {0, 50},        // пауза
    {NOTE_A5, 80}
};

// Мелодия стопа: нисходящий тон
static const Note melodyStop[] = {
    {NOTE_G4, 100},
    {NOTE_E4, 100},
    {NOTE_C4, 150}
};

// Линия потеряна: тревожный звук
static const Note melodyLineLost[] = {
    {NOTE_A4, 150},
    {0, 50},
    {NOTE_A4, 150}
};

// Линия найдена: короткий бип
static const Note melodyLineFound[] = {
    {NOTE_E5, 80}
};

// Ошибка: низкий длинный тон
static const Note melodyError[] = {
    {200, 300},
    {0, 100},
    {200, 300}
};

// Простой бип
static const Note melodyBeep[] = {
    {NOTE_C5, 100}
};

// Массив мелодий и их длин
static const Note* melodies[] = {
    melodyReady,
    melodyStart,
    melodyStop,
    melodyLineLost,
    melodyLineFound,
    melodyError,
    melodyBeep
};

static const uint8_t melodyLengths[] = {
    sizeof(melodyReady) / sizeof(Note),
    sizeof(melodyStart) / sizeof(Note),
    sizeof(melodyStop) / sizeof(Note),
    sizeof(melodyLineLost) / sizeof(Note),
    sizeof(melodyLineFound) / sizeof(Note),
    sizeof(melodyError) / sizeof(Note),
    sizeof(melodyBeep) / sizeof(Note)
};

// ═══════════════════════════════════════════════════════════════════════════
// РЕАЛИЗАЦИЯ
// ═══════════════════════════════════════════════════════════════════════════

Buzzer::Buzzer(int pin, int channel)
    : mPin(pin),
      mChannel(channel),
      mEnabled(true),
      mPlaying(false),
      mCurrentMelody(nullptr),
      mMelodyLength(0),
      mCurrentNote(0),
      mNoteStartTime(0)
{
}

void Buzzer::begin()
{
    // Настройка LEDC для генерации тонов
    ledcSetup(mChannel, 2000, 8);  // Начальная частота 2кГц, 8 бит
    ledcAttachPin(mPin, mChannel);
    ledcWrite(mChannel, 0);  // Выключен по умолчанию
    
    Serial.println("[OK] Бузер инициализирован на пине " + String(mPin));
}

void Buzzer::update()
{
    if (!mPlaying || !mEnabled) {
        return;
    }
    
    unsigned long now = millis();
    
    // Проверяем, закончилась ли текущая нота
    if (now - mNoteStartTime >= mCurrentMelody[mCurrentNote].duration) {
        // Переходим к следующей ноте
        mCurrentNote++;
        
        if (mCurrentNote >= mMelodyLength) {
            // Мелодия закончилась
            stop();
            return;
        }
        
        // Воспроизводим следующую ноту
        mNoteStartTime = now;
        playNote(mCurrentMelody[mCurrentNote].frequency);
    }
}

void Buzzer::play(BuzzerMelody melody)
{
    if (!mEnabled) {
        return;
    }
    
    // Остановить текущее воспроизведение
    stop();
    
    // Настроить новую мелодию
    mCurrentMelody = melodies[melody];
    mMelodyLength = melodyLengths[melody];
    mCurrentNote = 0;
    mNoteStartTime = millis();
    mPlaying = true;
    
    // Начать воспроизведение первой ноты
    playNote(mCurrentMelody[0].frequency);
}

void Buzzer::tone(uint16_t frequency, uint16_t duration)
{
    if (!mEnabled) {
        return;
    }
    
    // Создаём временную "мелодию" из одной ноты
    static Note singleNote;
    singleNote.frequency = frequency;
    singleNote.duration = duration;
    
    stop();
    mCurrentMelody = &singleNote;
    mMelodyLength = 1;
    mCurrentNote = 0;
    mNoteStartTime = millis();
    mPlaying = true;
    
    playNote(frequency);
}

void Buzzer::stop()
{
    stopNote();
    mPlaying = false;
    mCurrentMelody = nullptr;
    mMelodyLength = 0;
    mCurrentNote = 0;
}

void Buzzer::playNote(uint16_t frequency)
{
    if (frequency == 0) {
        // Пауза
        ledcWrite(mChannel, 0);
    } else {
        // Установить частоту и включить
        ledcWriteTone(mChannel, frequency);
    }
}

void Buzzer::stopNote()
{
    ledcWrite(mChannel, 0);
}
