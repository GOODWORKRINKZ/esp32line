#include "Motors.h"

Motors::Motors() {
}

void Motors::begin() {
    // Настройка ПВМ каналов для управления моторами
    ledcSetup(PWM_CHANNEL_L_FWD, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_L_BWD, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_R_FWD, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_R_BWD, PWM_FREQ, PWM_RESOLUTION);
    
    // Привязка ПВМ каналов к пинам моторов
    ledcAttachPin(MOTOR_LEFT_FWD, PWM_CHANNEL_L_FWD);
    ledcAttachPin(MOTOR_LEFT_BWD, PWM_CHANNEL_L_BWD);
    ledcAttachPin(MOTOR_RIGHT_FWD, PWM_CHANNEL_R_FWD);
    ledcAttachPin(MOTOR_RIGHT_BWD, PWM_CHANNEL_R_BWD);
    
    stop();
}

void Motors::setSpeed(int leftSpeed, int rightSpeed) {
    rightSpeed=rightSpeed*0.8;
    leftSpeed=leftSpeed*0.8;
    /*
     * Устанавливает скорость моторов через ШИМ на пинах INx
     * Так как ENA/ENB припаяны к HIGH, используем ШИМ на IN1/IN3 для скорости
     */
        
    // Левый мотор
    if (leftSpeed >= 0) {
        ledcWrite(PWM_CHANNEL_L_FWD, leftSpeed);
        ledcWrite(PWM_CHANNEL_L_BWD, 0);
    } else {
        ledcWrite(PWM_CHANNEL_L_FWD, 0);
        ledcWrite(PWM_CHANNEL_L_BWD, -leftSpeed);
    }
    
    // Правый мотор
    if (rightSpeed >= 0) {
        ledcWrite(PWM_CHANNEL_R_FWD, rightSpeed);
        ledcWrite(PWM_CHANNEL_R_BWD, 0);
    } else {
        ledcWrite(PWM_CHANNEL_R_FWD, 0);
        ledcWrite(PWM_CHANNEL_R_BWD, -rightSpeed);
    }
}

void Motors::stop() {
    // Полная остановка - отключаем все ПВМ каналы
    ledcWrite(PWM_CHANNEL_L_FWD, 0);
    ledcWrite(PWM_CHANNEL_L_BWD, 0);
    ledcWrite(PWM_CHANNEL_R_FWD, 0);
    ledcWrite(PWM_CHANNEL_R_BWD, 0);
}

void Motors::moveForward(int speed) {
    setSpeed(speed, speed);
}

void Motors::moveBackward(int speed) {
    setSpeed(-speed, -speed);
}

void Motors::turnLeft(int speed) {
    // Поворот на месте влево: левый назад, правый вперед
    setSpeed(-speed, speed);
}

void Motors::turnRight(int speed) {
    // Поворот на месте вправо: левый вперед, правый назад
    setSpeed(speed, -speed);
}
