#include "Motors.h"

Motors::Motors() {
}

void Motors::begin() {
    pinMode(MOTOR_LEFT_FWD, OUTPUT);
    pinMode(MOTOR_LEFT_BWD, OUTPUT);
    pinMode(MOTOR_RIGHT_FWD, OUTPUT);
    pinMode(MOTOR_RIGHT_BWD, OUTPUT);
    
    stop();
}

void Motors::setSpeed(int leftSpeed, int rightSpeed) {
    /*
     * Устанавливает скорость моторов через ШИМ на пинах INx
     * Так как ENA/ENB припаяны к HIGH, используем ШИМ на IN1/IN3 для скорости
     */
        
    // Левый мотор
    if (leftSpeed >= 0) {
        analogWrite(MOTOR_LEFT_FWD, leftSpeed);
        digitalWrite(MOTOR_LEFT_BWD, LOW);
    } else {
        digitalWrite(MOTOR_LEFT_FWD, LOW);
        analogWrite(MOTOR_LEFT_BWD, -leftSpeed);
    }
    
    // Правый мотор
    if (rightSpeed >= 0) {
        analogWrite(MOTOR_RIGHT_FWD, rightSpeed);
        digitalWrite(MOTOR_RIGHT_BWD, LOW);
    } else {
        digitalWrite(MOTOR_RIGHT_FWD, LOW);
        analogWrite(MOTOR_RIGHT_BWD, -rightSpeed);
    }
}

void Motors::stop() {
    // Полная остановка - сначала отключаем ШИМ, потом все пины в LOW
    analogWrite(MOTOR_LEFT_FWD, 0);
    analogWrite(MOTOR_LEFT_BWD, 0);
    analogWrite(MOTOR_RIGHT_FWD, 0);
    analogWrite(MOTOR_RIGHT_BWD, 0);
    
    // Затем явно устанавливаем LOW
    digitalWrite(MOTOR_LEFT_FWD, LOW);
    digitalWrite(MOTOR_LEFT_BWD, LOW);
    digitalWrite(MOTOR_RIGHT_FWD, LOW);
    digitalWrite(MOTOR_RIGHT_BWD, LOW);
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
