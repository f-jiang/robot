#include <stdio.h>

#include <Arduino.h>

#include <Encoder.h>
#include <PID_v1.h>

// TODO set and correct all names
#define MOTOR_LEFT_EN 11    // EN_B
#define MOTOR_RIGHT_EN 10   // EN_A
#define MOTOR_IN_1 6
#define MOTOR_IN_2 9
#define MOTOR_IN_3 12
#define MOTOR_IN_4 13

// ENC PINS
#define ENC_LEFT_A  2
#define ENC_LEFT_B  7
#define ENC_RIGHT_A 3
#define ENC_RIGHT_B 8

#define ENC_PULSES_PER_REVOLUTION 420

struct PidController {
    double input = 0;   // TODO private w/ getters
    double output = 0;
    double setpoint = 0;
    PID controller = PID(&input, &output, &setpoint, 0, 0, 0, DIRECT);
};

Encoder encLeft(ENC_LEFT_A, ENC_LEFT_B);
Encoder encRight(ENC_RIGHT_A, ENC_RIGHT_B);
unsigned long encLastTime;
long positionLeft = 0;
long positionRight = 0;

PidController pidLeft;
PidController pidRight;

// rad/s
float angularVelocity(const long& oldPosition,
                      const long& newPosition,
                      const unsigned long& oldTime,
                      const unsigned long& newTime) {
    return (newPosition - oldPosition)
           / static_cast<float>(ENC_PULSES_PER_REVOLUTION)
           * (PI * 2)
           / ((newTime - oldTime) / 1000.0f);
}

void setup()
{
    Serial.begin(57600);

    // MOTOR SETUP
    pinMode(MOTOR_RIGHT_EN, OUTPUT);
    pinMode(MOTOR_LEFT_EN, OUTPUT);
    pinMode(MOTOR_IN_1, OUTPUT);
    pinMode(MOTOR_IN_2, OUTPUT);
    pinMode(MOTOR_IN_3, OUTPUT);
    pinMode(MOTOR_IN_4, OUTPUT);

    pidLeft.input = 0;
    pidLeft.setpoint = 0;
    pidLeft.controller.SetMode(AUTOMATIC);
    pidLeft.controller.SetOutputLimits(-255, 255);
//    pidLeft.controller.SetTunings(5, .5, 0.05);
//    pidLeft.controller.SetTunings(4, 10, 0.01);
    pidLeft.controller.SetTunings(2, 16, 0.15);

    pidRight.input = 0;
    pidRight.setpoint = 0;
    pidRight.controller.SetMode(AUTOMATIC);
    pidRight.controller.SetOutputLimits(-255, 255);
    pidRight.controller.SetTunings(2, 16, 0.15);

    delay(3000);
}

void loop()
{
    // ENCODER
    long newPositionLeft, newPositionRight;
    float angularVelocityLeft, angularVelocityRight;
    newPositionLeft = encLeft.read();   // TODO mod?
    newPositionRight = -encRight.read();    // correct direction
    unsigned long encNow = millis();
    angularVelocityLeft = angularVelocity(positionLeft, newPositionLeft, encLastTime, encNow);
    angularVelocityRight = angularVelocity(positionRight, newPositionRight, encLastTime, encNow);
#ifdef DEBUG
    Serial.print("pos Left = ");
    Serial.print(newPositionLeft);
    Serial.print(", pos Right = ");
    Serial.print(newPositionRight);
    Serial.print(", ang vel Left (rad/s) = ");
    Serial.print(angularVelocityLeft);
    Serial.print(", ang vel Right (rad/s) = ");
    Serial.print(angularVelocityRight);
    Serial.print(", ang vel Left (deg/s) = ");
    Serial.print(angularVelocityLeft * 180 / PI);
    Serial.print(", ang vel Right (deg/s) = ");
    Serial.print(angularVelocityRight * 180 / PI);
    Serial.print(", ang vel Left (rpm) = ");
    Serial.print(angularVelocityLeft / (2 * PI) * 60);
    Serial.print(", ang vel Right (rpm) = ");
    Serial.print(angularVelocityRight / (2 * PI) * 60);
    Serial.println();
#endif
    encLastTime = encNow;
    positionLeft = newPositionLeft;
    positionRight = newPositionRight;

#ifndef DEBUG
    String buf;

    // write wheels' ang. velocity and position
    buf += angularVelocityLeft;
    buf += ',';
    buf += angularVelocityRight;
    buf += ',';
    buf += positionLeft;
    buf += ',';
    buf += positionRight;
    buf += ';';
    Serial.print(buf);

    // TODO maybe reaeding too soon
    // read setpoints
    buf.remove(0);
    while (Serial.available() > 0) {
        buf += (char) Serial.read();
        delay(10);
    }
#endif

    // PID
#define RPM_TO_RAD_S(val)   (val / 60.0f * 2 * PI)
    pidLeft.input = angularVelocityLeft;
    pidLeft.setpoint = RPM_TO_RAD_S(600);
    pidLeft.controller.Compute();
    pidRight.input = angularVelocityRight;
    pidRight.setpoint = RPM_TO_RAD_S(600);
    pidRight.controller.Compute();
#undef  RAD_S_TO_RPM

    // left motor
    int leftMotorValue = abs(pidLeft.output);
    // forward
    if (pidLeft.output > 0) {
        digitalWrite(MOTOR_IN_1, LOW);
        digitalWrite(MOTOR_IN_2, HIGH);
    // backward
    } else {
        digitalWrite(MOTOR_IN_1, HIGH);
        digitalWrite(MOTOR_IN_2, LOW);
    }
    // TODO deadzone
    analogWrite(MOTOR_LEFT_EN, leftMotorValue);

    // right motor
    int rightMotorValue = abs(pidRight.output);
    // forward
    if (pidRight.output > 0) {
        digitalWrite(MOTOR_IN_3, HIGH);
        digitalWrite(MOTOR_IN_4, LOW);
    // backward
    } else {
        digitalWrite(MOTOR_IN_3, LOW);
        digitalWrite(MOTOR_IN_4, HIGH);
    }
    analogWrite(MOTOR_RIGHT_EN, rightMotorValue);
}

