#include <ros.h>
#include <std_msgs/Float32.h>

#include <stdio.h>

#include <Arduino.h>

// NOTE: currently this lib and the encoder code here rely on an unreleased fix for the arduino mbed core:
// https://github.com/arduino/ArduinoCore-nRF528x-mbedos/pull/69
#include <QuadratureEncoder.h>

// #define DEBUG

#define MOTOR_LB    6   // left backwards
#define MOTOR_LF    7   // left forward
#define MOTOR_RB    8   // right backwards
#define MOTOR_RF    9   // right forward
#define MOTOR_SLEEP 10
#define MOTOR_FAULT 11

#define ENC_LEFT_A  4
#define ENC_LEFT_B  5
#define ENC_RIGHT_A 2
#define ENC_RIGHT_B 3

#define ENC_PULSES_PER_REVOLUTION (51.45 * 12)

Encoders encLeft(ENC_LEFT_A, ENC_LEFT_B);
Encoders encRight(ENC_RIGHT_A, ENC_RIGHT_B);
unsigned long encLastTime;
long positionLeft = 0;
long positionRight = 0;

#ifdef DEBUG
char buf[256];
#else
// TODO integrate this into catkin so that robot_constants can be used here

ros::NodeHandle nh;

std_msgs::Float32 encMsg;
ros::Publisher pubLeftPos("/left_wheel/pos", &encMsg);
ros::Publisher pubRightPos("/right_wheel/pos", &encMsg);
ros::Publisher pubLeftVel("/left_wheel/vel", &encMsg);
ros::Publisher pubRightVel("/right_wheel/vel", &encMsg);

void leftEffCallback(const std_msgs::Float32& msg);
void rightEffCallback(const std_msgs::Float32& msg);
ros::Subscriber<std_msgs::Float32> subLeftEff("/left_wheel/eff", &leftEffCallback);
ros::Subscriber<std_msgs::Float32> subRightEff("/right_wheel/eff", &rightEffCallback);

void setMotor(byte fwdPin, byte bkdPin, short val) {
    if (val > 0) {
        analogWrite(fwdPin, val);
        analogWrite(bkdPin, 0);
    } else {
        analogWrite(fwdPin, 0);
        analogWrite(bkdPin, val);
    }
}

void leftEffCallback(const std_msgs::Float32& msg) {
    setMotor(MOTOR_LF, MOTOR_LB, static_cast<short>(round(msg.data)));
}

void rightEffCallback(const std_msgs::Float32& msg) {
    setMotor(MOTOR_RF, MOTOR_RB, static_cast<short>(round(msg.data)));
}
#endif

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

float angularPosition(const long& position) {
    return (position / static_cast<float>(ENC_PULSES_PER_REVOLUTION)) * (PI * 2);
}

void setup()
{
#ifdef DEBUG
    Serial.begin(115200);
#endif

    pinMode(MOTOR_SLEEP, OUTPUT);
    pinMode(MOTOR_FAULT, INPUT);
    pinMode(MOTOR_LB, OUTPUT);
    pinMode(MOTOR_LF, OUTPUT);
    pinMode(MOTOR_RB, OUTPUT);
    pinMode(MOTOR_RF, OUTPUT);

    // motors off
    analogWrite(MOTOR_LB, 0);
    analogWrite(MOTOR_LF, 0);
    analogWrite(MOTOR_RB, 0);
    analogWrite(MOTOR_RF, 0);

    // disable low-power sleep mode
    digitalWrite(MOTOR_SLEEP, HIGH);

#ifndef DEBUG
    nh.initNode();

    nh.advertise(pubLeftPos);
    nh.advertise(pubRightPos);
    nh.advertise(pubLeftVel);
    nh.advertise(pubRightVel);

    nh.subscribe(subLeftEff);
    nh.subscribe(subRightEff);
#endif

    delay(1000);
}

void loop()
{
    long newPositionLeft, newPositionRight;
    float angularVelocityLeft, angularVelocityRight;
    newPositionLeft = -encLeft.getEncoderCount();
    newPositionRight = encRight.getEncoderCount();
    unsigned long encNow = millis();
    angularVelocityLeft = angularVelocity(positionLeft, newPositionLeft, encLastTime, encNow);
    angularVelocityRight = angularVelocity(positionRight, newPositionRight, encLastTime, encNow);

#ifdef DEBUG
    sprintf(buf,
            "pos_l: %6d, e_l: %6d, theta_l (deg): %4d, omega_l (deg/s): %8.2f, n_l (rpm): %8.2f, "
                "pos_r: %6d, e_r: %6d, theta_r (deg): %4d, omega_r (deg/s): %8.2f, n_r (rpm): %8.2f\n",
            newPositionLeft,
            encLeft.getEncoderErrorCount(),
            static_cast<int>(angularPosition(newPositionLeft) * 180 / PI) % 360,
            angularVelocityLeft * 180 / PI,
            angularVelocityLeft / (2 * PI) * 60,
            newPositionRight,
            encRight.getEncoderErrorCount(),
            static_cast<int>(angularPosition(newPositionRight) * 180 / PI) % 360,
            angularVelocityRight * 180 / PI,
            angularVelocityRight / (2 * PI) * 60);
    Serial.print(buf);
#endif

    encLastTime = encNow;
    positionLeft = newPositionLeft;
    positionRight = newPositionRight;

#ifndef DEBUG
    // TODO rpm, deg/s, rad/s?
    encMsg.data = 0;
    pubLeftPos.publish(&encMsg);
    encMsg.data = 0;
    pubRightPos.publish(&encMsg);

    encMsg.data = 0;
    pubLeftVel.publish(&encMsg);
    encMsg.data = 0;
    pubRightVel.publish(&encMsg);

    nh.spinOnce();
#endif

    delay(1000 / 100);   // TODO to maintain synchronization, keep this node and base_node spinning at same frequency
}
