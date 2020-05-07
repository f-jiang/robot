#include <ros.h>
#include <std_msgs/Float64.h>

#include <stdio.h>

#include <Arduino.h>

// NOTE: currently this lib and the encoder code here rely on an unreleased fix for the arduino mbed core:
// https://github.com/arduino/ArduinoCore-nRF528x-mbedos/pull/69
#include <QuadratureEncoder.h>  // fork: f-jiang/QuadratureEncoder

// #define DEBUG

#define MOTOR_LB    8   // left backwards
#define MOTOR_LF    9   // left forward
#define MOTOR_RB    6   // right backwards
#define MOTOR_RF    7   // right forward
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

int loopDelay = 10;

char buf[256];

#ifndef DEBUG
// TODO integrate this into catkin so that robot_constants can be used here

ros::NodeHandle nh;

std_msgs::Float64 leftPosMsg;
ros::Publisher pubLeftPos("/left_wheel/pos", &leftPosMsg);

std_msgs::Float64 rightPosMsg;
ros::Publisher pubRightPos("/right_wheel/pos", &rightPosMsg);

std_msgs::Float64 leftVelMsg;
ros::Publisher pubLeftVel("/left_wheel/vel", &leftVelMsg);

std_msgs::Float64 rightVelMsg;
ros::Publisher pubRightVel("/right_wheel/vel", &rightVelMsg);

void leftEffCallback(const std_msgs::Float64& msg);
void rightEffCallback(const std_msgs::Float64& msg);
ros::Subscriber<std_msgs::Float64> subLeftEff("/left_wheel/control_effort", &leftEffCallback);
ros::Subscriber<std_msgs::Float64> subRightEff("/right_wheel/control_effort", &rightEffCallback);

int controlFrequency = 100;

void setMotor(byte fwdPin, byte bkdPin, short val) {
    short absVal = abs(val);
    if (val > 0) {
        analogWrite(fwdPin, absVal);
        analogWrite(bkdPin, 0);
    } else {
        analogWrite(fwdPin, 0);
        analogWrite(bkdPin, absVal);
    }
}

void leftEffCallback(const std_msgs::Float64& msg) {
    setMotor(MOTOR_LF, MOTOR_LB, round(msg.data));
}

void rightEffCallback(const std_msgs::Float64& msg) {
    setMotor(MOTOR_RF, MOTOR_RB, round(msg.data));
}
#endif

// rad/s
float angularVelocity(const long& oldPosition,
                      const long& newPosition,
                      const unsigned long& oldTime,
                      const unsigned long& newTime) {
    return (newPosition - oldPosition)
           / ENC_PULSES_PER_REVOLUTION
           * (PI * 2)
           / ((newTime - oldTime) / 1000.0f);
}

float angularPosition(const long& position) {
    long wrappedPosition = position
                           - static_cast<long>(position / ENC_PULSES_PER_REVOLUTION)
                           * ENC_PULSES_PER_REVOLUTION;

    if (position < 0) {
        wrappedPosition += ENC_PULSES_PER_REVOLUTION;
    }

    return wrappedPosition
           / ENC_PULSES_PER_REVOLUTION
           * (PI * 2);
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

    while (!nh.connected()) {
        nh.spinOnce();
    }

    // TODO robot_constants::params::kControlFrequency
    nh.getParam("/control_frequency", &controlFrequency);
    sprintf(buf, "fw_node running at %d hz", controlFrequency);
    nh.loginfo(buf);
    loopDelay = round(1000 / static_cast<double>(controlFrequency));

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
    float angularPositionLeft, angularPositionRight;
    float angularVelocityLeft, angularVelocityRight;
    newPositionLeft = -encLeft.getEncoderCount();
    newPositionRight = encRight.getEncoderCount();
    unsigned long encNow = millis();
    angularVelocityLeft = angularVelocity(positionLeft, newPositionLeft, encLastTime, encNow);
    angularVelocityRight = angularVelocity(positionRight, newPositionRight, encLastTime, encNow);
    angularPositionLeft = angularPosition(newPositionLeft);
    angularPositionRight = angularPosition(newPositionRight);

#ifdef DEBUG
    sprintf(buf,
            "pos_l: %6d, e_l: %6d, theta_l (deg): %4d, omega_l (deg/s): %8.2f, n_l (rpm): %8.2f, "
                "pos_r: %6d, e_r: %6d, theta_r (deg): %4d, omega_r (deg/s): %8.2f, n_r (rpm): %8.2f\n",
            newPositionLeft,
            encLeft.getEncoderErrorCount(),
            angularPositionLeft * 180 / PI,
            angularVelocityLeft * 180 / PI,
            angularVelocityLeft / (2 * PI) * 60,
            newPositionRight,
            encRight.getEncoderErrorCount(),
            angularPositionRight * 180 / PI,
            angularVelocityRight * 180 / PI,
            angularVelocityRight / (2 * PI) * 60);
    Serial.print(buf);
#endif

    encLastTime = encNow;
    positionLeft = newPositionLeft;
    positionRight = newPositionRight;

#ifndef DEBUG
    leftPosMsg.data = angularPositionLeft;
    pubLeftPos.publish(&leftPosMsg);
    rightPosMsg.data = angularPositionRight;
    pubRightPos.publish(&rightPosMsg);

    leftVelMsg.data = angularVelocityLeft;
    pubLeftVel.publish(&leftVelMsg);
    rightVelMsg.data = angularVelocityRight;
    pubRightVel.publish(&rightVelMsg);

    nh.spinOnce();
#endif

    delay(loopDelay);
}

