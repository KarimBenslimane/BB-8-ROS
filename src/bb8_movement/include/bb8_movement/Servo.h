//
// Created by karim on 6-4-17.
//

#ifndef RASPBERRYPI_SERVO_H
#define RASPBERRYPI_SERVO_H

#include <stdio.h>
#include "ros/ros.h"

const int START_PIN = 9;

class Servo{
    public:
        Servo();
        Servo(int newAddress, int pinAddress);
        ~Servo();
        unsigned short int getCommand();
        void setSpeed(int X, int Y);
        int getSpeed();
        void setAddress(int newAddress);
        int getAddress();
        void setPin(int pinAddress);
        int getPin();
    private:
        int speed;
        int address;
        int pin;
        int motorX;
        int motorY;
};

#endif //RASPBERRYPI_SERVOARDUINO_H
