//
// Created by renzoscholman on 5-11-2016.
//

#ifndef RASPBERRYPI_MOTOR_H
#define RASPBERRYPI_MOTOR_H

#include <stdio.h>
#include "ros/ros.h"

class Motor{
    public:
        Motor();
        Motor(int newAddress);
        ~Motor();
        unsigned short int getCommand();
        void setSpeed(int newSpeed);
        int getSpeed();
        void setSteps(int newSteps);
        int getSteps();
        void setAddress(int newAddress);
        int getAddress();
    private:
        int steps;
        int speed;
        int address;
};

#endif //RASPBERRYPI_MOTOR_H
