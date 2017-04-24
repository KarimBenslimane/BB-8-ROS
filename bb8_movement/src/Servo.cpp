//
// Created by karim on 6-4-17.
//
#include <bb8_movement/Servo.h>

Servo::Servo()
{
    speed = 0;
    address = -1;
}

Servo::Servo(int newAddress, int pinAddress)
{
    speed = 0;
    address = newAddress;
    pin = pinAddress;
    if(pin == 9)
    {
        motorX = 50;
        motorY = 50;
    }else if(pin == 10){
        motorX = -50;
        motorY = -50;
    }else{
        motorX = 0;
        motorY = 0;
    }
}

Servo::~Servo()
{
    // Silence is golden
}

unsigned short int Servo::getCommand()
{
    speed = speed < 0 ? 0 : speed;
    return ((pin & 0xf) << 12) + (speed & 0xfff);
}

int Servo::getAddress()
{
    return address;
}

int Servo::getSpeed()
{
    return speed;
}

void Servo::setAddress(int newAddress)
{
    address = newAddress;
}

void Servo::setSpeed(int X, int Y)
{
    int deltaX = X - motorX;
    int deltaY = Y - motorY;
    int angleRad = 0;
    if(deltaX > 0)
    {
        angleRad = atan2(deltaY, deltaX);
    }else {
        angleRad = M_PI + atan2(deltaY, deltaX);
    }
    int angleDeg = angleRad * (180/M_PI); // Let us convert the angle from radians (angleRad) to degrees (angleDeg).

    if(pin == 10)
    {
        angleDeg += 90;
    }else if(pin == 9)
    {
        angleDeg -= 90;
    }

    if (angleDeg > 180) {
        angleDeg = 180; // Now, let us keep the angles between -180 to 180 degrees, for the sake of clarity.
    }else if(angleDeg < 0)
    {
        angleDeg = angleDeg * -1;
    }

    speed = angleDeg;
}

void Servo::setPin(int pinAddress)
{
    pin = pinAddress;
}

int Servo::getPin()
{
    return pin;
}


