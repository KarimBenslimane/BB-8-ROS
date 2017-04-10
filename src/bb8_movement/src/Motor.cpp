//
// Created by renzoscholman on 5-11-2016.
//

#include <bb8_movement/Motor.h>

Motor::Motor()
{
  speed = 0;
  address = -1;
}

Motor::Motor(int newAddress)
{
  speed = 0;
  address = newAddress;
}

Motor::~Motor()
{
  // Silence is golden
}

unsigned short int Motor::getCommand()
{
    unsigned short int newDirection = speed > 0 ? 0 : 1;
    unsigned short int newSpeed = speed < 0 ? speed * -1 : speed;
    if(address == 11)
    {
//        ROS_INFO("Sending speed: [%d]", newSpeed);
//        ROS_INFO("Sending direction: [%d]", newDirection);
    }
    return ((newDirection & 0xf) << 12) + (newSpeed & 0xfff);
}

int Motor::getAddress()
{
  return address;
}

int Motor::getSpeed()
{
  return speed;
}

void Motor::setAddress(int newAddress)
{
  address = newAddress;
}

void Motor::setSpeed(int newSpeed)
{
  speed = newSpeed;
}
