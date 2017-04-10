//
// Created by renzoscholman on 5-11-2016.
//

#ifndef RASPBERRYPI_MOVE_H
#define RASPBERRYPI_MOVE_H
#define _USE_MATH_DEFINES

#include <bb8_movement/Motor.h>
#include <bb8_movement/Servo.h>
#include <bb8_communications/i2c.h>
#include <math.h>
#include <vector>
#include "ros/ros.h"
#include <std_msgs/Int32.h>

const int MAX_MOTORS            = 4;
const int MAX_SERVOS            = 3;
const int START_ADDRESS         = 8;
const int SERVO_ARDUINO_ADDRESS = 12;

class Move
{
    public:
        Move(ros::NodeHandle* n);
        ~Move();
        void updateSpeed(const std_msgs::Int32::ConstPtr& msg);
        void updateDirection(const std_msgs::Int32::ConstPtr& msg);
        void updateHeadXSpeed(const std_msgs::Int32::ConstPtr& msg);
        void updateHeadYSpeed(const std_msgs::Int32::ConstPtr& msg);
        void updateHeadRotate(const std_msgs::Int32::ConstPtr& msg);
        void calculate();
        void send();
        void stop();
    private:
        I2C* communication;
        int direction;
        int speed;
        int head_Xspeed;
        int head_Yspeed;
        int head_rotate;
        std::vector<Motor *> motors;
        std::vector<Servo *> servos;
        ros::Subscriber ball_dir_sub;
        ros::Subscriber ball_speed_sub;
        ros::Subscriber head_Xspeed_sub;
        ros::Subscriber head_Yspeed_sub;
        ros::Subscriber head_rotate_sub;
};

#endif //RASPBERRYPI_MOVE_H
