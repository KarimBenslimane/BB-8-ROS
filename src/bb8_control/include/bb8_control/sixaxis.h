//
// Created by karimbenslimane on 17-12-2016.
//

#ifndef JETSON_SIXAXIS_H
#define JETSON_SIXAXIS_H

// Error handling
#include <errno.h>
#include <linux/joystick.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <sstream>

#define LEFT_JOYSTICK_NORMALIZE (400.0 / 32768.0)
#define BUTTON_NORMALIZE (45.0 / 255.0)
#define RIGHT_JOYSTICK_NORMALIZE (50.0 / 32768.0)

// axis constants
enum joystick
{
    LEFT_JOYSTICK_X,
    LEFT_JOYSTICK_Y,
    RIGHT_JOYSTICK_X,
    RIGHT_JOYSTICK_Y
};

// buttons constants
enum button
{
    BUTTON_L1 = 10,
    BUTTON_R1 = 11,
    BUTTON_L2 = 12,
    BUTTON_R2 = 13
};

class Sixaxis
{
    public:
        Sixaxis(ros::NodeHandle* n);
        ~Sixaxis();
        void openJoystick();
        void readInput();
    private:
        void updateBallSpDr();
        void updateBallRot();
        void updateHeadSp();
        void updateHeadRot();
        int left_joystick_x, left_joystick_y, right_joystick_x, right_joystick_y;
        int button_l1, button_r1, button_l2, button_r2;
        struct js_event e;
        int joyfd;
        ros::Publisher ball_speed_pub;
        ros::Publisher ball_dir_pub;
        ros::Publisher ball_rotate_pub;
        ros::Publisher head_Xspeed_pub;
        ros::Publisher head_Yspeed_pub;
        ros::Publisher head_rotate_pub;
};

#endif //JETSON_SIXAXIS_H

