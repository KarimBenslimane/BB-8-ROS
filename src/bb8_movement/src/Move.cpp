//
// Created by renzoscholman on 5-11-2016.
//

#include <bb8_movement/Move.h>

Move::Move(ros::NodeHandle* n)
{
    /**
     * Advertise topics to roshandle and
     * initialize the ros publishers.
     */
    ball_dir_sub = n->subscribe("ball_speed", 1, &Move::updateSpeed, this);
    ball_speed_sub = n->subscribe("ball_dir", 1, &Move::updateDirection, this);
    head_Xspeed_sub = n->subscribe("head_Xspeed", 1, &Move::updateHeadXSpeed, this);
    head_Yspeed_sub = n->subscribe("head_Yspeed", 1, &Move::updateHeadYSpeed, this);
    head_rotate_sub = n->subscribe("head_rotate", 1, &Move::updateHeadRotate, this);
    direction = 0;
    speed = 0;
    head_Xspeed = 0;
    head_Yspeed = 0;
    head_rotate = 0;
    motors.reserve(MAX_MOTORS);
    servos.reserve(MAX_SERVOS);
    communication = new I2C();
    for (int i = 0; i < MAX_MOTORS; ++i)
    {
        motors[i] = new Motor(START_ADDRESS + i);
    }
    for (int i = 0; i < MAX_SERVOS; ++i)
    {
        servos[i] = new Servo(SERVO_ARDUINO_ADDRESS, START_PIN + i);
    }
}

Move::~Move()
{
    for (int i = 0; i < MAX_MOTORS; ++i)
    {
        delete motors[i];
    }
    for (int i = 0; i < MAX_SERVOS; ++i)
    {
        delete servos[i];
    }
}

void Move::updateSpeed(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Retrieved ball_speed: [%d]", msg->data);
    speed = msg->data;
}

void Move::updateDirection(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Retrieved ball_dir: [%d]", msg->data);
    direction = msg->data;
}

void Move::updateHeadXSpeed(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Retrieved head_Xspeed: [%d]", msg->data);
    head_Xspeed = msg->data;
}

void Move::updateHeadYSpeed(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Retrieved head_Yspeed: [%d]", msg->data);
    head_Yspeed = msg->data;
}

void Move::updateHeadRotate(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Retrieved head_rotate: [%d]", msg->data);
    head_rotate = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bb8_movement_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(50);
    Move* move = new Move(&n);
    ROS_INFO("%s","Starting movement loop");
    while (ros::ok())
    {
        ros::spinOnce();
        move->calculate();
        move->send();
        loop_rate.sleep();

    }
    move->stop();
    return 0;
}

void Move::calculate(){
    //motors
    motors[0]->setSpeed(sin(((double) direction * M_PI) / 180.0) * speed);
    motors[1]->setSpeed(cos(((double) direction * M_PI) / 180.0) * speed);
    motors[2]->setSpeed(-sin(((double) direction * M_PI) / 180.0) * speed);
    motors[3]->setSpeed(-cos(((double) direction * M_PI) / 180.0) * speed);

    servos[1]->setSpeed(head_Xspeed, head_Yspeed);
    servos[0]->setSpeed(head_Xspeed, head_Yspeed);
    servos[2]->setSpeed(90, 90);
}

void Move::send() {
    for (int i = 0; i < MAX_MOTORS; ++i)
    {
        communication->setAddress(motors[i]->getAddress());
        if (communication->isReady())
        {
            communication->send(motors[i]->getCommand());
        }
    }
    for (int i = 0; i < MAX_SERVOS; ++i)
    {
        communication->setAddress(servos[i]->getAddress());
        if (communication->isReady())
        {
            communication->send(servos[i]->getCommand());
        }
    }

}

void Move::stop(){
    speed = 0;
    calculate();
    send();
}
