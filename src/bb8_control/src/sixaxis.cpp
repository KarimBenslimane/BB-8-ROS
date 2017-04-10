//
// Created by karimbenslimane on 17-12-2016.
//

#include <bb8_control/sixaxis.h>

Sixaxis::Sixaxis(ros::NodeHandle* n)
{
    /**
     * Advertise topics to roshandle and
     * initialize the ros publishers.
     */
    ball_speed_pub = n->advertise<std_msgs::Int32>("ball_speed", 1);
    ball_dir_pub = n->advertise<std_msgs::Int32>("ball_dir", 1);
    ball_rotate_pub = n->advertise<std_msgs::Int32>("ball_rotate", 1);
    head_Xspeed_pub = n->advertise<std_msgs::Int32>("head_Xspeed", 1);
    head_Yspeed_pub = n->advertise<std_msgs::Int32>("head_Yspeed", 1);
    head_rotate_pub = n->advertise<std_msgs::Int32>("head_rotate", 1);
    openJoystick();
    left_joystick_x = 0;
    left_joystick_y = 0;
    right_joystick_x = 0;
    right_joystick_y = 0;
    button_l1 = 0;
    button_r1 = 0;
    button_l2 = 0;
    button_r2 = 0;
}

Sixaxis::~Sixaxis()
{
    close(joyfd);
}

int main(int argc, char **argv)
{
    /**
    * Initialize rosnode with name 'controlnode'.
    */
    ros::init(argc, argv, "bb8_control_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(50);
    Sixaxis* sixaxis = new Sixaxis(&n);
    ROS_INFO("%s", "Starting control loop");
    while (ros::ok())
    {
        sixaxis->readInput();
        /**
         * Call all callbacks waiting to be called at this point in time.
         */
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void Sixaxis::openJoystick()
{
    joyfd = open("/dev/input/js0", O_RDONLY);
    if(joyfd < 0){
        printf("No joystick found");
    }
}

void Sixaxis::readInput()
{
    struct js_event ne;
    if(read(joyfd, &ne, sizeof(struct js_event)) == sizeof(struct js_event)){
       // ROS_INFO("%s", "JOYSTICK EVENT FIRED");
        e = ne;
        if(e.type == JS_EVENT_BUTTON){
            switch(e.number){
                case BUTTON_L1:
                    //turn left around center
                    button_l1 = e.value;
                    updateBallRot();
                    break;
                case BUTTON_R1:
                    button_r1 = e.value;
                    // turn right around center
                    updateBallRot();
                    break;
                case BUTTON_L2:
                    //turn left around center
                    button_l2 = e.value;
                    updateHeadRot();
                    break;
                case BUTTON_R2:
                    button_r2 = e.value;
                    // turn right around center
                    updateHeadRot();
                    break;
                default:
                    break;
            }
        }
        if(e.type == JS_EVENT_AXIS){
            switch(e.number){
                case LEFT_JOYSTICK_X:
                    // set speed for left/right
                    left_joystick_x = e.value;
                    updateBallSpDr();
                    break;
                case LEFT_JOYSTICK_Y:
                    // set speed for forward/backward
                    left_joystick_y = e.value;
                    updateBallSpDr();
                    break;
                case RIGHT_JOYSTICK_X:
                    //ROS_INFO("%s", "RIGHT JOYSTICK X ");
                    // turn head left/right
                    right_joystick_x = e.value;
                    updateHeadSp();
                    break;
                case RIGHT_JOYSTICK_Y:
                   // ROS_INFO("%s", "RIGHT JOYSTICK Y ");
                    // turn head forward/backward
                    right_joystick_y = e.value;
                    updateHeadSp();
                    break;
                default:
                    break;
            }
        }
    }
}

void Sixaxis::updateBallSpDr()
{
    /**
     * Learn to goniometrics if you dont get it (tip: pythagoras)
     */
    int speed = sqrt(pow(left_joystick_x, 2) + pow(left_joystick_y, 2)) * LEFT_JOYSTICK_NORMALIZE;
    if(speed > 400) speed = 400;
    int dir = 360 - ((atan2(left_joystick_x, left_joystick_y) * 180) / M_PI + 180);

    std_msgs::Int32 dir_msg;
    std_msgs::Int32 spd_msg;

    dir_msg.data = dir;
    spd_msg.data = speed;

    ROS_INFO("ball_dir_msg: [%d]", dir_msg.data);
    ROS_INFO("ball_speed_msg: [%d]", spd_msg.data);

    ball_dir_pub.publish(dir_msg);
    ball_speed_pub.publish(spd_msg);
}

void Sixaxis::updateBallRot()
{
    int rot = (button_r1 - button_l1) * BUTTON_NORMALIZE;

    std_msgs::Int32 rot_msg;

    rot_msg.data = rot;

    ROS_INFO("ball_rotate_msg: [%d]", rot_msg.data);

    ball_rotate_pub.publish(rot_msg);
}

void Sixaxis::updateHeadSp()
{
    int xSpeed = right_joystick_x * RIGHT_JOYSTICK_NORMALIZE;
    int ySpeed = right_joystick_y * RIGHT_JOYSTICK_NORMALIZE;

    if(xSpeed > 50) xSpeed = 50;
    if(ySpeed > 50) ySpeed = 50;

    std_msgs::Int32 xSpeed_msg;
    std_msgs::Int32 ySpeed_msg;

    xSpeed_msg.data = xSpeed;
    ySpeed_msg.data = ySpeed;

    ROS_INFO("head_Xspeed_msg: [%d]", xSpeed_msg.data);
    ROS_INFO("head_Yspeed_msg: [%d]", ySpeed_msg.data);

    head_Xspeed_pub.publish(xSpeed_msg);
    head_Yspeed_pub.publish(ySpeed_msg);
}

void Sixaxis::updateHeadRot()
{
    int rot = (button_r2 - button_l2) * BUTTON_NORMALIZE;
if(rot > 45) rot = 45;

    std_msgs::Int32 rot_msg;

    rot_msg.data = rot;

    ROS_INFO("head_rotate_msg: [%d]", rot_msg.data);

    head_rotate_pub.publish(rot_msg);
}
