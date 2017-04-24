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
    cmd_vel = n->advertise<geometry_msgs::Twist>("bb_8/cmd_vel", 1);
    cmd_vel_head = n->advertise<geometry_msgs::Twist>("bb_8/cmd_vel_head", 1);
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
                    updateBall();
                    break;
                case BUTTON_R1:
                    button_r1 = e.value;
                    // turn right around center
                    updateBall();
                    break;
                case BUTTON_L2:
                    //turn left around center
                    button_l2 = e.value;
                    updateHead();
                    break;
                case BUTTON_R2:
                    button_r2 = e.value;
                    // turn right around center
                    updateHead();
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
                    updateBall();
                    break;
                case LEFT_JOYSTICK_Y:
                    // set speed for forward/backward
                    left_joystick_y = e.value;
                    updateBall();
                    break;
                case RIGHT_JOYSTICK_X:
                    //ROS_INFO("%s", "RIGHT JOYSTICK X ");
                    // turn head left/right
                    right_joystick_x = e.value;
                    updateHead();
                    break;
                case RIGHT_JOYSTICK_Y:
                   // ROS_INFO("%s", "RIGHT JOYSTICK Y ");
                    // turn head forward/backward
                    right_joystick_y = e.value;
                    updateHead();
                    break;
                default:
                    break;
            }
        }
    }
}

void Sixaxis::updateBall()
{
    geometry_msgs::Twist ball;
    ball.linear.x = left_joystick_x;
    ball.linear.y = left_joystick_y;
    ball.angular.x = (button_r1 - button_l1) * BUTTON_NORMALIZE;
    cmd_vel.publish(ball);
}

void Sixaxis::updateHead()
{
    geometry_msgs::Twist head;
    head.linear.x = std::min(50, static_cast<int>(right_joystick_x * RIGHT_JOYSTICK_NORMALIZE));
    head.linear.y = std::min(50, static_cast<int>(right_joystick_y * RIGHT_JOYSTICK_NORMALIZE));
    head.angular.x = std::min(45, static_cast<int>((button_r2 - button_l2) * BUTTON_NORMALIZE));
    cmd_vel_head.publish(head);
}
