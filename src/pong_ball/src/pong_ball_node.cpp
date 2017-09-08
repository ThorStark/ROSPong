/*****************************************************************************
* @file pong_main_node.cpp
*
* @author  Thor Stark Stenvang
*          thor.stark.stenvang@gmail.com
* @version 0.00
* @date    2017-09-07 YYYY-MM-DD
******************************************************************************/

//ROS
#include<ros/ros.h>
#include<pong_ball/pong_ball.h>

int main(int argc, char** argv)
{
    //Init ROS Node
    ros::init(argc,argv,"pong_ball");
    ros::NodeHandle nh;
    ros::NodeHandle pNh("~");
    ros::Rate r (30);

    std::unique_ptr<pong::PongBall> pong(new pong::PongBall(nh, pNh));
    pong->startUpdateThread();

    while (!ros::isShuttingDown())
    {
        pong->spin();
        ros::spinOnce();
        r.sleep(); //*snoring*
    }
    pong->stopUpdateThread();

    ros::shutdown();
    return 0;
}
