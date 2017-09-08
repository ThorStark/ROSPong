/*****************************************************************************
* @file pong_main_.h
*
* Simulates a 2D point ball bouncing around in a inclosed area
*
* @author  Thor Stark Stenvang
*          thor.stark.stenvang@gmail.com
* @version 0.00
* @date    2017-09-07 YYYY-MM-DD
******************************************************************************/

#ifndef PONG_MAIN_H
#define PONG_MAIN_H

#include<random>
#include<thread>
#include<mutex>
#include<atomic>
#include<ros/ros.h>
// Services
#include <pong_ball/Reset.h>
#include <pong_ball/SetPose.h>

namespace pong {

struct Area {
    int width;
    int height;
};

struct Pose{
    float x;
    float y;
    float vx;
    float vy;
};

class PongBall
{
public:
    /**
     * @brief PongMain default ctor
     */
    PongBall();
    /**
     * @brief PongMain ctor
     * @param globalHandle. A global ros node handle
     * @param privateHandle. A private ros node handle
     * @param ns is the namespace for services topics etc.
     */
    PongBall(ros::NodeHandle globalHandle, ros::NodeHandle privateHandle);
    /**
     * @brief PongMain dtor
     */
    ~PongBall();

    /**
     * @brief spin updates and publishes messages
     * @return should always return true
     */
    bool spin();

    /**
     * @brief startUpdateThread start update thread which updates ball position.
     */
    void startUpdateThread();

    /**
     * @brief stopUpdateThread stop update thread for ball position
     */
    void stopUpdateThread();


    /***************************************************************************
     * Service Callback functions
     ***************************************************************************/
    bool resetCallback(pong_ball::Reset::Request& req, pong_ball::Reset::Response& res);
    bool setPoseCallback(pong_ball::SetPose::Request& req, pong_ball::SetPose::Response& res);

private:
    void updatePose();
    std::thread updatePoseThread_;
    std::atomic<bool> runUpdateThread_;
    std::mutex poseMutex_;

private:
    ros::NodeHandle nh_;       // Global
    ros::NodeHandle pnh_;      // Private
    ros::Publisher pub_pose_;
    ros::Publisher pub_viz_;
    ros::ServiceServer srv_reset_;
    ros::ServiceServer srv_setpose_;

    Area pong_area_;
    Pose pose_;

    /**
     * Note the random generator is not fully random. The seed used is only 32 bit while
     * the Mersenne Twister is 19937 bit. The first random value can only have a value of
     * one of 2^32 states and not 2^19937. No reseeding is used so only the initial value
     * is a problem.
     */
    std::mt19937 rand_mt_;

};
}

#endif  // PONG_MAIN_H
