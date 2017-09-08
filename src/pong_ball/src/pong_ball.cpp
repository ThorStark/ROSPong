/*****************************************************************************
* @file pong_main.cpp
*
* @author  Thor Stark Stenvang
*          thor.stark.stenvang@gmail.com
* @version 0.00
* @date    2017-09-07 YYYY-MM-DD
******************************************************************************/

#include<cmath>
#include <chrono>

// ROS
#include <pong_ball/pong_ball.h>

// Topics
#include <pong_ball/Pose.h>
#include <visualization_msgs/Marker.h>

pong::PongBall::PongBall():PongBall(ros::NodeHandle(), ros::NodeHandle("~"))
{
}

pong::PongBall::PongBall(ros::NodeHandle globalHandle, ros::NodeHandle privateHandle):
    nh_(globalHandle),
    pnh_(privateHandle),
    rand_mt_(std::random_device{}())
{
    // Get parameters from Global namespace
    nh_.param<int>("pong_area_width",  pong_area_.width,  10);
    nh_.param<int>("pong_area_height", pong_area_.height, 10);

    // Get parameters from private namespace
    std::lock_guard<std::mutex> guard(poseMutex_);
    {
        pnh_.param<float>("start_pos_x", pose_.x, 0.0);
        pnh_.param<float>("start_pos_y", pose_.y, 0.0);
        pnh_.param<float>("start_vel_x", pose_.vx, 0.0);
        pnh_.param<float>("start_vel_y", pose_.vy, 0.0);
    }

    // Publishers:
    pub_pose_ = pnh_.advertise<pong_ball::Pose>("pongballPose",1);
    pub_viz_ = pnh_.advertise<visualization_msgs::Marker>("ball_visualization",1);

    // Services:
    srv_reset_ = pnh_.advertiseService("reset", &PongBall::resetCallback, this);
    srv_setpose_ = pnh_.advertiseService("setPose", &PongBall::setPoseCallback, this);
}

pong::PongBall::~PongBall()
{
    runUpdateThread_ = false;
    if(updatePoseThread_.joinable())
        updatePoseThread_.join();
}

bool pong::PongBall::spin()
{
    pong_ball::Pose pose_msg;

    visualization_msgs::Marker point;
    point.header.frame_id = "/ball_frame";
    point.header.stamp = ros::Time::now();
    point.ns = "ball";
    point.id = 0;
    point.action = visualization_msgs::Marker::ADD;
    point.pose.orientation.w = 1.0;
    point.type = visualization_msgs::Marker::POINTS;
    point.scale.x = point.scale.y  = 0.1;
    point.color.g = 1.0f; point.color.a = 1.0f;
    geometry_msgs::Point p;
    p.z = 1.0;
    {
        std::lock_guard<std::mutex> guard(poseMutex_);
        p.x = pose_msg.x = pose_.x;
        p.y = pose_msg.y = pose_.y;
        pose_msg.vel_x = pose_.vx;
        pose_msg.vel_y = pose_.vy;
    }
    point.points.push_back(p);

    pub_pose_.publish(pose_msg);
    pub_viz_.publish(point);


    return true;
}

/***************************************************************************
 * Service callbacks
 ***************************************************************************/

bool pong::PongBall::resetCallback(pong_ball::Reset::Request &req, pong_ball::Reset::Response &res)
{
    std::lock_guard<std::mutex> guard(poseMutex_);
    pose_.x = 0.0;
    pose_.y = 0.0;
    if(req.random_vel_init){
        //Generate random velocity vector with size 1
        std::uniform_real_distribution<float> dist(0.0,1.0);
        pose_.vx = dist(rand_mt_);
        pose_.vy = std::sqrt(1-pose_.vx*pose_.vx);
    }else{
        pose_.vx = 0.0;
        pose_.vy = 0.0;
    }
    return true;
}

bool pong::PongBall::setPoseCallback(pong_ball::SetPose::Request &req, pong_ball::SetPose::Response& res)
{
    std::lock_guard<std::mutex> guard(poseMutex_);
    pose_.x = req.pos.x;
    pose_.y = req.pos.y;
    pose_.vx = req.pos.vel_x;
    pose_.vy = req.pos.vel_y;
    return true;
}

/***************************************************************************
 * Threads
 ***************************************************************************/

void pong::PongBall::updatePose()
{
    int delta = 20;
    while(runUpdateThread_){
        std::this_thread::sleep_for(std::chrono::milliseconds(delta));
        std::lock_guard<std::mutex> guard(poseMutex_);
        Pose new_pose = pose_;
        new_pose.x += new_pose.vx * (float)delta/1000.0;
        new_pose.y += new_pose.vy * (float)delta/1000.0;

        // Reflect direction if outside area
        if(std::abs(new_pose.x) > pong_area_.width/2)
            new_pose.vx = -new_pose.vx;
        if(std::abs(new_pose.y) > pong_area_.height/2)
            new_pose.vy = -new_pose.vy;
        pose_ = new_pose;
    }
}

void pong::PongBall::startUpdateThread()
{
    runUpdateThread_ = true;
    updatePoseThread_ = std::thread(&PongBall::updatePose, this);
}

void pong::PongBall::stopUpdateThread()
{
    runUpdateThread_ = false;
    updatePoseThread_.join();
}
