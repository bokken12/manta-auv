#ifndef PITCH_HOLD_ACTION_SERVER_ROS_H
#define PITCH_HOLD_ACTION_SERVER_ROS_H

#include "pitch_hold_action_server/DHpid.h"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "pitch_hold_action_server/PitchHoldAction.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <iostream>

class PitchHoldAction
{
    protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<pitch_hold_action_server::PitchHoldAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    pitch_hold_action_server::PitchHoldFeedback feedback_;
    pitch_hold_action_server::PitchHoldResult result_;

    private:
    ros::Publisher pub_;
    ros::Subscriber sub_;

    double goal_pitch = 1.0;
    std::unique_ptr<DHpid> height;

    public:
    PitchHoldAction(std::string name);
    ~PitchHoldAction();

    void stateEstimateCallback(const nav_msgs::Odometry &odometry_msgs);
    void executeCB(const pitch_hold_action_server::PitchHoldGoalConstPtr &goal);
};

#endif