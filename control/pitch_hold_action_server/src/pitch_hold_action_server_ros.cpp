#include <pitch_hold_action_server/DHpid.h>
#include <pitch_hold_action_server/pitch_hold_action_server_ros.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "pitch_hold_action_server/PitchHoldAction.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <iostream>
#include <math.h>

PitchHoldAction::PitchHoldAction(std::string name) :
    as_(nh_, name, boost::bind(&PitchHoldAction::executeCB, this, _1), false),
    action_name_(name)
    {
        as_.start();

        sub_ = nh_.subscribe("/manta/pose_gt", 1, &PitchHoldAction::stateEstimateCallback, this);

        pub_ = nh_.advertise<geometry_msgs::Wrench>("roll_input", 1);
        double dt = 0.1;
        double max = 40.0;
        double min = -40.0;
        double K_p = 100.5;
        double K_d = 0.13;
        double K_i = 0.00;

        height.reset(new DHpid(dt, max, min, K_p, K_d, K_i));
    }


PitchHoldAction::~PitchHoldAction()
{
}

void PitchHoldAction::executeCB(const pitch_hold_action_server::PitchHoldGoalConstPtr &goal)
{
    geometry_msgs::Wrench dh_command;
    ros::Rate rate(10);
    bool success = true;

    this->goal_pitch = goal->pitch;
    ROS_INFO("%s: Executing pitch hold with goal pitch %f m", action_name_.c_str(), goal->pitch);

    while(ros::ok()){
        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            feedback_.ready = false;
            as_.publishFeedback(feedback_);
            as_.setPreempted();
            success = false;
            dh_command.torque.y = 0;
            pub_.publish(dh_command);
            break;
        }
        dh_command.torque.y = this->height->calculate();

        as_.publishFeedback(feedback_);
        //std::cout << "Heave command" << dh_command.force.z << std::endl;
        pub_.publish(dh_command);
        ros::spinOnce();
        rate.sleep();
    }

    if(success)
    {
        result_.success = success;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
}

void PitchHoldAction::stateEstimateCallback(const nav_msgs::Odometry &odometry_msgs){
    feedback_.current_pitch = odometry_msgs.pose.pose.position.z;

    double x = odometry_msgs.pose.pose.orientation.x;
    double y = odometry_msgs.pose.pose.orientation.y;
    double z = odometry_msgs.pose.pose.orientation.z;
    double w = odometry_msgs.pose.pose.orientation.w;

    double roll = atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z);
    double pitch = atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z);
    double yaw =  asin(2*x*y + 2*z*w);

    double error = roll - this->goal_pitch;
    double limit = 0.1;
    if( (error < limit) && (error > -1*limit) ){
        //std::cout <<"Error true: "<<  error << std::endl;
        feedback_.ready = true;
    }else{
        //std::cout <<"Error false: "<<  error << std::endl;
        feedback_.ready = false;
    }

    this->height->updateError(error);
}