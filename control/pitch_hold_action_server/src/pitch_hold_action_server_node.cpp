#include <ros/ros.h>
#include "pitch_hold_action_server/pitch_hold_action_server_ros.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pitch_hold_action_server");

  PitchHoldAction pitchhold("pitch_hold_action_server");
  ros::spin();

  return 0;
}