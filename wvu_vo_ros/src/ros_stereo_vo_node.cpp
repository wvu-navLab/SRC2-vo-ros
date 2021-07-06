// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  ros_stereo_vo_node.hpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         4/17/20
 *
 */
//-----------------------------------------------------------------------------

#include <ros/ros.h>
#include <wvu_vo_ros/ros_stereo_vo.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_stereo_vo_node");
  ros::NodeHandle nh("");
  ros::Rate rate(50);

  ROS_INFO("ros_stereo_vo_node...");

  VO::ROSStereoVO ros_stereo_vo(nh);

  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  // ros::spin();

	return 0;
}
