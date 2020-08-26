// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  evaluator_node.hpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         4/17/20
 *
 */
//-----------------------------------------------------------------------------

#include <ros/ros.h>
#include <wvu_vo_ros/ros_stereo_vo.hpp>
#include <wvu_vo_ros/evaluator.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_stereo_vo_node");
  ros::NodeHandle nh("");
  ros::Rate rate(50);

  ROS_INFO("ros_stereo_vo_node...");
  ROS_INFO("Starting VO...");
  VO::ROSStereoVO ros_stereo_vo(nh);

  //config for debugging
  // ros_stereo_vo.config_tune_block_matcher=1;

  ROS_INFO("Waiting for truth to initialize...");
  boost::shared_ptr<nav_msgs::Odometry const> sharedPtr;
  sharedPtr  = ros::topic::waitForMessage<nav_msgs::Odometry>("/scout_1/localization/odometry/truth", nh);
  ros_stereo_vo.set_pose(*sharedPtr);

  ROS_INFO("Starting evaluator...");
  VO::Evaluator evaluator(nh);

  double x_prev =  utils::get_xyz(ros_stereo_vo.get_T_02k_in_base())[0];
  double y_prev =  utils::get_xyz(ros_stereo_vo.get_T_02k_in_base())[1];
  double z_prev =  utils::get_xyz(ros_stereo_vo.get_T_02k_in_base())[2];

  while(ros::ok()) {
    double x =  utils::get_xyz(ros_stereo_vo.get_T_02k_in_base())[0];
    double y =  utils::get_xyz(ros_stereo_vo.get_T_02k_in_base())[1];
    double z =  utils::get_xyz(ros_stereo_vo.get_T_02k_in_base())[2];

    if(std::fabs(x - x_prev) > 1e-6) {
        evaluator.print_error(x, y, z);
    }

    x_prev = x;
    y_prev = y;
    z_prev = z;

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}