// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  evaluator.hpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         5/19/20
 *
 */
//-----------------------------------------------------------------------------


#ifndef EVALUATOR_HPP
#define EVALUATOR_HPP

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

namespace VO
{

class Evaluator {
  private:

  public:
    /* ------------------------------------------------------------------------ */
    inline Evaluator(ros::NodeHandle & nh) 
    : nh_(nh) {
      truth_sub_ = nh_local_.subscribe<nav_msgs::Odometry>("/scout_1/localization/odometry/truth", 
                                                     1, 
                                                     &Evaluator::truth_callback,
                                                     this);
      kimera_sub_ = nh_local_.subscribe<nav_msgs::Odometry>("/kimera_vio_ros/odometry", 
                                                     1, 
                                                     &Evaluator::kimera_callback,
                                                     this);
    };

    /* ------------------------------------------------------------------------ */
    void truth_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void kimera_callback(const nav_msgs::Odometry::ConstPtr& msg);

    /* ------------------------------------------------------------------------ */
    void print_error(double x, double y, double z);

  protected:
    /* ------------------------------------------------------------------------ */
    ros::NodeHandle & nh_;
    ros::NodeHandle nh_local_;

    /* ------------------------------------------------------------------------ */
    ros::Subscriber truth_sub_;
    ros::Subscriber kimera_sub_;

    /* ------------------------------------------------------------------------ */
    double x_;
    double y_;
    double z_;
    double x_prev_tru_=0;
    double y_prev_tru_=0;
    double z_prev_tru_=0;
    int k_;

    double kimera_x_;
    double kimera_y_;
    double kimera_z_;
    bool is_kimera_running = false;
};

}

#endif // EVALUATOR_HPP
