// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  evaluator.cpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         5/19/20
 *
 */
//-----------------------------------------------------------------------------

#include <wvu_vo_ros/evaluator.hpp>

namespace VO
{

/* ------------------------------------------------------------------------ */
void Evaluator::
truth_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  z_ = msg->pose.pose.position.z;
};

/* ------------------------------------------------------------------------ */
void Evaluator::
kimera_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  kimera_x_ = msg->pose.pose.position.x;
  kimera_y_ = msg->pose.pose.position.y;
  kimera_z_ = msg->pose.pose.position.z;
  is_kimera_running = true;
};

/* ------------------------------------------------------------------------ */
void Evaluator::
print_error(double x, double y, double z) {
  //print header
  
  if(is_kimera_running) {
    static bool print_header_kim = true;
    if(print_header_kim) {
    std::cout << std::left 
              <<        std::setw(5)  << "k_" 
              <<        std::setw(13) << "err" 
              <<        std::setw(13) << "kim err" 
              << "%" << std::setw(13) << "err"
              << "%" << std::setw(13) << "kim err"
              <<        std::setw(13) << "x_est"
              <<        std::setw(13) << "x_tru"
              <<        std::setw(13) << "x_kim"
              <<        std::setw(13) << "y_est"
              <<        std::setw(13) << "y_tru"
              <<        std::setw(13) << "y_kim"
              <<        std::setw(13) << "z_est"
              <<        std::setw(13) << "z_tru"
              <<        std::setw(13) << "z_kim"
              << std::endl;
      print_header_kim = false;
    }
  }
  else {
    static bool print_header = true;
    if(print_header) {
    std::cout << std::left 
              <<        std::setw(5)  << "k_" 
              <<        std::setw(13) << "error" 
              << "%" << std::setw(13) << "error"
              <<        std::setw(13) << "x_est"
              <<        std::setw(13) << "x_tru"
              <<        std::setw(13) << "y_est"
              <<        std::setw(13) << "y_tru"
              <<        std::setw(13) << "z_est"
              <<        std::setw(13) << "z_tru"
              << std::endl;
      print_header = false;
    }
  }

  //compute error in distance
  double dx = x_ - x;
  double dy = y_ - y;
  double dz = 0;
  double err = std::sqrt( dx*dx + dy*dy + dz*dz );

  //compute error in distance (kimera)
  double kimera_dx=0;
  double kimera_dy=0;
  double kimera_dz=0;
  double kimera_err=0;
  if(is_kimera_running) {
    kimera_dx = x_ - kimera_x_;
    kimera_dy = y_ - kimera_y_;
    kimera_dz = 0;
    kimera_err = std::sqrt( kimera_dx*kimera_dx + kimera_dy*kimera_dy + kimera_dz*kimera_dz );
  }

  //accumulate distance
  static double delta_tru = 0;
  static bool is_first_step = true;
  if(!is_first_step) {
    double dx_tru = x_ - x_prev_tru_;
    double dy_tru = y_ - y_prev_tru_;
    double dz_tru = 0;
    delta_tru = delta_tru + std::sqrt(dx_tru*dx_tru + dy_tru*dy_tru + dz_tru*dz_tru); 
  }
  else {
    k_=0;
    is_first_step = false;
  }

  //print error and states
  if(is_kimera_running) {
    std::cout << std::left 
              <<        std::setw(5)  << k_
              <<        std::setw(13) << std::setprecision(6) << err
              <<        std::setw(13) << std::setprecision(6) << kimera_err
              << "%" << std::setw(13) << std::setprecision(6) << 130.0*err/delta_tru
              << "%" << std::setw(13) << std::setprecision(6) << 130.0*kimera_err/delta_tru
              <<        std::setw(13) << std::setprecision(6) << x
              <<        std::setw(13) << std::setprecision(6) << x_
              <<        std::setw(13) << std::setprecision(6) << kimera_x_
              <<        std::setw(13) << std::setprecision(6) << y
              <<        std::setw(13) << std::setprecision(6) << y_
              <<        std::setw(13) << std::setprecision(6) << kimera_y_
              <<        std::setw(13) << std::setprecision(6) << z
              <<        std::setw(13) << std::setprecision(6) << z_
              <<        std::setw(13) << std::setprecision(6) << kimera_z_
              << std::endl;
  }
  else {
    std::cout << std::left 
              <<        std::setw(5)  << k_
              <<        std::setw(13) << std::setprecision(6) << err
              << "%" << std::setw(13) << std::setprecision(6) << 130.0*err/delta_tru
              <<        std::setw(13) << std::setprecision(6) << x
              <<        std::setw(13) << std::setprecision(6) << x_
              <<        std::setw(13) << std::setprecision(6) << y
              <<        std::setw(13) << std::setprecision(6) << y_
              <<        std::setw(13) << std::setprecision(6) << z
              <<        std::setw(13) << std::setprecision(6) << z_
              << std::endl;
  }
  
  //update vars
  x_prev_tru_ = x_;
  y_prev_tru_ = y_;
  z_prev_tru_ = z_;
  k_++;
};

}
