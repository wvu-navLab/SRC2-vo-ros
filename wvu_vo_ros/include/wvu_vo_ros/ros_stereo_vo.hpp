// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  ros_stereo_vo.hpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         4/17/20
 *
 */
//-----------------------------------------------------------------------------


#ifndef ROS_STEREO_VO_HPP
#define ROS_STEREO_VO_HPP

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <stereo_msgs/DisparityImage.h>

#include <wvu_vo_ros/SetPose.h>

#include <Types.hpp>
#include <StereoVO.hpp>

namespace VO
{

class ROSStereoVO : public StereoVO {
  private:
    typedef message_filters::sync_policies::
      ApproximateTime<sensor_msgs::Image,
                      sensor_msgs::Image,
                      sensor_msgs::CameraInfo,
                      sensor_msgs::CameraInfo> SyncPolicy;

  public:
    /* ------------------------------------------------------------------------ */
    inline ROSStereoVO(ros::NodeHandle & nh)
    : nh_(nh),
      l_img_sub(nh_, "camera/left/image_raw", 1),
      r_img_sub(nh_, "camera/right/image_raw", 1),
      l_info_sub(nh_, "camera/left/camera_info", 1),
      r_info_sub(nh_, "camera/right/camera_info", 1),
      // l_img_sub(nh_, "camera/left_sel/image_raw", 1),
      // r_img_sub(nh_, "camera/right_sel/image_raw", 1),
      // l_info_sub(nh_, "camera/left_sel/camera_info", 1),
      // r_info_sub(nh_, "camera/right_sel/camera_info", 1),
      sync_policy(SyncPolicy(10),
                  l_img_sub,
                  r_img_sub,
                  l_info_sub,
                  r_info_sub),
      StereoVO(StereoVO::DEFAULT) { //use default VO pipeline, see StereoVO.hpp
      //load parameters from server
      load_parameters();

      //create detector and tracker
      tracker_.print_GFTT_params();
      tracker_.print_KLT_params();

      //setup publishers
      disparity_pub_ = nh_.advertise<stereo_msgs::DisparityImage>("disparity",1);
      odom_pub_ = nh_.advertise<nav_msgs::Odometry>("vo",1);
      tracks_pub_ = nh_.advertise<sensor_msgs::Image>("feature_tracks",1);
      disparity_display_pub_ = nh_.advertise<sensor_msgs::Image>("disparity_display",1);

      //setup servers
      set_pose_srv_ = nh_.advertiseService("set_pose",
                                           &ROSStereoVO::set_pose_callback,
                                           this);

      //setup callbacks
      sync_policy.registerCallback(
        boost::bind(&ROSStereoVO::stereo_callback, this, _1, _2, _3, _4));
    };

    /* ------------------------------------------------------------------------ */
    void set_pose(const nav_msgs::Odometry & odom);

    /* ------------------------------------------------------------------------ */
    cv::Mat get_T_02k_in_base() const;
    cv::Mat get_T_k20_in_base() const;

    /* ------------------------------------------------------------------------ */
    bool set_pose_callback(wvu_vo_ros::SetPose::Request  & req,
                           wvu_vo_ros::SetPose::Response & res);

    /* ------------------------------------------------------------------------ */
    void stereo_callback(const sensor_msgs::ImageConstPtr      & l_img,
                         const sensor_msgs::ImageConstPtr      & r_img,
                         const sensor_msgs::CameraInfoConstPtr & l_info,
                         const sensor_msgs::CameraInfoConstPtr & r_info);

    /* ------------------------------------------------------------------------ */
    void publishDisparity();
    void publishPose();
    void publishTracks();

    /* ------------------------------------------------------------------------ */
    //TODO: clean up implementation of these options
    //config for debugging and tuning parameters, ALL should be default to 0
    int config_tune_block_matcher = 0;

  protected:
    /* ------------------------------------------------------------------------ */
    void load_parameters();

    /* ------------------------------------------------------------------------ */
    //TODO: add toggle vo

    /* ------------------------------------------------------------------------ */
    //nodehandle
    ros::NodeHandle & nh_;

    //services
    ros::ServiceServer set_pose_srv_;

    //subscribers
    message_filters::Subscriber<sensor_msgs::Image> l_img_sub;
    message_filters::Subscriber<sensor_msgs::Image> r_img_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> r_info_sub;

    //synchronization
    message_filters::Synchronizer<SyncPolicy> sync_policy;

    //publishers
    ros::Publisher disparity_pub_;
    ros::Publisher disparity_display_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher tracks_pub_;

    //
    ros::Time time_stamp_;
    ros::Time time_stamp_previous_;
    std::string base_frame_id_;
    std::string camera_frame_id_;

    //tf
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    /* ------------------------------------------------------------------------ */
    VO::StereoImage stereo_img_;

    /* ------------------------------------------------------------------------ */
    BMParameters bm_params_;

    /* ------------------------------------------------------------------------ */
    bool publish_tracks_;
};

}

#endif // ROS_STEREO_VO_HPP
