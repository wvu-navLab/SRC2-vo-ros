// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  ros_stereo_vo.cpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         4/17/20
 *
 */
//-----------------------------------------------------------------------------

#include <wvu_vo_ros/ros_stereo_vo.hpp>

#include <cv_bridge/cv_bridge.h>

namespace VO
{

/* ------------------------------------------------------------------------ */
void ROSStereoVO::
load_parameters() {
  //tracker parameters
  GFTTParameters gftt_params;
  KLTParameters klt_params;

  nh_.getParam("good_features_to_track/block_size", gftt_params.block_size);
  nh_.getParam("good_features_to_track/k", gftt_params.k);
  nh_.getParam("good_features_to_track/max_features", gftt_params.max_features);
  nh_.getParam("good_features_to_track/min_distance", gftt_params.min_distance);
  nh_.getParam("good_features_to_track/quality", gftt_params.quality);
  nh_.getParam("good_features_to_track/use_harris_detector", gftt_params.use_harris_detector);

  nh_.getParam("tracker/eps", klt_params.eps);
  nh_.getParam("tracker/max_age", klt_params.max_age);
  nh_.getParam("tracker/max_iter", klt_params.max_iter);
  nh_.getParam("tracker/max_level", klt_params.max_level);
  nh_.getParam("tracker/window", klt_params.window);

  tracker_.setGFTTParameters(gftt_params);
  tracker_.setKLTParameters(klt_params);

  //block matcher parameters
  nh_.getParam("block_matcher/block_size", bm_params_.block_size);
  nh_.getParam("block_matcher/max_disparity", bm_params_.max_disparity);
  nh_.getParam("block_matcher/min_disparity", bm_params_.min_disparity);
  nh_.getParam("block_matcher/num_disparities", bm_params_.num_disparities);
  nh_.getParam("block_matcher/prefilter_cap", bm_params_.prefilter_cap);
  nh_.getParam("block_matcher/prefilter_size", bm_params_.prefilter_size);
  nh_.getParam("block_matcher/speckle_range", bm_params_.speckle_range);
  nh_.getParam("block_matcher/speckle_window_size", bm_params_.speckle_window_size);
  nh_.getParam("block_matcher/uniqueness_ratio", bm_params_.uniqueness_ratio);
  nh_.getParam("block_matcher/use_semi_global", bm_params_.use_semi_global);

  //frame ids
  //TODO: get these automatically instead of requiring as parameters
  nh_.getParam("frame_ids/camera_frame_id", camera_frame_id_);
  nh_.getParam("frame_ids/base_frame_id", base_frame_id_);

  //parameters for debugging
  nh_.getParam("vo/publish_tracks", publish_tracks_);
};

/* ------------------------------------------------------------------------ */
void ROSStereoVO::
set_pose(const nav_msgs::Odometry & odom) {
  //frame ids
  std::string base_frame_id = odom.header.frame_id;

  //pose
  geometry_msgs::Pose pose = odom.pose.pose;

  //base transformation (camera frame to base frame)
  tf::StampedTransform T_b2c;

  try
  {
    tf_listener_.waitForTransform(base_frame_id_,
                                  camera_frame_id_,
                                  ros::Time(0),
                                  ros::Duration(1.0));
    tf_listener_.lookupTransform(base_frame_id_,
                                 camera_frame_id_,
                                 ros::Time(0),
                                 T_b2c);
  }
  catch(tf::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }

  //pose of robot (i.e., transform base 0 to base k, T_b02bk)
  tf::Quaternion q_b02bk(pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w);

  tf::Vector3 t_b02bk(pose.position.x,
                      pose.position.y,
                      pose.position.z);

  tf::Matrix3x3 R_b02bk(q_b02bk);

  //pose of camera
  tf::Transform T_b02bk(q_b02bk,t_b02bk);
  tf::Transform T_ck2c0 = T_b2c.inverse() * T_b02bk.inverse() * T_b2c;

  //convert to cv::Mat and set pose
  tf::Matrix3x3 R = T_ck2c0.getBasis();
  tf::Vector3 t = T_ck2c0.getOrigin();

  T_ = (cv::Mat_<double>(4,4) << R[0][0], R[0][1], R[0][2], t[0],
                                 R[1][0], R[1][1], R[1][2], t[1],
                                 R[2][0], R[2][1], R[2][2], t[2],
                                 0.0,     0.0,     0.0,     1.0);
}

/* ------------------------------------------------------------------------ */
cv::Mat ROSStereoVO::
get_T_02k_in_base() const {
  //base transformation (camera frame to base frame)
  tf::StampedTransform T_b2c;
  try
  {
    tf_listener_.waitForTransform(base_frame_id_,
                                  camera_frame_id_,
                                  ros::Time(0),
                                  ros::Duration(1.0));
    tf_listener_.lookupTransform(base_frame_id_,
                                 camera_frame_id_,
                                 ros::Time(0),
                                 T_b2c);
  }
  catch(tf::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }

  //pose of camera (transform from k=k to k=0)
  tf::Matrix3x3 R(T_.at<double>(0,0),
                  T_.at<double>(0,1),
                  T_.at<double>(0,2),
                  T_.at<double>(1,0),
                  T_.at<double>(1,1),
                  T_.at<double>(1,2),
                  T_.at<double>(2,0),
                  T_.at<double>(2,1),
                  T_.at<double>(2,2));

  tf::Vector3 t(T_.at<double>(0,3),
                T_.at<double>(1,3),
                T_.at<double>(2,3));

  //pose of base transform
  tf::Transform T_ck2c0(R, t);
  tf::Transform T_b02bk = T_b2c * T_ck2c0.inverse() * T_b2c.inverse();

  //return pose in base frame (i.e., transform base at k to base at 0)
  tf::Matrix3x3 R_b02bk = T_b02bk.getBasis();
  tf::Vector3 t_b02bk = T_b02bk.getOrigin();
  cv::Mat T = (cv::Mat_<double>(4,4) << R_b02bk[0][0], R_b02bk[0][1], R_b02bk[0][2], t_b02bk[0],
                                        R_b02bk[1][0], R_b02bk[1][1], R_b02bk[1][2], t_b02bk[1],
                                        R_b02bk[2][0], R_b02bk[2][1], R_b02bk[2][2], t_b02bk[2],
                                        0.0,     0.0,     0.0,     1.0);
  return T;
}

/* ------------------------------------------------------------------------ */
cv::Mat ROSStereoVO::
get_T_k20_in_base() const {
  cv::Mat T_02k = get_T_02k_in_base();
  return T_02k.inv();
}

bool ROSStereoVO::
set_pose_callback(wvu_vo_ros::SetPose::Request  & req,
                  wvu_vo_ros::SetPose::Response & res) {
  set_pose(req.odom);
  return true;
}

/* ------------------------------------------------------------------------ */
void ROSStereoVO::
stereo_callback(const sensor_msgs::ImageConstPtr      & l_img,
                const sensor_msgs::ImageConstPtr      & r_img,
                const sensor_msgs::CameraInfoConstPtr & l_info,
                const sensor_msgs::CameraInfoConstPtr & r_info) {
  //header
  time_stamp_previous_ = time_stamp_;
  time_stamp_ = l_img->header.stamp;

  //frame ids
  std::string camera_frame_id = l_img->header.frame_id;
  if(camera_frame_id != camera_frame_id_) {
    ROS_WARN("Error! camera_frame_id != camera_frame_id_");
    ROS_WARN("camera_frame_id = %s", camera_frame_id.c_str());
    ROS_WARN("camera_frame_id_ = %s", camera_frame_id_.c_str());
  }

  //images and camera parameters
  auto l_img_temp =
    cv_bridge::toCvCopy(l_img, sensor_msgs::image_encodings::MONO8);
  auto r_img_temp =
    cv_bridge::toCvCopy(r_img, sensor_msgs::image_encodings::MONO8);

  CameraInfo l_info_temp(l_info->P[0],
                         l_info->P[5],
                         l_info->P[2],
                         l_info->P[6],
                         0,  //NOTE: THIS SHOULD BE ZERO
                         0); //NOTE: THIS SHOULD BE ZERO

  CameraInfo r_info_temp(r_info->P[0],
                         r_info->P[5],
                         r_info->P[2],
                         r_info->P[6],
                         r_info->P[3],
                         r_info->P[7]);

  //create stereo image
  cv::Mat l_img_cv = l_img_temp->image.clone();
  cv::Mat r_img_cv = r_img_temp->image.clone();
  stereo_img_ = VO::StereoImage(l_img_cv,
                                r_img_cv,
                                l_info_temp,
                                r_info_temp);

  //TODO: instead, a service or another node should be used for testing
  //      frontend parameters
  if(config_tune_block_matcher==1) {
    static int temp_tuner_scale     = 100;
    static int temp_block_size      = bm_params_.block_size;
    static int temp_num_disparities = bm_params_.num_disparities;
    static int temp_min_disparity   = bm_params_.min_disparity;
    static int temp_max_disparity   = bm_params_.max_disparity;
    static int temp_uniquness_ratio = bm_params_.uniqueness_ratio;
    static int temp_speckle_window  = bm_params_.speckle_window_size;
    static int temp_speckle_range   = bm_params_.speckle_range;
    static int temp_prefilter_cap   = bm_params_.prefilter_cap;
    static int temp_prefilter_size  = bm_params_.prefilter_size;
    static int temp_use_semi_global = bm_params_.use_semi_global;
    stereo_img_.tuneAndSetBlockMatcherParams(temp_tuner_scale,
                                             temp_block_size,
                                             temp_num_disparities,
                                             temp_min_disparity,
                                             temp_max_disparity,
                                             temp_uniquness_ratio,
                                             temp_speckle_window,
                                             temp_speckle_range,
                                             temp_prefilter_cap,
                                             temp_prefilter_size,
                                             temp_use_semi_global);
  }
  else {
    //set parameters for computing disparity
    stereo_img_.setBlockMatcherParams(bm_params_.block_size,
                                      bm_params_.num_disparities,
                                      bm_params_.min_disparity,
                                      bm_params_.max_disparity,
                                      bm_params_.uniqueness_ratio,
                                      bm_params_.speckle_window_size,
                                      bm_params_.speckle_range,
                                      bm_params_.prefilter_cap,
                                      bm_params_.prefilter_size,
                                      bm_params_.use_semi_global);
  }

  //print camera parameters (as defined in stereo_img)
  // stereo_img_.get_l_info().print();
  // stereo_img_.get_r_info().print();

  //compute disparity image and publish result
  if(!stereo_img_.is_disparity_computed()) {
    stereo_img_.denseMatching();
    publishDisparity();
  }

  //estimate motion
  process(stereo_img_);

  //publish pose
  //only publish if transform is valid
  if(status_ == 0) {
    publishPose();

    //publish tracks
    if(publish_tracks_) {
      publishTracks();
    }
  }
  else {

  }

  //debug
  // cv::waitKey(1);
};

/* ------------------------------------------------------------------------ */
void ROSStereoVO::
publishDisparity() {
  //publish disparity (to topic "disparity")
  //cv::Mat = stereo_img_.get_disp();
  // ros::NodeHandle nh("");


   // ros::Rate loop_rate(10);
   //  //int count =0;
   // while (ros::ok())
   // {
    stereo_msgs::DisparityImagePtr disp_msg = boost::make_shared<stereo_msgs::DisparityImage>();
    cv::Mat cv_image;
    cv_image = stereo_img_.get_disp();
    disp_msg->min_disparity = bm_params_.min_disparity;
    disp_msg->max_disparity = bm_params_.min_disparity + bm_params_.num_disparities;

    disp_msg->valid_window.x_offset = 0;
    disp_msg->valid_window.y_offset = 0;
    disp_msg->valid_window.width    = cv_image.cols;
    disp_msg->valid_window.height   = cv_image.rows;
    disp_msg->T                     = (float) (-stereo_img_.get_r_info().getProjectionMatrix().at<double>(0,3)/stereo_img_.get_r_info().getProjectionMatrix().at<double>(0,0));
    disp_msg->f                     = (float) stereo_img_.get_l_info().getProjectionMatrix().at<double>(0,0);
    disp_msg->delta_d               = 0;
    disp_msg->header.stamp          = time_stamp_;
    disp_msg->header.frame_id       = camera_frame_id_;

    sensor_msgs::Image& dimage = disp_msg->image;
    dimage.width  = cv_image.size().width ;
    dimage.height = cv_image.size().height ;
    dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    dimage.step = dimage.width * sizeof(float);
    dimage.data.resize(dimage.step * dimage.height);
    cv::Mat_<float> dmat(dimage.height, dimage.width, reinterpret_cast<float*>(&dimage.data[0]), dimage.step);

    cv_image.convertTo(dmat,dmat.type());
    disparity_pub_.publish(disp_msg);


    //create sensor_msgs image for viewing in rviz
    cv::Mat disp8;
    cv::Mat temp_disp = stereo_img_.get_disp();
    temp_disp.convertTo(disp8, CV_8U, 1.0/(bm_params_.num_disparities*16)*256.0);
    sensor_msgs::ImagePtr temp_disp_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", disp8).toImageMsg();
    disparity_display_pub_.publish(temp_disp_msg);

    // ros::spinOnce();
    // loop_rate.sleep();
  //   ++count;
   // }
};

/* ------------------------------------------------------------------------ */
void ROSStereoVO::
publishPose() {
  //reference transformation
  tf::StampedTransform T_b2c; //camera frame to reference frame
  try
  {
    tf_listener_.waitForTransform(base_frame_id_,
                                  camera_frame_id_,
                                  ros::Time(0),
                                  ros::Duration(1.0));
    tf_listener_.lookupTransform(base_frame_id_,
                                 camera_frame_id_,
                                 ros::Time(0),
                                 T_b2c);
  }
  catch(tf::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }

  //pose of camera (transform from k=k to k=0)
  tf::Matrix3x3 R(T_.at<double>(0,0),
                  T_.at<double>(0,1),
                  T_.at<double>(0,2),
                  T_.at<double>(1,0),
                  T_.at<double>(1,1),
                  T_.at<double>(1,2),
                  T_.at<double>(2,0),
                  T_.at<double>(2,1),
                  T_.at<double>(2,2));

  tf::Vector3 t(T_.at<double>(0,3),
                T_.at<double>(1,3),
                T_.at<double>(2,3));

  //broadcast transform
  tf::Transform T_ck2c0(R, t);
  tf::Transform T_bk2b0 = T_b2c * T_ck2c0 * T_b2c.inverse();
  tf::Transform T_b02bk = T_bk2b0.inverse();
  // tf_broadcaster_.sendTransform(tf::StampedTransform(T_b02bk,
  //                                                    time_stamp_,
  //                                                    "vo",
  //                                                    base_frame_id_));

  //POSE
  //convert pose transform to geometry_msgs::PoseWithCovariance
  geometry_msgs::Transform pose_transform;
  tf::transformTFToMsg(T_b02bk, pose_transform);

  geometry_msgs::PoseWithCovariance pose_with_covariance; //we don't actually have covariance
  pose_with_covariance.pose.position.x = pose_transform.translation.x;
  pose_with_covariance.pose.position.y = pose_transform.translation.y;
  pose_with_covariance.pose.position.z = pose_transform.translation.z;
  pose_with_covariance.pose.orientation.x = pose_transform.rotation.x;
  pose_with_covariance.pose.orientation.y = pose_transform.rotation.y;
  pose_with_covariance.pose.orientation.z = pose_transform.rotation.z;
  pose_with_covariance.pose.orientation.w = pose_transform.rotation.w;



  //delta of camera (transform from k=k to k=k-1)
  tf::Matrix3x3 R_delta(delta_.at<double>(0,0),
                        delta_.at<double>(0,1),
                        delta_.at<double>(0,2),
                        delta_.at<double>(1,0),
                        delta_.at<double>(1,1),
                        delta_.at<double>(1,2),
                        delta_.at<double>(2,0),
                        delta_.at<double>(2,1),
                        delta_.at<double>(2,2));

  tf::Vector3 t_delta(delta_.at<double>(0,3),
                      delta_.at<double>(1,3),
                      delta_.at<double>(2,3));

  //broadcast transform
  tf::Transform T_delta_ck2c0(R_delta, t_delta);
  tf::Transform T_delta_bk2b0 = T_b2c * T_delta_ck2c0 * T_b2c.inverse();
  tf::Transform T_delta_b02bk = T_delta_bk2b0.inverse();

  //TWIST
  //convert delta transform to geometry_msgs::TwistWithCovariance
  geometry_msgs::Transform twist_transform;
  tf::transformTFToMsg(T_delta_b02bk, twist_transform);

  double delta_time = (time_stamp_ - time_stamp_previous_).toSec();

  geometry_msgs::TwistWithCovariance twist_with_covariance; //we don't actually have covariance
  twist_with_covariance.twist.linear.x = twist_transform.translation.x / delta_time;
  twist_with_covariance.twist.linear.y = twist_transform.translation.y / delta_time;
  twist_with_covariance.twist.linear.z = twist_transform.translation.z / delta_time;
  twist_with_covariance.twist.angular.x = 0; //not used
  twist_with_covariance.twist.angular.y = 0; //not used
  twist_with_covariance.twist.angular.z = 0; //not used
  twist_with_covariance.covariance[0] = Sigma.at<double>(0,0);
  twist_with_covariance.covariance[7] = Sigma.at<double>(1,1);
  twist_with_covariance.covariance[14] = Sigma.at<double>(2,2);
  twist_with_covariance.covariance[21] = Sigma.at<double>(3,3);
  twist_with_covariance.covariance[28] = Sigma.at<double>(4,4);
  twist_with_covariance.covariance[35] = Sigma.at<double>(5,5);
  std::cout << "\n Sigma size :" << Sigma.size();

  //ODOM
  nav_msgs::Odometry odom;
  odom.header.stamp = time_stamp_;
  odom.header.frame_id = "iris/odom"; // "vo"
  odom.child_frame_id = base_frame_id_;
  odom.pose = pose_with_covariance;
  odom.twist = twist_with_covariance;

  odom_pub_.publish(odom);
}



void ROSStereoVO::
publishCov() {





}







/* ------------------------------------------------------------------------ */
void ROSStereoVO::
publishTracks() {
  //publish feature tracks
  //TODO: need to add supress option (with default 1, to maintain
  //compatibility with existing code), so show_tracks_on_anaglyph and
  //show_image_points_color do not display an image. Instead, we just want
  //the returned images.
  // cv::Mat img_tracks =
  //   utils::show_tracks_on_anaglyph("feature tracks",
  //                                  img_draw_p_.get_ipts(),
  //                                  img_draw_.get_ipts(),
  //                                  img_draw_p_.get_l_img(),
  //                                  img_draw_.get_l_img(),
  //                                  1);
  // img_tracks = utils::show_image_points_color("feature tracks",
  //                                             img_tracks,
  //                                             ipts_clique_draw_,
  //                                             'y');

  cv::Mat img_tracks =
    utils::show_tracks_on_anaglyph("feature tracks",
                                   ipts_clique_draw_p_,
                                   ipts_clique_draw_,
                                   img_draw_p_.get_l_img(),
                                   img_draw_.get_l_img(),
                                   1);




  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_tracks).toImageMsg();
  tracks_pub_.publish(msg);
};

}
