// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  opencv_utils.hpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         9/28/18
 *
 * TODO: add test script for at least the math operations
 */
//-----------------------------------------------------------------------------

#ifndef OPENCV_UTILS_HPP
#define OPENCV_UTILS_HPP

#include <opencv2/opencv.hpp>
// #include <opencv/highgui.h> 
#include <opencv2/highgui/highgui_c.h>
// #include <opencv2/xfeatures2d.hpp>

// #include <fstream>

namespace utils
{

/****************************************************************************
                                   Typedefs
*****************************************************************************/
//"v" represents vector
typedef std::vector<cv::KeyPoint> v_kpt;
typedef std::vector<cv::DMatch>   v_m;
typedef std::vector<cv::Point2f>  v_pt2f;
typedef std::vector<cv::Point3f>  v_pt3f;

//"t" represents type
typedef cv::KeyPoint t_kpt;
typedef cv::DMatch   t_m;
typedef cv::Point2f  t_pt2f;
typedef cv::Point3f  t_pt3f;

/****************************************************************************
                                OpenCV Debugging 
*****************************************************************************/
//print cv::Mat type and code for accessing elements
void print_Mat_info(const cv::Mat & img);

/****************************************************************************
                                Visualization
*****************************************************************************/
//display image
void show_image    (      std::string   title,
                    const cv::Mat     & img); 

//display stereo pair
void show_stereo   (      std::string   title,
                    const cv::Mat     & img_l,
                    const cv::Mat     & img_r);

//display anaglyph
void show_anaglyph(      std::string   title,
                    const cv::Mat     & img_l,
                    const cv::Mat     & img_r);

//display keypoints
void show_keypoints(      std::string   title,
                    const cv::Mat     & img,
                    const v_kpt       & kpts);

void show_image_points(      std::string   title,
                       const cv::Mat     & img_l,
                       const v_pt2f      & ipts_l);

cv::Mat show_image_points_color(      std::string   title,
                                const cv::Mat     & img,
                                const v_pt2f      & ipts,
                                      char          color);

//display feature tracks
void show_tracks   (      std::string   title,
                    const v_pt2f      & kpts_i,
                    const v_pt2f      & kpts_j,
                    const cv::Mat     & img_j);

void show_tracks   (      std::string   title,
                    const v_kpt       & kpts_i,
                    const v_kpt       & kpts_j,
                    const cv::Mat     & img_j);

cv::Mat show_tracks_on_anaglyph   (      std::string   title,
                                   const v_pt2f      & kpts_i,
                                   const v_pt2f      & kpts_j,
                                   const cv::Mat     & img_i,
                                   const cv::Mat     & img_j,
                                   int = 0);

cv::Mat show_tracks_on_anaglyph   (      std::string   title,
                                   const v_kpt       & kpts_i,
                                   const v_kpt       & kpts_j,
                                   const cv::Mat     & img_i,
                                   const cv::Mat     & img_j,
                                   int = 0);

void save_tracks_on_anaglyph   (      std::string   title,
                                const v_pt2f      & kpts_i,
                                const v_pt2f      & kpts_j,
                                const cv::Mat     & img_i,
                                const cv::Mat     & img_j);

void save_tracks_on_anaglyph   (      std::string   title,
                                const v_kpt       & kpts_i,
                                const v_kpt       & kpts_j,
                                const cv::Mat     & img_i,
                                const cv::Mat     & img_j);

//display matches
void show_matches  (      std::string   title,
                    const cv::Mat     & img_i, 
                    const cv::Mat     & img_j, 
                    const v_kpt       & kp_i, 
                    const v_kpt       & kp_j, 
                    const v_m         & matches);

//write .ply file
void write_ply_xyzrgb(      std::string   filename,
                      const v_pt3f      & pts3f);

/****************************************************************************
                            Image Processing
*****************************************************************************/
//sharpen
void sharpen (cv::Mat & img);

//smooth
void smooth(cv::Mat & img);

//image gradients
void compute_gradient_x(cv::Mat & img);
void compute_gradient_y(cv::Mat & img);

//blob filter
void filter_blobs(cv::Mat & img);

//corner filter
void filter_corners(cv::Mat & img);

/****************************************************************************
                                Features
*****************************************************************************/
//compute keypoints
v_kpt compute_keypoints(const cv::Mat & img);

//compute subpixel
void compute_subpixel(const cv::Mat & img, 
                            v_kpt   & kpts);

//compute descriptors
cv::Mat compute_descriptors(const cv::Mat & img, 
                                  v_kpt   & kpts);

//compute matches
v_m compute_matches (cv::Mat desc_i, 
                     cv::Mat desc_j);

//delete outliers
void delete_outliers (v_pt2f             & ipts,
                      std::vector<uchar>   inliers);

void delete_outliers (v_pt3f             & opts,
                      std::vector<uchar>   inliers);

void delete_outliers (v_kpt              & kpts,
                      std::vector<uchar>   inliers);

void delete_outliers (cv::Mat            & desc,
                      std::vector<uchar>   inliers);

//filter matches and convert
v_pt2f filter_matches_pt2f(v_kpt & kpts, 
                           v_m   & matches, 
                           bool    is_right);

//filter matches
void filter_matches(v_kpt & kpts, 
                    v_m   & matches, 
                    bool    is_right);

void filter_matches(cv::Mat & desc, 
                    v_m     & matches, 
                    bool      is_right);

void filter_matches(v_pt2f  & ipts, 
                    v_m     & matches, 
                    bool      is_right);

void filter_matches(v_pt3f  & opts, 
                    v_m     & matches, 
                    bool      is_right);

//convert opencv types
void convert_kpts_to_ipts(v_kpt  & kpts, 
                          v_pt2f & ipts);

//filter points
void filter_points(      v_pt2f           & ipts,
                   const std::vector<int> & ids);

void filter_points(      v_pt3f           & opts,
                   const std::vector<int> & ids);

/****************************************************************************
                            Transformations
*****************************************************************************/
//get 4x4 transformation matrix
cv::Mat compose_T (const cv::Mat & rvec,
                   const cv::Mat & tvec);

//get xyx from 4x4 transformation
std::vector<double> get_xyz(const cv::Mat & T);

//get rpy from 4x4 transformation
std::vector<double> get_rpy(const cv::Mat & T);

// //get euler angles from 4x4 transformation
// std::vector<double> get_euler_zyx(const cv::Mat & T);

// //get quaternion from 4x4 transformation
// std::vector<double> get_rot_from_quat(const cv::Mat & T);

/****************************************************************************
                                Math
*****************************************************************************/
//compute distance between n dimensional points p1 and p2
double dist(std::vector<double> p1, std::vector<double> p2);
float dist(cv::Point2f p1, cv::Point2f p2);
double dist(cv::Point2d p1, cv::Point2d p2);
float dist(cv::Point3f p1, cv::Point3f p2);
double dist(cv::Point3d p1, cv::Point3d p2);

}

#endif // OPENCV_UTILS_HPP
