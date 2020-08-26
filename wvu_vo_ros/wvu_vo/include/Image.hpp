// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  Image.hpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         9/14/18
 */
//-----------------------------------------------------------------------------

#ifndef Image_HPP
#define Image_HPP

#include <opencv2/opencv.hpp>
#include <opencv_utils.hpp>

#include <CameraInfo.hpp>

namespace VO
{

class Image {
  public:
    /* ------------------------------------------------------------------------ */
    inline Image() {};

    /* ------------------------------------------------------------------------ */
    inline Image(const cv::Mat & img,
                 const CameraInfo & info)
    :img_(img),
     info_(info) {};


    ///print
    /* ------------------------------------------------------------------------ */
    inline void print() {
        std::cout << "\n******** Image ********" << std::endl;
        std::cout << "INFO:\n";
        info_.print();
    };

    ///getters (ish)
    /* ------------------------------------------------------------------------ */
    cv::Mat                   & get_img () { return img_;  };
    CameraInfo                & get_info() { return info_; };
    std::vector<cv::KeyPoint> & get_kpts() { return kpts_; };
    cv::Mat                   & get_desc() { return desc_; };
    std::vector<cv::Point3f>  & get_opts() { return opts_; };
    std::vector<cv::Point2f>  & get_ipts() { return ipts_; };
    std::vector<int>          & get_ages() { return ages_; };
    // cv::Mat                   & get_disp() { return disp_; };

  protected:
    /// members
    /* ------------------------------------------------------------------------ */
    cv::Mat                   img_;  //image raw
    CameraInfo                info_; //image camera parameters

    /* ------------------------------------------------------------------------ */
    std::vector<cv::KeyPoint> kpts_; //image keypoints
    cv::Mat                   desc_; //image descriptors
    std::vector<cv::Point3f>  opts_; //object points (as XYZ coordinates)
    std::vector<cv::Point2f>  ipts_; //image points 
    std::vector<int>          ages_; //age of tracked features

    /* ------------------------------------------------------------------------ */
    // cv::Mat disp_; //disparity image
};

}

#endif // Image_HPP
