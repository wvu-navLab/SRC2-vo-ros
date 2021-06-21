// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  StereoVO.hpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         9/14/18
 *
 */
//-----------------------------------------------------------------------------

#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <opencv2/opencv.hpp>

#include <Types.hpp>
#include <StereoImage.hpp>

namespace VO
{

class Tracker {
  public:
    /// constructors
    cv::Mat U_, V_, W_, Jac_, Jac_k, Jac_km1, P_, S_, disp_, l_img_k, l_img_km1, r_img_k, r_img_km1;
    double d_k, d_km1;
    double kpt_ul, kpt_ur, kpt_vl, kpt_vr;
    Tracker() {};


    /// setters (public)
    /* ------------------------------------------------------------------------ */
    void setGFTTParameters(const GFTTParameters params);

    /* ------------------------------------------------------------------------ */
    void setKLTParameters(const KLTParameters params);



    /// methods (public)
    /* ------------------------------------------------------------------------ */
    //detect features in image 
    void stereoDetect(StereoImage & img);

    /* ------------------------------------------------------------------------ */
    //track features between frames
    void stereoTrack(StereoImage & img_km1, StereoImage & img_k);
    /* ------------------------------------------------------------------------ */

    //select covariance estimation method
    void stereoComputeCovariance(StereoImage & img_km1, StereoImage & img_k);

     /* ------------------------------------------------------------------------ */
    void stereoComputeCovarianceDellaert(StereoImage & img_km1, StereoImage & img_k);

    /* ------------------------------------------------------------------------ */
    void stereoComputeCovarianceMatthies(StereoImage & img_km1, StereoImage & img_k);



    /// debug
    /* ------------------------------------------------------------------------ */
    void print_GFTT_params();

    /* ------------------------------------------------------------------------ */
    void print_KLT_params();

  protected:
    /// methods (protected)
    /* ------------------------------------------------------------------------ */
    void init_GFTT();
    void init_KLT();

    /* ------------------------------------------------------------------------ */
    std::vector<cv::KeyPoint> detectGFTT(const cv::Mat & img);



    /// members (protected)
    /* ------------------------------------------------------------------------ */
    //parameters for good features to track
    cv::Ptr<cv::Feature2D> GFTT_;
    GFTTParameters gftt_params_;
    bool is_GFTT_init_ = false;

    /* ------------------------------------------------------------------------ */
    // parameters for kanade lucas tomasi feature tracker
    cv::TermCriteria KLT_termcrit_;
    cv::Size KLT_window_;
    KLTParameters klt_params_;
    bool is_KLT_init_ = false;
};

}

#endif // TRACKER_HPP
