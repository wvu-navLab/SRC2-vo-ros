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

#ifndef VO_HPP
#define VO_HPP

#include <opencv2/opencv.hpp>
// #include <opencv2/xfeatures2d.hpp>

#include <Types.hpp>
#include <StereoImage.hpp>
#include <Tracker.hpp>

#include <unordered_set>
#include <stack>

namespace VO
{

class StereoVO  {
  public:
    /// constructors

    cv::Mat Sigma;
    cv::Mat std_dev;
    //cv::Mat P_i_;

    inline StereoVO(const unsigned int mode)
    : mode_(mode) {
        empty_ = true;
        T_ = (cv::Mat_<double>(4,4) << 
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0);
      //check if using proper opencv version, (although should be checked in
      //when compiling)
      std::cout << "OpenCV Version: " << CV_VERSION << std::endl;
      if(CV_MAJOR_VERSION < 3) {
        std::cout << "OpenCV 3.0 or later is required! Using Exiting...\n";
        exit(1);
      }
    };



    /// constants
    static const unsigned int LEFT_RIGHT = 0;
    static const unsigned int LEFT_DISPARITY = 1;
    static const unsigned int DEFAULT = 1;



    /// methods (public)
    /* ------------------------------------------------------------------------ */
    void setGFTTParameters(const GFTTParameters & params);

    /* ------------------------------------------------------------------------ */
    void setKLTParameters(const KLTParameters & params);

    /* ------------------------------------------------------------------------ */
    //processes undistorted rectified images
    void process(StereoImage & stereo_img);

    /* ------------------------------------------------------------------------ */
    //reset VO (resets pose and tracked features)
    void reset();



    /// getters (ish)
    /* ------------------------------------------------------------------------ */
    inline cv::Mat get_T_k20() const { return T_; };        //world to camera
    inline cv::Mat get_T_02k() const { return T_.inv(); };  //camera to world
    
    // inline GFTTParameters getGFTTParams() const { return gftt_params_; };
    // inline KLTParameters  getKLTParams () const { return klt_params_;  };

  protected:
    /// pipelines
    /* ------------------------------------------------------------------------ */
    //LEFT-RIGHT
    //1. matching between left and right images for computing sparse point cloud
    //2. matching between left image at k and previous left image at k-1
    //3. estimate motion using pnp in ransc with 2D-3D correspondences
    void processLeftRight(StereoImage img);

    /* ------------------------------------------------------------------------ */
    //LEFT-DISPARITY
    //1. compute disparity image using semi global block matcher
    //2. track between features in left images using optical flow
    //3. compute maximum clique in tracked features
    //4. estimate motion using pnp from 2D-3D matches in clique
    //5. add features if below max number of features after tracking
    void processLeftDisparity(StereoImage img);



    /// members (protected)
    /* ------------------------------------------------------------------------ */
    bool empty_;         //identifies if process was previously called
    StereoImage img_p_;  //previous stereo image
    cv::Mat T_;          //pose of camera
    unsigned int mode_;  //specifies pipeline for processing images
    cv::Mat delta_;      //transform from time step k-1 to k
    int status_;         //flag for VO status
    Tracker tracker_;    //feature tracker
    //cv::Mat Sigma;

    //variables for debugging
    StereoImage img_draw_; 
    StereoImage img_draw_p_;
    std::vector<cv::Point2f> ipts_clique_draw_;
    std::vector<cv::Point2f> ipts_clique_draw_p_;


    /// helper functions
    /* ------------------------------------------------------------------------ */
    //compute essential matrix and returns vector containing 1 for inlier
    //and 0 for outlier
    std::vector<uchar> epipolarConstraint(const std::vector<cv::Point2f> & ipts_p, 
                                          const std::vector<cv::Point2f> & ipts,
                                          CameraInfo & info_p,
                                          CameraInfo & info);

    /* ------------------------------------------------------------------------ */
    //compute maximum clique and return indices for matches in clique
    std::vector<int> compute_clique(StereoImage img, float delta);

    /* ------------------------------------------------------------------------ */
    // void _compute_residuals(const cv::Mat & img_l);
    // void _compute_jacobian (const cv::Mat & img_l);

 



    /// parameters
    /* ------------------------------------------------------------------------ */



    /// hacks (AKA "heuristics")
    /* ------------------------------------------------------------------------ */
    //hack only requiring mostly forward translation, if not forward, the
    //frame is skipped
    cv::Mat hack_require_forward_motion(const cv::Mat & delta);

    /* ------------------------------------------------------------------------ */
    //hack requiring small motion, if too large, the frame is skipped
    cv::Mat hack_require_small_motion(const cv::Mat & delta);

};

}

#endif // VO_HPP
