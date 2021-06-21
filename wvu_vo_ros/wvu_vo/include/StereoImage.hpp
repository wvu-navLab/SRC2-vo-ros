// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  StereoImage.hpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         9/14/18
 */
//-----------------------------------------------------------------------------

#ifndef StereoImage_HPP
#define StereoImage_HPP

#include <opencv2/opencv.hpp>
#include <opencv_utils.hpp>

#include <Types.hpp>
#include <CameraInfo.hpp>
#include <Image.hpp>
// #include <StereoVO.hpp>

namespace VO
{

class StereoImage {
  public:
    /// constructors
    cv::Mat P_i_;
    /* ------------------------------------------------------------------------ */
    inline StereoImage() {};

    /* ------------------------------------------------------------------------ */
    inline StereoImage(const cv::Mat & l_img,
                       const cv::Mat & r_img,
                       const CameraInfo & l_info,
                       const CameraInfo & r_info)
    :l_img_(l_img, l_info),
     r_img_(r_img, r_info) {
        // compute_Q();
     };

    /* ------------------------------------------------------------------------ */
    inline StereoImage(const Image & l_img,
                       const Image & r_img)
    :l_img_(l_img),
     r_img_(r_img) {
        // compute_Q();
     };

    ///print
    /* ------------------------------------------------------------------------ */
    inline void print() {
        std::cout << "\n******** Stereo Image ********" << std::endl;
        std::cout << "LEFT INFO:\n";
        l_img_.get_info().print();
        std::cout << "RIGHT INFO:\n";
        r_img_.get_info().print();
        std::cout << "baseline = " << (double) 
            (-r_img_.get_info().getProjectionMatrix().at<double>(0,3)/
            r_img_.get_info().getProjectionMatrix().at<double>(0,0)) << std::endl;
        // std::cout << "Q = \n" << Q_ << std::endl;
    };

    /* ------------------------------------------------------------------------ */
    inline void printBlockMatcherParams() {
        std::cout << "block matcher parameters:";
        std::cout  << "\nbm_params_.block_size = " 
            << bm_params_.block_size;
        std::cout  << "\nnum_disparities = "       
            << bm_params_.num_disparities;
        std::cout  << "\nmin_disparity = "         
            << bm_params_.min_disparity;
        std::cout  << "\nmax_disparity = "         
            << bm_params_.max_disparity;
        std::cout  << "\nuniqueness_ratio = "      
            << bm_params_.uniqueness_ratio;
        std::cout  << "\nspeckle_window_size = "   
            << bm_params_.speckle_window_size;
        std::cout  << "\nspeckle_range = "         
            << bm_params_.speckle_range;
        std::cout  << "\nprefilter_cap = "         
            << bm_params_.prefilter_cap;
        std::cout  << "\nprefilter_size = "        
            << bm_params_.prefilter_size;
        std::cout  << "\nuse_semi_global = "       
            << bm_params_.use_semi_global;
        std::cout << std::endl;
    };



    /// setters
    /* ------------------------------------------------------------------------ */
    //sets block matcher parameters
    //number of disparities defines the maximum disparity (e.g., min_disparity +
    //num_disparities is the max disparity)
    void setBlockMatcherParams(const int & block_size,
                               const int & num_disparities,
                               const int & min_disparity,
                               const int & max_disparity,
                               const int & uniqueness_ratio,
                               const int & speckle_window_size,
                               const int & speckle_range,
                               const int & prefilter_cap,
                               const int & prefilter_size,
                               const int & use_semi_global);



    ///getters (ish)
    // TODO: the getters pass a non const references, so the protected members
    //       can be modified by external function, which kind of defeats the
    //       the purpose of having the member protected...
    /* ------------------------------------------------------------------------ */
    cv::Mat                   & get_l_img () { return l_img_.get_img();  };
    cv::Mat                   & get_r_img () { return r_img_.get_img();  };
    CameraInfo                & get_l_info() { return l_img_.get_info(); };
    CameraInfo                & get_r_info() { return r_img_.get_info(); };
    std::vector<cv::KeyPoint> & get_l_kpts() { return l_img_.get_kpts(); };
    std::vector<cv::KeyPoint> & get_r_kpts() { return r_img_.get_kpts(); };
    cv::Mat                   & get_l_desc() { return l_img_.get_desc(); };
    cv::Mat                   & get_r_desc() { return r_img_.get_desc(); };
    std::vector<cv::Point3f>  & get_opts  () { return l_img_.get_opts(); };
    std::vector<cv::Point2f>  & get_ipts  () { return l_img_.get_ipts(); };
    std::vector<cv::Point2f>  & get_ipts_r() { return r_img_.get_ipts_r(); };
    std::vector<int>          & get_ages  () { return l_img_.get_ages(); };
    cv::Mat                   & get_disp  () { return disp_;   };
    // cv::Mat                   & triangulate() {return P_i_;     };
    // cv::Mat                   & get_img3d () { return img3d_;  };

    Image & get_l_Image () { return l_img_; };
    Image & get_r_Image () { return r_img_; };

    ///public methods
    /* ------------------------------------------------------------------------ */
    //return true if disparity image is computed and false if disparity image 
    //has not been computed
    bool is_disparity_computed();
    void triangulate();
    /* ------------------------------------------------------------------------ */
    //sets block matcher params and adds trackbars to image, the trackbars will
    //update the values passed to the tuner function
    void tuneAndSetBlockMatcherParams(int & tuner_scale,
                                      int & block_size,
                                      int & num_disparities,
                                      int & min_disparity,
                                      int & max_disparity,
                                      int & uniqueness_ratio,
                                      int & speckle_window_size,
                                      int & speckle_range,
                                      int & prefilter_cap,
                                      int & prefilter_size,
                                      int & use_semi_global);

    /* ------------------------------------------------------------------------ */
    //creates trackbars for changing block matcher parameters
    void tune_block_matcher();

    /* ------------------------------------------------------------------------ */
    //performs sparse matching using keypoints, descriptors, and triangulates
    //matches to get 3d point
    void sparseMatching();

    /* ------------------------------------------------------------------------ */
    //computes disparity image using block matcher (does not compute 3d points)
    //TODO: add support for both cpu and gpu dense matching
    void denseMatching();
    // cv::Mat triangulate();// {return P_i_;};


    /* ------------------------------------------------------------------------ */
    //TODO: replace this method with method for converting disparity to 3D point
    bool ipt_to_opt(const cv::Point2f & ipt, cv::Point3f & opt);

  protected:
    /// structs
    /* ------------------------------------------------------------------------ */
    //struct for block matcher parameters
    // struct BlockMatcherParameters {
    //     int block_size;
    //     int num_disparities;
    //     int min_disparity;
    //     int max_disparity;
    //     int uniqueness_ratio;
    //     int texture_threshold;
    //     int speckle_window_size;
    //     int speckle_range;
    //     int prefilter_size;
    //     int prefilter_cap;
    //     int use_semi_global;
    //     bool empty = true;
    // };



    /// methods
    /* ------------------------------------------------------------------------ */
    //triangulates keypoints in left/right images  (result stored in opts_)
    // void triangulate();

    /* ------------------------------------------------------------------------ */
    //compute transformation Q that takes image points and depth to 3D points:
    //[X,Y,Z,W] = Q*[u,v,disparity,1]' where
    //Q = [1, 0, 0,   -cx;...
    //     0, 1, 0,   -cy;...
    //     0, 0, 0,    f;...
    //     0, 0, 1/bl, (cx-cx')/bl]
    //NOTE: Q is not used because the 3D are compute directly from projection
    //      matrix. Q was stored previously for using an OpenCV function
    // void compute_Q(); //not used anymore



    /// members
    /* ------------------------------------------------------------------------ */
    //header (need to add for synchronization)
    // const int timestamp_;
    // const std::string frame_id_;

    //image objects
    Image l_img_;
    Image r_img_;

    //matches between left and right
    std::vector<cv::DMatch> matches_;

    //disparity
    cv::Mat disp_;
    // cv::Mat Q_;     //not used anymore
    // cv::Mat img3d_; //not used anymore

    /* ------------------------------------------------------------------------ */
    //blockmatcher parameters
    BMParameters bm_params_;



    /// GUI
    /* ------------------------------------------------------------------------ */
    //create trackbar for modifying parameters
    void modify_param(std::string window_name, 
                      std::string param_name,
                      int *       param_pointer,
                      int         param_max_value);
    
    /* ------------------------------------------------------------------------ */
    int tuner_scale_ = 100;
};

}

#endif // StereoImage_HPP
