// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  CameraInfo.hpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         9/14/18
 */
//-----------------------------------------------------------------------------

#ifndef CameraInfo_HPP
#define CameraInfo_HPP

#include <opencv2/opencv.hpp>

namespace VO
{

class CameraInfo {
  public:
    /// constructors
    /* ------------------------------------------------------------------------ */
    inline CameraInfo() {};

    /* ------------------------------------------------------------------------ */
    inline CameraInfo(const double sx,
                      const double sy,
                      const double cx,
                      const double cy,
                      const double tx,
                      const double ty) {
        projection_matrix_ = (cv::Mat_<double>(3,4) << sx, 0,  cx, tx, 
                                                       0,  sy, cy, ty,
                                                       0,  0,  1,  0);
        distortion_coefficients_ = (cv::Mat_<double>(1,5) << 0, 0, 0, 0, 0);
    };

    /* ------------------------------------------------------------------------ */
    inline CameraInfo(const std::string filepath)
    : filepath_(filepath) {
        distortion_coefficients_ = (cv::Mat_<double>(1,5) << 0, 0, 0, 0, 0);
    };



    /// getters (ish)
    /* ------------------------------------------------------------------------ */
    inline cv::Mat getDistortionCoefficients() const {return distortion_coefficients_;};
    inline cv::Mat getIntrinsicMatrix()        const {return intrinsic_matrix_;       };
    inline cv::Mat getRectificationMatrix()    const {return rectification_matrix_;   };
    inline cv::Mat getProjectionMatrix()       const {return projection_matrix_;      };

    /* ------------------------------------------------------------------------ */
    //return 3x3 part of projection matrix
    cv::Mat projectionMatrix3x3();
    cv::Mat distortionCoefficientsZeros() const;



    /// methods
    /* ------------------------------------------------------------------------ */
    //load parameters from .yaml file
    bool loadYAML();

    //load parameters from kitti .txt file
    // bool loadKITTI();

    /* ------------------------------------------------------------------------ */
    //print all parameters
    inline void print() const {
    std::cout << "distortion_coefficients_ =\n" << distortion_coefficients_
        << "\nintrinsic_matrix_ =\n"      << intrinsic_matrix_
        << "\nrectification_matrix_ =\n"  << rectification_matrix_
        << "\nprojection_matrix_ =\n"     << projection_matrix_ << std::endl;
    }
    
  protected:
    std::string filepath_;

    int height_;
    int width_;

    cv::Mat distortion_coefficients_;
    cv::Mat intrinsic_matrix_;
    cv::Mat rectification_matrix_;
    cv::Mat projection_matrix_;

};

}

#endif // CameraInfo_HPP
