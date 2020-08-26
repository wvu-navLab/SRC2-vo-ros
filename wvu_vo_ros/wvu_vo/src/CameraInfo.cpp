// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  CameraInfo.cpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         9/14/18
 */
//-----------------------------------------------------------------------------

#include <CameraInfo.hpp>

namespace VO
{

/* ------------------------------------------------------------------------ */
bool CameraInfo::
loadYAML() {
	cv::FileStorage fs(filepath_, cv::FileStorage::READ);

	fs["distortion_coefficients"] >> distortion_coefficients_;
	fs["intrinsic_matrix"]        >> intrinsic_matrix_;
	fs["rectification_matrix"]    >> rectification_matrix_;
	fs["projection_matrix"]       >> projection_matrix_;

	return true;
}

/* ------------------------------------------------------------------------ */
cv::Mat CameraInfo::
projectionMatrix3x3() {
  return cv::Mat(projection_matrix_,cv::Rect(0,0,3,3));
}

/* ------------------------------------------------------------------------ */
cv::Mat CameraInfo::
distortionCoefficientsZeros() const {
  return (cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);
}

}