// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  Tracker.cpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         9/28/18
 */
//-----------------------------------------------------------------------------

#include <Tracker.hpp>

namespace VO
{

/* ------------------------------------------------------------------------ */
void Tracker:: 
setGFTTParameters(const GFTTParameters params) {
  gftt_params_ = params;
  init_GFTT();
}

/* ------------------------------------------------------------------------ */
void Tracker:: 
setKLTParameters(const KLTParameters params) {
  klt_params_ = params;
  init_KLT();
} 

/* ------------------------------------------------------------------------ */
void Tracker::
stereoDetect(StereoImage & img) {
    //detect good features to track
    if(!is_GFTT_init_) {
      std::cout << "Error! GFTT not initialized!";
      std::cout << " Must call setGFTTParameters\n";
      exit(1);
    }

    //existing features
    std::vector<cv::KeyPoint> kpts_temp = img.get_l_kpts();
    std::vector<cv::Point2f>  ipts_temp = img.get_ipts();
    std::vector<int>          ages_temp = img.get_ages();
    std::vector<cv::Point3f>  opts_temp = img.get_opts();

    //detect features if number of features is less than allowed max
    if(img.get_ipts().size() < gftt_params_.max_features) {
      //detect keypoints
      std::vector<cv::KeyPoint> kpts_detected = detectGFTT(img.get_l_img());
      
      //loop over detected keypoints and add if 1. not too close to existing
      //keypoints and 2. disparity is valid 
      for(const auto & kpt_detected : kpts_detected) {
        bool add_keypoint = true;
        for(const auto & kpt_existing : img.get_l_kpts()) {
          //skip kpts too close to existing kpts
          if(utils::dist(kpt_detected.pt, kpt_existing.pt) 
              < gftt_params_.min_distance) {
            add_keypoint = false;
            break;
          }
        }

        if(add_keypoint) {
          //compute disparity and add kpt if disparity is valid
          cv::Point3f opt_detected;
          if(img.ipt_to_opt(kpt_detected.pt, opt_detected)) {
            kpts_temp.push_back(kpt_detected);
            ipts_temp.push_back(kpt_detected.pt);
            ages_temp.push_back(0);
            opts_temp.push_back(opt_detected);
          }
        }
      }

      //add keypoints and object points to image
      img.get_l_kpts() = kpts_temp;
      img.get_ipts()   = ipts_temp;
      img.get_ages()   = ages_temp;
      img.get_opts()   = opts_temp;
    }
};

/* ------------------------------------------------------------------------ */
void Tracker::
stereoTrack(StereoImage & img_km1, StereoImage & img_k) {
  //check if tracker parameters initialized
  if(!is_KLT_init_) {
    std::cout << "Error! KLT not initialized!";
    std::cout << " Must call setKLTParameters\n";
    exit(1);
  }

  //compute optical flow between previous and current images
  //here, we assume that the keypoints for the current image is empty
  std::vector<float> err;
  std::vector<uchar> status;
  calcOpticalFlowPyrLK(img_km1.get_l_img(),
                       img_k.get_l_img(),
                       img_km1.get_ipts(),
                       img_k.get_ipts(),
                       status,
                       err,
                       KLT_window_,
                       klt_params_.max_level,
                       KLT_termcrit_);

  //only keep tracked keypoints with valid disparity, and keep vectors 
  //aligned, so i at km1 corresponds to i at k for kpts, ipts, ages, opts  
  std::vector<cv::KeyPoint> kpts_k;
  std::vector<cv::Point2f>  ipts_k;
  std::vector<int>          ages_k;
  std::vector<cv::Point3f>  opts_k;
  std::vector<cv::KeyPoint> kpts_km1;
  std::vector<cv::Point2f>  ipts_km1;
  std::vector<int>          ages_km1;
  std::vector<cv::Point3f>  opts_km1;
  for(int i=0; i<status.size(); i++) {
    //if kpt is tracked, check disparity
    if(status[i]) { 
      //if disparity is valid track kpt, else ignore
      cv::Point3f opt;
      if(img_k.ipt_to_opt(img_k.get_ipts()[i], opt)) { //return false if invalid
        kpts_k.push_back(cv::KeyPoint(img_k.get_ipts()[i], 1.f));
        ipts_k.push_back(img_k.get_ipts()[i]);
        ages_k.push_back(img_km1.get_ages()[i] + 1);
        opts_k.push_back(opt);
        kpts_km1.push_back(img_km1.get_l_kpts()[i]);
        ipts_km1.push_back(img_km1.get_ipts()[i]);
        ages_km1.push_back(img_km1.get_ages()[i]);
        opts_km1.push_back(img_km1.get_opts()[i]);
      }
    }
  }
  img_k.get_l_kpts()   = kpts_k;
  img_k.get_ipts()     = ipts_k;
  img_k.get_ages()     = ages_k;
  img_k.get_opts()     = opts_k;
  img_km1.get_l_kpts() = kpts_km1;
  img_km1.get_ipts()   = ipts_km1;
  img_km1.get_ages()   = ages_km1;
  img_km1.get_opts()   = opts_km1;
};

/* ------------------------------------------------------------------------ */
void Tracker:: 
init_GFTT() {
    GFTT_ = cv::GFTTDetector::create(gftt_params_.max_features,
                                     gftt_params_.quality,
                                     gftt_params_.min_distance,
                                     gftt_params_.block_size,
                                     gftt_params_.use_harris_detector,
                                     gftt_params_.k);
    is_GFTT_init_ = true;
};

/* ------------------------------------------------------------------------ */
void Tracker:: 
init_KLT() {
  KLT_termcrit_ = cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 
                                             klt_params_.max_iter, 
                                             klt_params_.eps);
  KLT_window_ = cv::Size(klt_params_.window, klt_params_.window);
  is_KLT_init_ = true;
};

/* ------------------------------------------------------------------------ */
std::vector<cv::KeyPoint> Tracker:: 
detectGFTT(const cv::Mat & img) {
    cv::Mat mask(img.size(), CV_8U, cv::Scalar(255));
    std::vector<cv::KeyPoint> kpts;
    GFTT_->detect(img, kpts, mask);
    return kpts;
}

/* ------------------------------------------------------------------------ */
void Tracker::
print_GFTT_params() {
    std::cout << "param_GFTT_max_features_ = " 
        << gftt_params_.max_features << std::endl;
    std::cout << "param_GFTT_quality_ = " 
        << gftt_params_.quality << std::endl;
    std::cout << "param_GFTT_min_distance_ = " 
        << gftt_params_.min_distance << std::endl;
    std::cout << "param_GFTT_block_size_ = " 
        << gftt_params_.block_size << std::endl;
    std::cout << "param_GFTT_use_harris_detector_ = " 
        << gftt_params_.use_harris_detector << std::endl;
    std::cout << "param_GFTT_k_ = " 
        << gftt_params_.k << std::endl;
    std::cout << "is_GFTT_init_ = " 
        << is_GFTT_init_ << std::endl;
};

/* ------------------------------------------------------------------------ */
void Tracker::
print_KLT_params() {
    std::cout << "param_KLT_max_iter_ = " 
        << klt_params_.max_iter << std::endl;
    std::cout << "param_KLT_eps_ = " 
        << klt_params_.eps << std::endl;
    std::cout << "param_KLT_window_ = " 
        << klt_params_.window << std::endl;
    std::cout << "param_KLT_max_level_ = " 
        << klt_params_.max_level << std::endl;
    std::cout << "param_KLT_max_age_ = " 
        << klt_params_.max_age << std::endl;
    std::cout << "is_KLT_init_ = " 
        << is_KLT_init_ << std::endl;
};

}
