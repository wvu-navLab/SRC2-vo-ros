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
// possibly add flag ot compute covar
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
// Select covariance estimation method - DEREK
void Tracker::
stereoComputeCovariance(StereoImage & img_km1, StereoImage & img_k) {
      /// constants
  unsigned int covariance_method_;
  covariance_method_ = 1;
  static const unsigned int DellaertCov = 1;
  static const unsigned int MatthiesCov = 0;
  switch(covariance_method_) {
    case DellaertCov:
      stereoComputeCovarianceDellaert(img_km1, img_k);
      break;
    case MatthiesCov:
      stereoComputeCovarianceMatthies(img_km1, img_k);
      break;
    default:
      std::cout << "Invalid mode selected for Tracker::stereoComputeCovariance. \n";
      exit(1);
      break;
  }
}





/* ------------------------------------------------------------------------ */

void Tracker::
stereoComputeCovarianceDellaert(StereoImage & img_km1, StereoImage & img_k) {

  // disp_ = img_k.get_disp();
  // double disp_ = img_k.get_l_kpts()[i].pt.x 
  //   - img_k.get_r_kpts()[i].pt.x;
  // std::cout <<"\n disp_ = " << disp_;
  l_img_k = img_k.get_l_img();
  l_img_km1 = img_km1.get_l_img();
  r_img_k = img_k.get_r_img();
  r_img_km1 = img_km1.get_r_img();
  double bl = 0.07; // constant baseline (m)
  double sx = 238.352; // constant focal length (mm)
  double sigma_p = 0.1;
  double sigma_m = 0.1;

  //should change to assert
  if(img_km1.get_ipts().size() != img_k.get_ipts().size()) {
    std::cout << "Error! matched points not same between images \n";
    exit(1);
  }

  //loop over all tracked features
  for(int i=0; i<img_km1.get_l_kpts().size(); i++) {

    //d_km1 = disp_.at<short>(img_km1.get_ipts()[i].y, img_km1.get_ipts()[i].x)/16;
    d_k = img_k.get_l_kpts()[i].pt.x - img_k.get_r_kpts()[i].pt.x;

    if(d_k > 5 && d_k < 100)
    {

      //sx and bl are the same for km1 and k
      // double sx = (double) 
      //   l_img_k.get_l_info().getProjectionMatrix().at<double>(0,0); // sx = sy = focal length
      // double bl = (double) (-r_img_k.get_info().getProjectionMatrix().at<double>(0,3)
      //     /img_k.get_r_img().get_info().getProjectionMatrix().at<double>(0,0));
      kpt_ul = img_k.get_l_kpts()[i].pt.x;
      kpt_ur = img_k.get_r_kpts()[i].pt.x;
      kpt_vl = img_k.get_l_kpts()[i].pt.y;
      kpt_vr = img_k.get_r_kpts()[i].pt.y;
      // std::cout << "\n--- d_k = " << d_k;
      // std::cout << "\n--- d_km1 = " << d_km1;

      Jac_ =  (cv::Mat_<double>(3,3) <<
              bl/d_k, 0.0, -(kpt_ul*bl)/pow(d_k,2), 
              0.0, bl/d_k, -(kpt_vl*bl)/pow(d_k,2), 
              0.0, 0.0, -(sx*bl)/pow(d_k,2)); 
      
      // std::cout << "\n Jac NEW= " << Jac_;

      S_ = (cv::Mat_<double>(3,3) <<
        sigma_p, 0.0, 0.0,
        0.0, sigma_p, 0.0,
        0.0, 0.0, sigma_m);

      P_ = Jac_*S_*Jac_.t();
      // std::cout << "\n+++++  P_ = " << P_;

    }
  }

  // std::cout << "\n+++++  P_ = " << P_;
  // img_km1.P = something1;
  // img_k.P = something2;
}





/* ------------------------------------------------------------------------ */
void Tracker::
stereoComputeCovarianceMatthies(StereoImage & img_km1, StereoImage & img_k) {
  l_img_k = img_k.get_l_img();
  l_img_km1 = img_km1.get_l_img();
  r_img_k = img_k.get_r_img();
  r_img_km1 = img_km1.get_r_img();
  double bl = 0.07; // constant baseline (m)
  double sx = 238.352; // constant focal length (mm)
  double sigma_p = 0.1;
  double sigma_m = 0.1;

  //should change to assert
  if(img_km1.get_ipts().size() != img_k.get_ipts().size()) {
    std::cout << "Error! matched points not same between images \n";
    exit(1);
  }

  //loop over all tracked features
  for(int i=0; i<img_km1.get_l_kpts().size(); i++) {



    // if(i>0)
    // {
      d_km1 = img_km1.get_l_kpts()[i].pt.x - img_km1.get_r_kpts()[i].pt.x;
      d_k = img_k.get_l_kpts()[i].pt.x - img_k.get_r_kpts()[i].pt.x;

      kpt_ul = img_k.get_l_kpts()[i].pt.x;
      kpt_ur = img_k.get_r_kpts()[i].pt.x;
      kpt_vl = img_k.get_l_kpts()[i].pt.y;
      kpt_vr = img_k.get_r_kpts()[i].pt.y;

      double kpt_ul_m1 = img_km1.get_l_kpts()[i].pt.x;
      double kpt_ur_m1 = img_km1.get_r_kpts()[i].pt.x;
      double kpt_vl_m1 = img_km1.get_l_kpts()[i].pt.y;
      double kpt_vr_m1 = img_km1.get_r_kpts()[i].pt.y;
      // to make sure keypoints are valid and not e20+
      double err_flag_ = kpt_ul + kpt_ur + kpt_vl + kpt_vr + kpt_ul_m1 + kpt_ur_m1 + kpt_vl_m1 + kpt_vr_m1;
      double err_thresh = abs(err_flag_);

      if(d_k > 5 && d_k < 100 && d_km1 > 5 && d_km1 < 100 && err_thresh < 10000)
      {
        // std::cout << " \n matth Dkm1 :: " << d_km1;
        // std::cout << "\n matth DK :: " << d_k; 
        // //sx and bl are the same for km1 and k
        // double sx = (double) 
        //   l_img_k.get_l_info().getProjectionMatrix().at<double>(0,0); // sx = sy = focal length
        // double bl = (double) (-r_img_k.get_info().getProjectionMatrix().at<double>(0,3)
        //     /img_k.get_r_img().get_info().getProjectionMatrix().at<double>(0,0));

        // std::cout << "\n--- d_k = " << d_k;
        // std::cout << "\n--- d_km1 = " << d_km1;
        // std::cout << "\n ~~~~~~~~~kpt_ul_m1 : "<< kpt_ul_m1;

        Jac_km1 = (cv::Mat_<double>(3,4) <<
            -(2*bl*kpt_ur_m1)/pow(d_km1,2),  0.0, (2*bl*kpt_ul_m1)/pow(d_km1,2), 0.0,
            -(bl*(kpt_vl_m1 + kpt_vr_m1))/pow(d_km1,2), bl/(d_km1), (bl*(kpt_vl_m1 + kpt_vr_m1))/pow(d_km1,2),  bl/(d_km1),
            -(2*bl)/pow(d_km1,2), 0.0, 2*bl/pow(d_km1,2), 0.0); 


        Jac_k = (cv::Mat_<double>(3,4) <<
            -(2*bl*kpt_ur)/pow(d_k,2),  0.0, (2*bl*kpt_ul)/pow(d_k,2), 0.0,
            -(bl*(kpt_vl + kpt_vr))/pow(d_k,2), bl/(d_k), (bl*(kpt_vl + kpt_vr)/pow(d_k,2)),  bl/(d_k),
            -(2*bl)/pow(d_k,2), 0.0, 2*bl/pow(d_k,2), 0.0); 


        // std::cout << " \n kpt_vl_m1 = " << kpt_vl_m1;
        // std::cout << " \n kpt_vr_m1 = " << kpt_vr_m1;
        // std::cout << " \n kpt_vl_m1 = " << kpt_vl_m1;
        // std::cout << " \n kpt_vr_m1 = " << kpt_vr_m1;
        // std::cout <<" \n NUM k=====  " << -(bl*(kpt_vl + kpt_vr));
        // std::cout << " \n NUM km1==== " << -(bl*(kpt_vl_m1 + kpt_vr_m1));
        // std::cout << "\n Jac K " << Jac_k;
        // std::cout << "\n Jac Km1" << Jac_km1;
        // std::cout << "\n Jac K__ T " << Jac_k.t();
        // std::cout << "\n Jac Km1__ T" << Jac_km1.t();


        U_ = Jac_km1*(Jac_km1.t());
        V_ = Jac_k*(Jac_k.t());
        std::cout << "\n---U_ : " << U_;
        std::cout << "-\n--V_ : " << V_;
        W_ = (U_ + V_);
        std::cout << "\n****W_ = ****" << W_;
        // P_ = Jac_*S_*Jac_.t();
        // std::cout << "\n+++++  P_ = " << P_;

      }


    }
    // }

  
}












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
