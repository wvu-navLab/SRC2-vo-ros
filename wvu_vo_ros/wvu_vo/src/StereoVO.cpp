// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  StereoVO.cpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         9/28/18
 */
//-----------------------------------------------------------------------------

#include <StereoVO.hpp>
// #include <eigen3/Eigen/Dense>
#include <typeinfo>


namespace VO
{

/* ------------------------------------------------------------------------ */
void StereoVO::
setGFTTParameters(const GFTTParameters & params) {
  tracker_.setGFTTParameters(params);
};

/* ------------------------------------------------------------------------ */
void StereoVO::
setKLTParameters(const KLTParameters & params) {
  tracker_.setKLTParameters(params);
};


/* ------------------------------------------------------------------------ */
void StereoVO::
process(StereoImage & stereo_img)
{
  switch(mode_) {
    case LEFT_RIGHT:
      processLeftRight(stereo_img);
      break;
    case LEFT_DISPARITY:
      processLeftDisparity(stereo_img);
      break;
    default:
      std::cout << "Invalid mode selected for StereoVO::process. Using \n";
      exit(1);
      break;
  }
}

/* ------------------------------------------------------------------------ */
void StereoVO::
reset() {
  T_ = (cv::Mat_<double>(4,4) <<
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0);

  empty_ = true;
}

/* ------------------------------------------------------------------------ */
void StereoVO::
processLeftRight(StereoImage img) {
  //sharpen
  utils::sharpen(img.get_l_img());
  utils::sharpen(img.get_r_img());

  //triangulate
  img.sparseMatching();

  //check if initial image, if so add data and skip
  if(empty_)
  {
    img_p_ = img;
    empty_ = false;
    return;
  }

  //compute matches between previous frame
  std::vector<cv::DMatch> mP = utils::compute_matches(img_p_.get_l_desc(),
                                                      img.get_l_desc());

  //add matched 2d/3d points
  img.get_ipts() = utils::filter_matches_pt2f(img.get_l_kpts(), mP, 0);
  img_p_.get_ipts() = utils::filter_matches_pt2f(img_p_.get_l_kpts(), mP, 1);
  utils::filter_matches(img_p_.get_opts(), mP, 1);

  //check number of points
  if(img_p_.get_opts().size() < 5)
  {
    std::cout << "Not enough observed points to compute motion!" << std::endl;
    return;
  }

  //essential constraint (for removing outliers only)
  std::vector<uchar> inliers_epipolar;
  inliers_epipolar = epipolarConstraint(img_p_.get_ipts(),
                                        img.get_ipts(),
                                        img.get_l_info(),
                                        img.get_l_info());
  utils::delete_outliers(img_p_.get_ipts(), inliers_epipolar);
  utils::delete_outliers(img_p_.get_opts(), inliers_epipolar);
  utils::delete_outliers(img.get_ipts(), inliers_epipolar);

  //estimate motion ransac
  cv::Mat rvec, tvec;
  std::vector<uchar> inliers_ransac;
  cv::solvePnPRansac(img_p_.get_opts(),
                     img.get_ipts(),
                     img.get_l_info().projectionMatrix3x3(),
                     img.get_l_info().distortionCoefficientsZeros(),
                     rvec,
                     tvec,
                     false,
                     200,
                     0.5,
                     0.99,
                     inliers_ransac,
                     1 //CV_EPNP
                     );

  //estimate motion iterative
  utils::delete_outliers(img_p_.get_ipts(), inliers_ransac);
  utils::delete_outliers(img_p_.get_opts(), inliers_ransac);
  utils::delete_outliers(img.get_ipts(), inliers_ransac);

  // cv::solvePnP(img_p_.get_opts(),
  //              img.get_ipts(),
  //              img.get_l_info().projectionMatrix3x3(),
  //              img.get_l_info().distortionCoefficientsZeros(),
  //              rvec,
  //              tvec,
  //              true,
  //              CV_ITERATIVE
  //              );

  //show tracks after removing outliers
  // utils::show_tracks("process_lr: show_tracks (after outlier removal)",
  //                    img_p_.get_ipts(),
  //                    img.get_ipts(),
  //                    img.get_l_img());

  //compute motion (transform from previous left to current left)
  cv::Mat delta_ = utils::compose_T(rvec, tvec);
  // delta_ = hack_require_forward_motion(delta_);
  // delta_ = hack_require_small_motion(delta_);

  //update pose
  T_ = delta_*T_;

  //update vars
  img_p_ = img;

  return;
};

/* ------------------------------------------------------------------------ */
void StereoVO::
processLeftDisparity(StereoImage img) {
  status_ = -1;

  ///////////////////////////////
  /////////PREPROCESSING/////////
  ///////////////////////////////
  //smooth using edge preserving filter
  // utils::smooth(img.get_l_img());
  // utils::smooth(img.get_r_img());
  // utils::show_stereo("l_img, r_img",img.get_l_img() ,img.get_r_img());
  // utils::show_anaglyph("anaglyph",img.get_l_img() ,img.get_r_img());

  ///////////////////////////
  /////////DISPARITY/////////
  ///////////////////////////
  //compute disparity
  if(!img.is_disparity_computed()) {
    // std::cout << "processLeftDisparity: denseMatching\n";
    img.denseMatching();
  }


  /////////////////////////////////
  /////////DETECT FEATURES/////////
  /////////////////////////////////
  //check if initial image, if so detect features
  if(empty_) {
    // std::cout << "init stereoDetect\n";
    tracker_.stereoDetect(img);

    //store image for next iteration
    img_p_ = img;
    empty_ = false;

    return;
  }

  ////////////////////////////////
  /////////TRACK FEATURES/////////
  ////////////////////////////////
  //TODO: add parameter for this
  if(img_p_.get_l_kpts().size() < 5) {
    std::cout << "Not enough features in previous frame for tracking!\n";

    //set delta equal to zero
    delta_ =  (cv::Mat_<double>(4,4) <<
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0);

    //reintialize tracker (detect keypoints and add to image structure)
    // std::cout << "processLeftDisparity: reinit stereoDetect 1\n";
    tracker_.stereoDetect(img);

    //store image for next iteration
    img_p_ = img;

    //set status where -1 means VO failed at time step k
    status_ = -1;

    return;
  }

  tracker_.stereoTrack(img_p_, img);

  //////////////////////////////////////////////////////
  /////////CHECK IF ENOUGH POINTS FOR TRACKING//////////
  //////////////////////////////////////////////////////
  //TODO: add parameter for this
  if(img.get_opts().size() < 5)
  {
    std::cout << "Not enough observed points to compute motion!\n";

    //set delta equal to zero
    delta_ =  (cv::Mat_<double>(4,4) <<
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0);

    //reintialize tracker (detect keypoints and add to image structure)
    // std::cout << "processLeftDisparity: reinit stereoDetect 2\n";
    tracker_.stereoDetect(img);

    //store image for next iteration
    img_p_ = img;

    //set status where -1 means VO failed at time step k
    status_ = -1;

    return;
  }

  /////////////////////////////////
  /////////MAXIMUM CLIQUE//////////
  /////////////////////////////////
  // std::cout << "compute_clique\n";
  std::vector<cv::Point2f> ipts_clique = img.get_l_Image().get_ipts();
  std::vector<cv::Point2f> ipts_clique_p = img_p_.get_l_Image().get_ipts();
  std::vector<cv::Point3f> opts_clique = img_p_.get_l_Image().get_opts();
  cv::Mat ipts_clique_out;// = img_p_.get_l_Image().get_ipts();

  //cv::Point2f(ipts_clique_out.at<double>(i, 0), ipts_clique_out.at<double>(i, 1))



  std::vector<int> clique = compute_clique(img, 0.1);
  utils::filter_points(ipts_clique, clique);
  utils::filter_points(ipts_clique_p, clique);
  utils::filter_points(opts_clique, clique);

  //check if enough points in clique to detect motion
  //TODO: add parameter for this (all these thresholds should use the same
  //      parameters)
  if(opts_clique.size() < 5) {
    std::cout << "Not enough points in max clique to compute motion!\n";

    //set delta equal to zero
    delta_ =  (cv::Mat_<double>(4,4) <<
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0);

    //reintialize tracker (detect keypoints and add to image structure)
    // std::cout << "processLeftDisparity: reinit stereoDetect 3\n";
    tracker_.stereoDetect(img);

    //store image for next iteration
    img_p_ = img;

    //set status where -1 means VO failed at time step k
    status_ = -1;

    return;
  }

  /////////////////////////////////
  /////////ESTIMATE MOTION/////////
  /////////////////////////////////
  //TODO: use essential matrix for rotation and pnp for translation
  //(instead of pnp for rotation and translation)
  //estimate motion ransac
  // std::cout << "solvePnPRansac\n";
  cv::Mat rvec, tvec, jacobian, I, Vp;  //(img.get_l_info().distortionCoefficientsZeros().cols + 10, 2* opts_clique.size()
  // std::cout << "\n JACsizeInit: " << jacobian.size();
  // std::cout << "\n jacType INIT: " << typeid(jacobian).name();

  std::vector<uchar> inliers_ransac;
  cv::solvePnPRansac(opts_clique,//img_p_.get_l_Image().get_opts(),//tracked_opts_km1_,
                     ipts_clique,//img.get_l_Image().get_ipts(),//tracked_ipts_k_,
                     img.get_l_info().projectionMatrix3x3(),
                     img.get_l_info().distortionCoefficientsZeros(),
                     rvec,
                     tvec,
                     false,
                     200,
                     0.5,
                     0.99,
                     inliers_ransac,
                     1  //CV_EPNP
                     );
  std::cout << "end solvePnPRansac\n";




// *********************************************************************************

  // std::cout << "solvePnP\n";
  // cv::solvePnP(opts_clique,
  //              ipts_clique,
  //              img.get_l_info().projectionMatrix3x3(),
  //              img.get_l_info().distortionCoefficientsZeros(),
  //              rvec,
  //              tvec,
  //              true,
  //              CV_ITERATIVE
  //              );

  // tvec.at<float>(0, 0) = pose.center()[0];
  // tvec.at<float>(0, 1) = pose.center()[1];
  // tvec.at<float>(0, 2) = pose.center()[2];



  // std::cout << "\n jacSize :" << jacobian.size();
  // std::cout << "\n jacType: " << typeid(jacobian).name();
  // std::cout << "\n JT size : " << jacobian_T.size();
  //std::cout << "\n JT type: " << typeid(jacobian_T).name();

  std::cout << "prjectPoints\n";
  cv::projectPoints(opts_clique,
                    rvec,
                    tvec,
                    img.get_l_info().projectionMatrix3x3(),
                    img.get_l_info().distortionCoefficientsZeros(),
                    ipts_clique_out,
                    jacobian,
                    0
                    );

  // double IP_ = (ipts_clique_out.at<double>(0),ipts_clique_out.at<double>(1));

  // std::cout << ipts_clique_out.at<double>(0,1) << std::endl;

  std::vector <cv::Point2f> reprojected;

  int jac_rows = jacobian.rows;
  int jac_cols = jacobian.cols;

  I = cv::Mat::eye(cv::Size(jac_rows,jac_rows), CV_64FC1);

  cv::Mat jacobian_T;
  jacobian_T  = jacobian.t();
 

  std::cout << "\n VP size :" << Vp.size();
  // std::cout << "\n VP type: " << typeid(Vp).name();
  Vp = jacobian_T*I*jacobian;

  //cv::Point2f(ipts_clique_out.at<double>(i, 0), ipts_clique_out.at<double>(i, 1))
  for (int i = 0; i < ipts_clique_out.rows; i++) {
      reprojected.push_back(cv::Point2f(ipts_clique_out.at<double>(i, 0), ipts_clique_out.at<double>(i, 1)));
  }

  Sigma = cv::Mat(Vp, cv::Rect(0, 0, 6, 6)).inv();
  sqrt(Sigma.diag(), std_dev);
  //cv::Mat std_dev_sum;
  std::cout << "\n std: " << typeid(std_dev).name();
  //std::cout << "\n sumType: " << typeid(std_dev_sum).name();
  // std_dev_sum += std_dev;?

  std::cout << "\n *** Sigma: " << Sigma;
  std::cout << "\n *** std_dev: " << std_dev;




  //cv::Mat src; // some input image
  // cv::Mat sigma_new;

  // Convert to double (much faster than a simple for loop)
  // Sigma.convertTo(sigma_new, CV_64F, 1, 0);

  // double *ptrSig[sigma_new.rows];
  // for(int i = 0; i < sigma_new.rows; ++i) {
  //     ptrSig[i] = sigma_new.ptr<double>(i);

  //     for(int j = 0; j < sigma_new.cols; ++j) {
  //         double value = ptrSig[i][j];
  //         std::cout << "\n *** value: " << value;

  //     }
  // }

  //std::cout << "\n *** Dev?? : " << Dev_;

  // std::cout << sqrt(Sigma.diag(), std_dev) << std::endl;


  // *****************************************************************************************

  // std::vector<cv::Point3f> M_;
  // M_ = ipts_clique - opts_clique;
  // std::cout << "M?? = " << M_;



  // std::cout << "\n \ndistCoeff(zeros?) : " << img.get_l_info().distortionCoefficientsZeros();
  // std::cout << "\n jacobian = " << jacobian.size();
  // std::cout << "\n distcoeffSize = " << img.get_l_info().distortionCoefficientsZeros().size();
  // std::cout << "\n opointsSize = " << opts_clique.size();
  // std::cout << "\n iptsJ = " << ipts_clique.size();
  // std::cout << "\n ipts_out = " << ipts_clique_out.size();
  //compute motion (transform from k-1 left to k left)
  delta_ = utils::compose_T(rvec, tvec);
  // delta_ = hack_require_forward_motion(delta_);
  delta_ = hack_require_small_motion(delta_);
  // std::cout << "\n delta_sz : " << delta_.size();
  // std::cout << "\n T_sz : " << T_.size();
  // std::cout << "\n delta type: " << typeid(delta_).name();
  // std::cout << "\n T_type: " << typeid(T_).name();


  //update pose (transform from 0 left to k left)
  T_ = delta_*T_;
  // std::cout << "\n I type: " << typeid(I).name();
  // std::cout << "\n I type: " << typeid(I).name();



  // std::cout << "\n delta_ : " << delta_;
  // std::cout << "\n T_ : " << T_;

  // int jac_rows = jacobian.rows;
  // int jac_cols = jacobian.cols;

  // I = cv::Mat::eye(cv::Size(jac_rows,jac_rows), CV_64FC1);

  // cv::Mat jacobian_T;
  // jacobian_T  = jacobian.t();
 
  // std::cout << "\n VP size :" << Vp.size();
  // std::cout << "\n VP type: " << typeid(Vp).name();

  // std::cout << "\n jacSize :" << jacobian.size();
  // std::cout << "\n jacType: " << typeid(jacobian).name();
  // std::cout << "\n JT size : " << jacobian_T.size();
  // std::cout << "\n JT type: " << typeid(jacobian_T).name();


  // Vp = jacobian_T*I*jacobian;
  // std::cout << "\n Vp size : " << Vp.size();
  // std::cout << "\n Vp : " << Vp;



  /////////////////////////////
  /////////SHOW TRACKS/////////
  /////////////////////////////
  //TODO: add verbosity
  //show tracks after removing outliers
  // cv::Mat img_tracks = utils::show_tracks_on_anaglyph("feature tracks",
  //                                                      img_p_.get_ipts(),
  //                                                      img.get_ipts(),
  //                                                      img_p_.get_l_img(),
  //                                                      img.get_l_img());
  // utils::show_image_points_color("feature tracks",
  //                                img_tracks,
  //                                ipts_clique,
  //                               'y');

  //save previous image for drawing purposes
  img_draw_p_ = img_p_;
  img_draw_   = img;
  ipts_clique_draw_p_ = ipts_clique_p;
  ipts_clique_draw_   = ipts_clique;

  //////////////////////////////
  /////////ADD FEATURES/////////
  //////////////////////////////
  // std::cout << "add stereoDetect\n";
  tracker_.stereoDetect(img);

  ///////////////////////////////////////////////
  /////////SAVE IMAGE FOR NEXT ITERATION/////////
  ///////////////////////////////////////////////
  img_p_  = img;
  status_=0;
  return;
};

/* ------------------------------------------------------------------------ */
std::vector<uchar> StereoVO::
epipolarConstraint(const std::vector<cv::Point2f> & ipts_p,
                   const std::vector<cv::Point2f> & ipts,
                   CameraInfo & info_p,
                   CameraInfo & info)
{
  //compute essential
  std::vector<uchar> inliers;
  cv::findEssentialMat(ipts_p,
                       ipts,
                       info_p.projectionMatrix3x3(),
                       cv::RANSAC,
                       0.99,
                       1.0,
                       inliers);

  return inliers;
}

/* ------------------------------------------------------------------------ */
std::vector<int> StereoVO::
compute_clique(StereoImage img, float delta) {
  //compute adjacency matrix
  int n = img.get_opts().size();
  bool M[n][n];
  int max_deg = -1;
  int max_idx = -1;
  for(int i=0; i<n; i++) {
      int deg = 0;
      for(int j=0; j<n; j++) {
          //do not allow loops
          if(i==j){
              M[i][j] = false;
              continue;
          }

          //distance between i and j at k and k-1
          float d_k   = utils::dist(img.get_opts()[i],
                                    img.get_opts()[j]);
          float d_km1 = utils::dist(img_p_.get_opts()[i],
                                    img_p_.get_opts()[j]);

          //add entry to adjacency matrix
          if(d_k - d_km1 < delta) {
              M[i][j] = true;
              deg++; //increment vertex degree
          }
          else {
              M[i][j] = false;
          }
      }

      //maximum degree vertex, needed for intializing clique
      if(deg > max_deg) {
          max_deg = deg;
          max_idx = i;
      }
  }

  //init clique with max degree vertex
  std::vector<int> clique;
  clique.push_back(max_idx);

  while(1) {
      //find subset compatible vertices
      std::vector<int> v;
      v.clear();
      for(int i=0; i<n; i++) {
          bool is_compatible = true;
          for(const auto & j : clique) {
              if(M[i][j] == false) {
                  is_compatible = false;
                  break;
              }
          }
          if(is_compatible) {
              v.push_back(i);
          }
      }

      if(v.size() == 0) {
          break;
      }

      //add vertex with max adjacent compatible vertices
      max_deg = -1;
      max_idx = -1;
      for(int i=0; i<v.size(); i++) {
          int idx_i = v[i];
          int deg = 0;
          for(int j=0; j<v.size(); j++) {
              int idx_j = v[j];
              if(M[idx_i][idx_j] == true) {
                  deg++;
              }
          }
          if(deg > max_deg) {
              max_deg = deg;
              max_idx = idx_i;
          }
      }
      clique.push_back(max_idx);
  }

  return clique;
}

/* ------------------------------------------------------------------------ */
cv::Mat StereoVO::
hack_require_forward_motion(const cv::Mat & delta) {
    cv::Mat tempT = delta.inv();
    double tx = tempT.at<double>(0,3);
    double ty = tempT.at<double>(1,3);
    double tz = tempT.at<double>(2,3);
    if(tx < tz && ty < tz)
    {
        return delta;
    }
    else
    {
        return cv::Mat::eye(4, 4, CV_64FC1);
    }
}

/* ------------------------------------------------------------------------ */
cv::Mat StereoVO::
hack_require_small_motion(const cv::Mat & delta) {
    cv::Mat tempT = delta.inv();
    double tx = tempT.at<double>(0,3);
    double ty = tempT.at<double>(1,3);
    double tz = tempT.at<double>(2,3);

    double param_max_tx = 0.5;
    double param_max_ty = 0.5;
    double param_max_tz = 0.5;
    double param_min_tx = -0.5;
    double param_min_ty = -0.5;
    double param_min_tz = -0.5;

    if(tx < param_max_tx &&
       ty < param_max_ty &&
       tz < param_max_tz &&
       tx > param_min_tx &&
       ty > param_min_ty &&
       tz > param_min_tz)
    {
        return delta;
    }
    else
    {
        return cv::Mat::eye(4, 4, CV_64FC1);
    }
}

/* ------------------------------------------------------------------------ */
// void _compute_jacobian (const cv::Mat & img)
// {

// }

/* ------------------------------------------------------------------------ */
// void _compute_residuals(const std::vector<cv::Point2f> & kpts)
// {

// }

}
