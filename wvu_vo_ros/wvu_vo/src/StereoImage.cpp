// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  StereoImage.cpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         9/14/18
 */
//-----------------------------------------------------------------------------

#include <StereoImage.hpp>

namespace VO
{

/* ------------------------------------------------------------------------ */
// void StereoImage::compute_Q() {
// 				double cx = l_img_.get_info().getProjectionMatrix().at<double>(0,2);
// 				double cy = l_img_.get_info().getProjectionMatrix().at<double>(1,2);
// 				double f  = r_img_.get_info().getProjectionMatrix().at<double>(0,0);
// 				double tx = r_img_.get_info().getProjectionMatrix().at<double>(0,3)/f;

//         //Q_ can be used for converting disparity to 3D points, but not 
//         //is not currently used
//         Q_ = cv::Mat::zeros(cv::Size(4, 4), CV_64FC1);
//         Q_.at<double>(0,0) = 1.0;
//         Q_.at<double>(0,1) = 0.0;
//         Q_.at<double>(0,2) = 0.0;
//         Q_.at<double>(0,3) = -cx;
//         Q_.at<double>(1,0) = 0.0;
//         Q_.at<double>(1,1) = 1.0;
//         Q_.at<double>(1,2) = 0.0;
//         Q_.at<double>(1,3) = -cy;
//         Q_.at<double>(2,0) = 0.0;
//         Q_.at<double>(2,1) = 0.0;
//         Q_.at<double>(2,2) = 0.0;
//         Q_.at<double>(2,3) = f;
//         Q_.at<double>(3,0) = 0.0;
//         Q_.at<double>(3,1) = 0.0;
//         Q_.at<double>(3,2) = -1.0/tx;
//         Q_.at<double>(3,3) = 0.0;
// };

/* ------------------------------------------------------------------------ */
void StereoImage::
sparseMatching() {
	l_img_.get_kpts() = utils::compute_keypoints(l_img_.get_img());
  r_img_.get_kpts() = utils::compute_keypoints(r_img_.get_img());

	utils::compute_subpixel(l_img_.get_img(), l_img_.get_kpts());
	utils::compute_subpixel(r_img_.get_img(), r_img_.get_kpts());

	l_img_.get_desc() = utils::compute_descriptors(l_img_.get_img(), 
                                                 l_img_.get_kpts());
	r_img_.get_desc() = utils::compute_descriptors(r_img_.get_img(), 
                                                 r_img_.get_kpts());

	matches_ = utils::compute_matches(l_img_.get_desc(), r_img_.get_desc());
  
	utils::filter_matches(l_img_.get_kpts(), matches_, 1);
	utils::filter_matches(r_img_.get_kpts(), matches_, 0);
	utils::filter_matches(l_img_.get_desc(), matches_, 1);
	// utils::filter_matches(r_desc_, matches_, 0); //not used typically

	//triangulate detected keypoints
  //TODO: maybe triangulate should require inputs and output vector of
  //      points so that the variables modified are more clear
	triangulate();

};

/* ------------------------------------------------------------------------ */
void StereoImage::
triangulate() {
	std::vector<uchar> valid_points; valid_points.clear();
	l_img_.get_opts().clear();
	for(size_t i=0; i<l_img_.get_kpts().size(); i++)
	{
    // disparity is horizontal only (l_pt.x - r_pt.x) true if rectified ********

// if(i>0)
//   {
//     double prev_disparity = l_img_.get_kpts()[i-1].pt.x
//     - r_img_.get_kpts()[i-1].pt.x;

//     double prev_offset = fabs(l_img_.get_kpts()[i-1].pt.y 
//       - r_img_.get_kpts()[i-1].pt.y);
//   }

		double disparity = l_img_.get_kpts()[i].pt.x 
      - r_img_.get_kpts()[i].pt.x;
    // offset is vertical disparity
		double offset = fabs(l_img_.get_kpts()[i].pt.y 
      - r_img_.get_kpts()[i].pt.y);
    // std::cout << "\n l_img_Y_kpt"<< l_img_.get_kpts()[i].pt.y;
    // std::cout << "\n r_img_Y_kpt"<< r_img_.get_kpts()[i].pt.y;

    
		uchar is_point_valid = 0;
    // std::cout << "\n disparity: " << disparity;
    // std::cout << "\n offset: " << offset;
    // std::cout << "\n ********** TRIANGULATE************";
		//check disparity range 
    //TODO: this should be a parameter for sparse matching
    //      maybe, overload and add as argument to triangulate
		if(disparity > 5 && disparity < 100)
		{
			//check epipolar constraint
      //TODO: this should be a parameter for sparse matching
      //      maybe, overload and add as argument to triangulate
			if(offset < 5 && offset > -5)
			{
        // add loop to add sigma points
				double cx = (double) 
          l_img_.get_info().getProjectionMatrix().at<double>(0,2);
				double cy = (double) 
          l_img_.get_info().getProjectionMatrix().at<double>(1,2);
				double sx = (double) 
          l_img_.get_info().getProjectionMatrix().at<double>(0,0); // sx = sy = focal length
				double sy = (double) 
          l_img_.get_info().getProjectionMatrix().at<double>(1,1);
				double bl = (double) 
          (-r_img_.get_info().getProjectionMatrix().at<double>(0,3)
          /r_img_.get_info().getProjectionMatrix().at<double>(0,0));


        // std::cout <<"\ndisparity old: " << disparity;
				double z = sx/disparity*bl;
				double x = (l_img_.get_kpts()[i].pt.x-cx)/sx*z;
				double y = (l_img_.get_kpts()[i].pt.y-cy)/sy*z;
        // std::cout <<"sx = "<< sx;
        // std::cout <<"bl = "<< bl;
        // point error and matching error approximations (Dellaert's ref uses sigma_m = 0.06)
        double sigma_p = 0.1;
        double sigma_m = 0.1;
        cv::Mat U_i, Jac_p;

        // covariance calculation previous point****************************
        // if(i>0)
        // {


        //   double prev_disparity = l_img_.get_kpts()[i-1].pt.x
        //   - r_img_.get_kpts()[i-1].pt.x;

        //   double prev_offset = fabs(l_img_.get_kpts()[i-1].pt.y 
        //     - r_img_.get_kpts()[i-1].pt.y);

        //   double kpt_ul_p = l_img_.get_kpts()[i-1].pt.x;
        //   double kpt_ur_p = r_img_.get_kpts()[i-1].pt.x;
        //   double kpt_vl_p = l_img_.get_kpts()[i-1].pt.y;
        //   double kpt_vr_p = r_img_.get_kpts()[i-1].pt.y;

        //   Jac_p =  (cv::Mat_<double>(3,3) <<
        //   bl/prev_disparity, 0.0, -(kpt_ul_p*bl)/pow(prev_disparity,2), 
        //   0.0, bl/prev_disparity, -(kpt_vr_p*bl)/pow(prev_disparity,2), 
        //   0.0, 0.0, -(sx*bl)/pow(prev_disparity,2)); 
        //   std::cout << "\n -----prev Jacobian------";       
        //   std::cout << "\n Jac p = " << Jac_p;

        //   // cv::Mat S_i_;
        //   // // covariance of stereo measurements in disparity space: S_i*********
        //   // S_i_ = (cv::Mat_<double>(3,3) <<
        //   //   sigma_p, 0.0, 0.0,
        //   //   0.0, sigma_p, 0.0,
        //   //   0.0, 0.0, sigma_m);

        //   // // Dellaert: covariance matrix of reconstructed 3D point propagation
        //   // cv::Mat P_p_;
        //   // P_p_ = Jac_p*S_i_*Jac_p.t();
        //   // std::cout << "\n +++++++++++++ prev - COVARIANCE++++++++++++++";
        //   // std::cout << "\n Pp = " << P_p_;

        //   // // matthies approx
          
        //   //U_i = Jac_p*Jac_p.t();
        //   //std::cout << "UI ----------- " << typeid(U_i).name();
        
        // }
        


        // covariance calculation current point ___________________________________________________________________
        cv::Mat Jac_, Jac_m;
        double kpt_ul = l_img_.get_kpts()[i].pt.x;
        double kpt_ur = r_img_.get_kpts()[i].pt.x;
        double kpt_vl = l_img_.get_kpts()[i].pt.y;
        double kpt_vr = r_img_.get_kpts()[i].pt.y;
        // std::cout << "sx (fx): "<< sx;
        // std::cout << "sy (fy): "<< sy;

        //jacobian of 3d point
        Jac_ =  (cv::Mat_<double>(3,3) <<
                bl/disparity, 0.0, -(kpt_ul*bl)/pow(disparity,2), 
                0.0, bl/disparity, -(kpt_vr*bl)/pow(disparity,2), 
                0.0, 0.0, -(sx*bl)/pow(disparity,2)); 

        Jac_m = (cv::Mat_<double>(3,4) <<
                -(2*bl*kpt_ur)/pow(disparity,2),  0.0, (2*bl*kpt_ur)/pow(disparity,2), 0.0,
                -(bl*(kpt_vl + kpt_vr)/pow(disparity,2)), bl/(disparity), (bl*(kpt_vl + kpt_vr)/pow(disparity,2)),  bl/(disparity),
                -(2*bl)/pow(disparity,2), 0.0, 2*bl/pow(disparity,2), 0.0);

        // std::cout << "\n -----Jacobian------";       
        // std::cout << "\n Jac OLD = " << Jac_;
        // std::cout << "\n Jac_m = " << Jac_m;

        //std::cout << "\n JacT = " << Jac_.t();



        // Dellaert: covariance matrix of reconstructed 3D point propagation
        cv::Mat V_i, W_i, S_i_;
        S_i_ = (cv::Mat_<double>(3,3) <<
          sigma_p, 0.0, 0.0,
          0.0, sigma_p, 0.0,
          0.0, 0.0, sigma_m);

        //cv::Mat U_i;
        //U_i = Jac_p*Jac_p.t();
        // std::cout << "\njac ----------- " << typeid(S_i_).name();

        // std::cout << "\nSI ----------- " << typeid(S_i_).name();
        // std::cout << "\nSI ----------- " << S_i_;

        //std::cout << "\nj ----------- " << typeid(S_i_).name();

        // std::cout << "\nPI ----------- " << typeid(P_i_).name();
        //std::cout << "VI ----------- " << typeid(V_i).name();

        // cv::Mat P_i_;
        P_i_ = Jac_.t()*S_i_*Jac_;
        // // std::cout << "\n +++++++++++++COVARIANCE++++++++++++++";
        // // std::cout << "\n Pi = " << P_i_;

        // // matthies approx

        // V_i = Jac_*Jac_.t();
        //std::cout << "VI ----------- " << typeid(V_i).name();


        // W_i = (U_i + V_i);//.inv();
        // std::cout << "\n ****U_i = " << U_i;
        // std::cout << "\n ****V_i = " << V_i;
        // std::cout << "\n ****W_i = " << W_i;
        // std::cout << "\n ****P_i = " << P_i_;

        


				l_img_.get_opts().push_back( cv::Point3f(x,y,z) );
				is_point_valid = 1;
        // std::cout <<"\n $$$$$$   P_i_ : " << P_i_;
        // return P_i_;
			}
		}
		valid_points.push_back(is_point_valid);
	}

	//delete outliers in desc and kpts
	utils::delete_outliers(l_img_.get_desc(), valid_points);
	utils::delete_outliers(l_img_.get_kpts(), valid_points);
	// utils::delete_outliers(r_desc_, valid_points); //not typically used
	// utils::delete_outliers(r_kpts_, valid_points); //not typically used  
};

/* ------------------------------------------------------------------------ */
void StereoImage::
denseMatching() {
	//compute disparity using block matcher
	//NOTE: disparity image is of type CV_S16!
	if(bm_params_.use_semi_global) {
		cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(bm_params_.min_disparity, bm_params_.num_disparities, bm_params_.block_size);
		sgbm->setBlockSize        (bm_params_.block_size);
		sgbm->setNumDisparities   (bm_params_.num_disparities);
		sgbm->setMinDisparity     (bm_params_.min_disparity);
		sgbm->setDisp12MaxDiff    (bm_params_.max_disparity);
		sgbm->setUniquenessRatio  (bm_params_.uniqueness_ratio);
		sgbm->setSpeckleWindowSize(bm_params_.speckle_window_size);
		sgbm->setSpeckleRange     (bm_params_.speckle_range);
    sgbm->setPreFilterCap     (bm_params_.prefilter_cap);
		sgbm->setP1               (120);//(bm_params_.block_size*bm_params_.block_size*8);
		sgbm->setP2               (240);//(bm_params_.block_size*bm_params_.block_size*32);
		sgbm->compute(l_img_.get_img(), r_img_.get_img(), disp_);
	}
	else {
		cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create();
		bm->setBlockSize        (bm_params_.block_size);
		bm->setNumDisparities   (bm_params_.num_disparities);
		bm->setMinDisparity     (bm_params_.min_disparity);
		bm->setDisp12MaxDiff    (bm_params_.max_disparity);
		bm->setUniquenessRatio  (bm_params_.uniqueness_ratio);
		bm->setSpeckleWindowSize(bm_params_.speckle_window_size);
		bm->setSpeckleRange     (bm_params_.speckle_range);
		bm->setPreFilterCap     (bm_params_.prefilter_cap);
		bm->setPreFilterSize    (bm_params_.prefilter_size);
		bm->compute(l_img_.get_img(), r_img_.get_img(), disp_);
	}

  //add disparity to image structure
  // l_img_.get_disp() = disp_;

	//triangulate (not used, instead 3D points computed only for tracks)
	// cv::reprojectImageTo3D(disp_, img3d_, Q_, true, CV_32F);

	//images (rescale only for displaying disparity map)
	cv::Mat disp8;
	disp_.convertTo(disp8, CV_8U, 1.0/(bm_params_.num_disparities*16)*256.0);
	// utils::show_image("disparity", disp8);

};

/* ------------------------------------------------------------------------ */
void StereoImage::
setBlockMatcherParams(const int & block_size,
                			const int & num_disparities,
                			const int & min_disparity,
                			const int & max_disparity,
                			const int & uniqueness_ratio,
                			const int & speckle_window_size,
                			const int & speckle_range,
                			const int & prefilter_cap,
                			const int & prefilter_size,
                			const int & use_semi_global) {
  bm_params_.block_size          = block_size;
  bm_params_.num_disparities     = num_disparities;
  bm_params_.min_disparity       = min_disparity;
  bm_params_.max_disparity       = max_disparity;
  bm_params_.uniqueness_ratio    = uniqueness_ratio;
  bm_params_.speckle_window_size = speckle_window_size;
  bm_params_.speckle_range       = speckle_range;
  bm_params_.prefilter_cap       = prefilter_cap;
  bm_params_.prefilter_size      = prefilter_size;
  bm_params_.use_semi_global     = use_semi_global;
  bm_params_.empty               = false;
}

/* ------------------------------------------------------------------------ */
bool StereoImage::
ipt_to_opt(const cv::Point2f & ipt, 
                 cv::Point3f & opt) {
    //check that image exists
    if(!disp_.data) {
        std::cout << "Error! Must compute disparity before object points!";
        exit(1);
    }

    //check if point is inside image
    if(ipt.y > disp_.rows - 1 || 
        ipt.y < 0 || 
        ipt.x > disp_.cols || 
        ipt.x < 0) {
        return false;
    }

    //get disparity (carefule with disparity type! CV_S16 is expected!)
    double disparity = disp_.at<short>(ipt.y, ipt.x)/16;
    // std::cout << "\ndisparity: " << disparity;

    if(disparity < 1) { //if disparity < 1 pixel ignore
        return false;
    }

    // get left and right ipts (image coord)
    std::vector<cv::Point2f> ipts_clique_l = l_img_.get_ipts();
    std::vector<cv::Point2f> ipts_clique_r = r_img_.get_ipts_r();

    // std::cout << "\nipts_L: " << ipts_clique_l;
    // std::cout << "\nipts_R: " << ipts_clique_r;


    //compute 3D point from disparity
    double cx = (double) l_img_.get_info().getProjectionMatrix().at<double>(0,2);
    double cy = (double) l_img_.get_info().getProjectionMatrix().at<double>(1,2);
    double sx = (double) l_img_.get_info().getProjectionMatrix().at<double>(0,0);
    double sy = (double) l_img_.get_info().getProjectionMatrix().at<double>(1,1);
    double bl = (double) (-r_img_.get_info().getProjectionMatrix().at<double>(0,3)
        /r_img_.get_info().getProjectionMatrix().at<double>(0,0));

    opt.z = sx/disparity*bl;
    opt.x = (ipt.x-cx)/sx*opt.z;
    opt.y = (ipt.y-cy)/sy*opt.z;
    // std::cout << "z = " << opt.z;
    // std::cout << "x = " << opt.zx;
    // std::cout << "y = " << y;



    if(opt.z > 1000) { 
        return false; //return false if no disparity
    }
    else {
        return true; //return true if disparity exists
    }
};

/* ------------------------------------------------------------------------ */
void StereoImage::
tuneAndSetBlockMatcherParams(int & tuner_scale,
                             int & block_size,
                             int & num_disparities,
                             int & min_disparity,
                             int & max_disparity,
                             int & uniqueness_ratio,
                             int & speckle_window_size,
                             int & speckle_range,
                             int & prefilter_cap,
                             int & prefilter_size,
                             int & use_semi_global) {
    tuner_scale_ = tuner_scale;
    setBlockMatcherParams(block_size,
                          num_disparities,
                          min_disparity,
                          max_disparity,
                          uniqueness_ratio,
                          speckle_window_size,
                          speckle_range,
                          prefilter_cap,
                          prefilter_size,
                          use_semi_global);
    tune_block_matcher();
    tuner_scale         = tuner_scale_;
    block_size          = bm_params_.block_size;
    num_disparities     = bm_params_.num_disparities;
    min_disparity       = bm_params_.min_disparity;
    max_disparity       = bm_params_.max_disparity;
    uniqueness_ratio    = bm_params_.uniqueness_ratio;
    speckle_window_size = bm_params_.speckle_window_size;
    speckle_range       = bm_params_.speckle_range;
    prefilter_cap       = bm_params_.prefilter_cap;
    prefilter_size      = bm_params_.prefilter_size;
    use_semi_global     = bm_params_.use_semi_global;
};

/* ------------------------------------------------------------------------ */
void StereoImage::
tune_block_matcher() {
    while(1) {
        //compute disparity (computed to ensure latest parameters applied)
        denseMatching();

        //display image (disparity = disp_.at<short>(y,x)/16.0)
        //so rescaling we have disparity*256/number_of_disparities
        //then multiply by scale to increase brightness of image
        cv::Mat disp8;
        disp_.convertTo(disp8, CV_8U, (double)tuner_scale_/100.0 
            *  1.0/16.0 * 256.0/(double)bm_params_.num_disparities);
        // utils::show_image("disparity", disp8);
        utils::show_stereo("left image, disparity", l_img_.get_img(), disp8);

        //trackbars
        static int temp_num_disparities = bm_params_.num_disparities;
        static int temp_pre_filter_cap = bm_params_.prefilter_cap;
        modify_param("left image, disparity", "tuner_scale_*100",    
            &tuner_scale_,                   10000);
        modify_param("left image, disparity", "block_size",          
            &bm_params_.block_size,          21);
        modify_param("left image, disparity", "num_disparities",     
            &temp_num_disparities,           256);
        modify_param("left image, disparity", "min_disparity",       
            &bm_params_.min_disparity,       100);
        modify_param("left image, disparity", "max_disparity",       
            &bm_params_.max_disparity,       100);
        modify_param("left image, disparity", "uniqueness_ratio",    
            &bm_params_.uniqueness_ratio,    50);
        modify_param("left image, disparity", "speckle_window_size", 
            &bm_params_.speckle_window_size, 750);
        modify_param("left image, disparity", "speckle_range",       
            &bm_params_.speckle_range,       3);
        modify_param("left image, disparity", "prefilter_cap",       
            &temp_pre_filter_cap,            100);
        modify_param("left image, disparity", "prefilter_size",      
            &bm_params_.prefilter_size,      100);
        modify_param("left image, disparity", "use_semi_global",     
            &bm_params_.use_semi_global,     1);

        if(temp_pre_filter_cap == 0) {
            temp_pre_filter_cap = -1; //if 0, set as -1 to turn off
        }
        bm_params_.prefilter_cap = temp_pre_filter_cap;

        //num_disparities must be divisible by 16 (required)
        if(temp_num_disparities < 16) temp_num_disparities = 16;
        int q = temp_num_disparities / 16;
        temp_num_disparities = q*16;
        bm_params_.num_disparities = temp_num_disparities;

        //break if pressing ENTER, ESC, or SPACE
        char keypress = cv::waitKey(1);
        if(keypress==27 || keypress==32 || keypress==13) {
            break;
        }
    }
};

/* ------------------------------------------------------------------------ */
void StereoImage::
modify_param(std::string window_name, 
             std::string param_name,
             int *       param_pointer,
             int         param_max_value) {
    cv::createTrackbar(param_name, 
                       window_name, 
                       param_pointer, 
                       param_max_value, 
                       NULL);
};

/* ------------------------------------------------------------------------ */
bool StereoImage::
is_disparity_computed() {
  if(disp_.data) {
    return true;
  }
  else {
    return false;
  }
};

}
