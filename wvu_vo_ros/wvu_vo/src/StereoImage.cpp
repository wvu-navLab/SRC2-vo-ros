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
		double disparity = l_img_.get_kpts()[i].pt.x 
      - r_img_.get_kpts()[i].pt.x;
		double offset = fabs(l_img_.get_kpts()[i].pt.y 
      - r_img_.get_kpts()[i].pt.y);
    
		uchar is_point_valid = 0;

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
				double cx = (double) 
          l_img_.get_info().getProjectionMatrix().at<double>(0,2);
				double cy = (double) 
          l_img_.get_info().getProjectionMatrix().at<double>(1,2);
				double sx = (double) 
          l_img_.get_info().getProjectionMatrix().at<double>(0,0);
				double sy = (double) 
          l_img_.get_info().getProjectionMatrix().at<double>(1,1);
				double bl = (double) 
          (-r_img_.get_info().getProjectionMatrix().at<double>(0,3)
          /r_img_.get_info().getProjectionMatrix().at<double>(0,0));

				double z = sx/disparity*bl;
				double x = (l_img_.get_kpts()[i].pt.x-cx)/sx*z;
				double y = (l_img_.get_kpts()[i].pt.y-cy)/sy*z;

				l_img_.get_opts().push_back( cv::Point3f(x,y,z) );
				is_point_valid = 1;
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

    if(disparity < 1) { //if disparity < 1 pixel ignore
        return false;
    }

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
