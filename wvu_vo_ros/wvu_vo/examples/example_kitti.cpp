// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  example.cpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         9/28/18
 */
//-----------------------------------------------------------------------------

#include <CameraInfo.hpp>
#include <opencv_utils.hpp>
#include <StereoVO.hpp>

//usage 
void help()
{
  std::cout << "usage1: ./demo_vo [calibration folder] [image folder]" << std::endl;
  std::cout << "usage2: ./demo_vo [calibration folder] [image folder] [truth folder]" << std::endl;
  std::cout << "usage3: ./demo_vo [data folder]" << std::endl;
  std::cout << "\t [calibration folder] path to folder containing calibration files" << std::endl;
  std::cout << "\t [image folder] path to folder containing images" << std::endl;
  std::cout << "\t [truth folder] path to folder containing truth data" << std::endl;
  std::cout << "\t [data folder] path to folder containing calib, image, and poses folders" << std::endl;
  std::cout << std::endl << "demo_vo: error. invalid arguments." << std::endl;
};

//fns
void get_args(int argc, char** argv);
bool get_lr_kitti(int index);
void display_trajectory(std::vector<double> xyz);
void load_truth();

//vars
bool is_truth_provided_;
std::string calib_path_ = "";
std::string image_path_ = "";
std::string truth_path_ = "";
VO::StereoImage stereo_img_;
VO::CameraInfo l_info_;;
VO::CameraInfo r_info_;
std::vector<std::vector<double> > _xyz_truth;

int main( int argc, char** argv )
{
  std::cout << "Running kitti example..." << std::endl;

  //args
  get_args(argc, argv);

  //calib
  l_info_ = VO::CameraInfo(calib_path_ + "/left.yaml");
  r_info_ = VO::CameraInfo(calib_path_ + "/right.yaml");
  l_info_.loadYAML();
  r_info_.loadYAML();

  //vo
  // VO::StereoVO vo(VO::StereoVO::LEFT_RIGHT);
  VO::StereoVO vo(VO::StereoVO::LEFT_DISPARITY);

  VO::GFTTParameters gftt_params;
  gftt_params.max_features        = 2000;
  gftt_params.quality             = 0.001;
  gftt_params.min_distance        = 10;
  gftt_params.block_size          = 5;
  gftt_params.use_harris_detector = false;
  gftt_params.k                   = 0.04;
  gftt_params.empty               = false;
  vo.setGFTTParameters(gftt_params);

  VO::KLTParameters klt_params;
  klt_params.max_iter  = 30;
  klt_params.eps       = 0.01;
  klt_params.window    = 20;
  klt_params.max_level = 4;
  klt_params.max_age   = 10;
  klt_params.empty     = false;
  vo.setKLTParameters(klt_params);

  //print details
  std::cout << "i, "
            << "error, "
            << "percent, "
            << "x, "
            << "y, "
            << "z, "
            << "roll, "
            << "pitch, "
            << "yaw"
            << std::endl;

  //loop
  int i=0;
  double d=0;
  while(1)
  {
    //imgs
    if(!get_lr_kitti(i))
    {
      std::cout << "Image data is empty." << std::endl;
      cv::waitKey(0);
      break;
    }

    //print first image info
    if(i==0) {
      stereo_img_.print();
    }

    //set block matcher parameters
    stereo_img_.setBlockMatcherParams(15,    //block_size
                                      64,    //num_disparities
                                      0,     //min_disparity
                                      12,    //max_disparity
                                      32,    //uniquness_ratio
                                      200,   //speckle_window
                                      2,     //speckle_range
                                      0,     //prefilter_cap (not used for SGBM)
                                      0,     //prefilter_size not used for SGBM)
                                      1);    //use_semi_global
    // stereo_img_.tune_block_matcher();

    //process vo (left/right)
    vo.process(stereo_img_);

    //total distance traveled
    if(i>0)
    {
      double dz = _xyz_truth[i][2] - _xyz_truth[i-1][2];
      double dx = _xyz_truth[i][0] - _xyz_truth[i-1][0];
      d = d + std::sqrt(dz*dz + dx*dx);
    }

    //print output
    if(is_truth_provided_) {
      std::vector<double> x_est = utils::get_xyz(vo.get_T_02k());
      std::vector<double> x_tru = _xyz_truth[i];
      double error = utils::dist(x_est, x_tru);

      std::cout << i << ","
                << error << ","
                << 100*error/d << "%, "
                << utils::get_xyz(vo.get_T_02k())[0] << ","
                << utils::get_xyz(vo.get_T_02k())[1] << ","
                << utils::get_xyz(vo.get_T_02k())[2] << ","
                << utils::get_rpy(vo.get_T_02k())[0]*180.0/3.14159265 << ","
                << utils::get_rpy(vo.get_T_02k())[1]*180.0/3.14159265 << ","
                << utils::get_rpy(vo.get_T_02k())[2]*180.0/3.14159265 << ","
                << std::endl;
    }
    else {
      std::cout << i << ","
                << utils::get_xyz(vo.get_T_02k())[0] << ","
                << utils::get_xyz(vo.get_T_02k())[1] << ","
                << utils::get_xyz(vo.get_T_02k())[2] << ","
                << utils::get_rpy(vo.get_T_02k())[0]*180.0/3.14159265 << ","
                << utils::get_rpy(vo.get_T_02k())[1]*180.0/3.14159265 << ","
                << utils::get_rpy(vo.get_T_02k())[2]*180.0/3.14159265 << ","
                << std::endl;
    }

    //plot result
    // display_trajectory(utils::get_xyz(vo.get_T_02k()));
             
    //image counter
    i++;
    cv::waitKey(1);
  }

  std::cout << "Terminating..." << std::endl;
  return 0;
}

void get_args(int argc, char** argv)
{
  //ground truth
  if(argc == 4)
  {
    is_truth_provided_ = true;
    calib_path_ = argv[1];
    image_path_ = argv[2];
    truth_path_ = argv[3];
    load_truth();
  }
  else if(argc == 3) {
    is_truth_provided_ = false;
    calib_path_ = argv[1];
    image_path_ = argv[2];
    truth_path_ = "";
  }
  else if(argc == 2) {
    is_truth_provided_ = true;
    std::string filepath = argv[1];
    calib_path_ = filepath + "/calib";
    image_path_ = filepath + "/images";
    truth_path_ = filepath + "/poses";
    load_truth();
  }
  else
  {
    std::cout << "Invalid arguments" << std::endl;
    help();
    exit(1);
  }
  std::cout << "get_args: calib_path_ = " << calib_path_ << std::endl;
  std::cout << "get_args: image_path_ = " << image_path_ << std::endl;
  std::cout << "get_args: truth_path_ = " << truth_path_ << std::endl;
}

bool get_lr_kitti(int index)
{
  //digits 
  int digits = (index < 10 ? 1 :   
      (index < 100 ? 2 :   
      (index < 1000 ? 3 :   
      (index < 10000 ? 4 :   
      (index < 100000 ? 5 :   
      (index < 1000000 ? 6 :   
      (index < 10000000 ? 7 :  
      (index < 100000000 ? 8 :  
      (index < 1000000000 ? 9 :  
      10)))))))));  

  //filename
  std::string file = std::string(6 - digits, '0').append( std::to_string(index) );

  //imgs 
  cv::Mat img_l = cv::imread(image_path_ + "/image_0/" + file + ".png", CV_8UC1);
  cv::Mat img_r = cv::imread(image_path_ + "/image_1/" + file + ".png", CV_8UC1);

  if(!img_l.data || !img_r.data) {
    return false;
  }

  stereo_img_ = VO::StereoImage(img_l, img_r, l_info_, r_info_);
  // stereo_img_.print();

  return true;
}

void display_trajectory(std::vector<double> xyz)
{
  //size of plot
  int w  = 2000;
  int h  = 2000;
  int r  = 5;
  int cx = w/4;
  int cy = h/2;

  //scale (pixels per meter)
  double s = 2;

  //image for plotting
  static cv::Mat plot(h,w,CV_8UC3, cv::Scalar(255,255,255));
  static bool init_plot = false;

  //plot ground truth position
  if(!init_plot)
  {
    for(int i=0; i<_xyz_truth.size(); i++)
    {
      cv::circle(plot, cv::Point(cx + _xyz_truth[i][2]*s, cy +  _xyz_truth[i][0]*s), r, cv::Scalar(0,255,0),CV_FILLED);
    }
  }
  init_plot = true;

  //plot estimated position
  cv::circle(plot, cv::Point(cx + xyz[2]*s, cy + xyz[0]*s), r, cv::Scalar(255,0,0), CV_FILLED);

  //plot window
  cv::namedWindow("trajectory", cv::WINDOW_NORMAL);
  cv::imshow("trajectory", plot);

}

void load_truth()
{
  //get path to lookup table
    std::string filepath = truth_path_ + "/00.txt";

    //open file containing lookup table
    std::ifstream inputFile;
    inputFile.open(filepath.c_str());
    if(!inputFile)
    {
        std::cout << "Error! Failed to open file " << filepath.c_str() << "!" << std::endl;
        return;
    }
    else
    {
      std::cout << "File " << filepath.c_str() << " open success!" << std::endl;
    }

    //load data in the lookup table
    _xyz_truth.clear();
    while(1)
    {
      double element = 0;
      if(inputFile.eof())
      {
          std::cout << "End of file while accessing element in" << filepath <<  "." << std::endl;
          return;
      }
      else
      {   //extract line of data (each line has 12 elements)
          inputFile >> element;
          inputFile >> element;
          inputFile >> element;
          inputFile >> element;
          double x = element;
          inputFile >> element;
          inputFile >> element;
          inputFile >> element;
          inputFile >> element;
          double y = element;
          inputFile >> element;
          inputFile >> element;
          inputFile >> element;
          inputFile >> element;
          double z = element;
          
          //push back
          std::vector<double> xyz;
          xyz.push_back(x);
          xyz.push_back(y);
          xyz.push_back(z);
          _xyz_truth.push_back(xyz);
      }
    }

    //close file
    inputFile.close();

    return;
}