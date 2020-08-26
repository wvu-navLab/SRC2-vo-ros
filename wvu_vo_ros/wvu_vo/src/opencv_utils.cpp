  // -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  opencv_utils.hpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         9/14/18
 */
//-----------------------------------------------------------------------------

#include <opencv_utils.hpp>

namespace utils
{


/****************************************************************************
                                OpenCV Types 
*****************************************************************************/
//get cv::Mat type
void print_Mat_info(const cv::Mat & img) {
  std::string str_type, str_access;
  switch (img.type()) {
    case CV_8U:  
      str_type = "8U"; 
      str_access = "img.at<uchar>(y,x)";
      break;
    case CV_8S:  
      str_type = "8S"; 
      str_access = "img.at<schar>(y,x)";
      break;
    case CV_16U: 
      str_type = "16U"; 
      str_access = "img.at<ushort>(y,x)";
      break;
    case CV_16S: 
      str_type = "16S"; 
      str_access = "img.at<short>(y,x)";
      break;
    case CV_32S: 
      str_type = "32S"; 
      str_access = "img.at<int>(y,x)";
      break;
    case CV_32F: 
      str_type = "32F"; 
      str_access = "img.at<float>(y,x)";
      break;
    case CV_64F: 
      str_type = "64F"; 
      str_access = "img.at<double>(y,x)";
      break;
    default:     
      str_type = "UNKNOWN"; 
      str_access = "img.at<UNKNOWN>(y,x)";
      break;
  }

  std::cout << "img is type = " << str_type << " and accessed with " 
            << str_access << std::endl;
  return;
}


/****************************************************************************
                                Visualization
*****************************************************************************/
//----------------------------------------------------------------------------
/** 
 * Displays image
 */
void show_image   (       std::string   title,
                    const cv::Mat     & img)
{
  cv::namedWindow(title, CV_WINDOW_NORMAL);
  imshow(title, img);
}
//----------------------------------------------------------------------------
/** 
 * Displays stereo pair of images
 */
void show_stereo  (       std::string   title,
                    const cv::Mat     & img_l,
                    const cv::Mat     & img_r)
{
  assert(img_l.cols == img_r.cols && img_l.rows == img_r.rows);
  cv::Mat dst(cv::Size(img_l.cols*2, img_l.rows), 
              img_l.type(), 
              cv::Scalar::all(0));
  cv::Mat left(dst, cv::Rect(0,0,img_l.cols,img_l.rows));
  img_l.copyTo(left); 
  cv::Mat right(dst, cv::Rect(img_l.cols, 0, img_r.cols, img_r.rows));
  img_r.copyTo(right);
  // for(int i=0; i<img_l.rows; i=i+20) //draw epipolar lines
  //   cv::line(dst, cv::Point(0,i), cv::Point(img_l.cols*2,i), cv::Scalar(255), 1, 8, 0);
  cv::namedWindow(title, CV_WINDOW_NORMAL);
  imshow(title, dst);
}

//----------------------------------------------------------------------------
/** 
 * Displays stereo pair of images
 */
void show_anaglyph  (       std::string   title,
                      const cv::Mat     & img_l,
                      const cv::Mat     & img_r)
{
  assert(img_l.cols == img_r.cols && img_l.rows == img_r.rows);

  cv::Mat channelR = img_l.clone();
  cv::Mat channelB = img_r.clone();
  cv::Mat channelG;
  cv::add(channelR/2, channelB/2, channelG);
  
  std::vector<cv::Mat> channels{channelR, channelG, channelB};

  cv::Mat anaglyph;
  cv::merge(channels, anaglyph);

  cv::namedWindow(title, CV_WINDOW_NORMAL);
  imshow(title, anaglyph);
}

//----------------------------------------------------------------------------
/** 
 * Displays keypoints in image
 */
void show_keypoints(      std::string   title,
                    const cv::Mat     & img,
                    const v_kpt       & kpts)
{
  cv::Mat display;
  drawKeypoints(img, kpts, display, cv::Scalar(0,255,0));
  cv::namedWindow(title, CV_WINDOW_NORMAL);
  imshow(title, display);
}

void show_image_points(      std::string   title,
                       const cv::Mat     & img,
                       const v_pt2f      & ipts)
{
  v_kpt kpts;
  cv::KeyPoint::convert(ipts, kpts);

  cv::Mat display;
  drawKeypoints(img, kpts, display, cv::Scalar(0,255,0));
  cv::namedWindow(title, CV_WINDOW_NORMAL);
  imshow(title, display);
}

cv::Mat show_image_points_color(      std::string   title,
                                const cv::Mat     & img,
                                const v_pt2f      & ipts,
                                      char          color)
{
  v_kpt kpts;
  cv::KeyPoint::convert(ipts, kpts);

  cv::Scalar scalar;
  switch (color) {
      case 'b':  
        scalar = cv::Scalar(255,0,0);
        break;
      case 'g':  
        scalar = cv::Scalar(0,255,0);
        break;
      case 'r': 
        scalar = cv::Scalar(0,0,255);
        break;
      case 'w': 
        scalar = cv::Scalar(255,255,255);
        break;
      case 'c': 
        scalar = cv::Scalar(255,255,0);
        break;
      case 'y': 
        scalar = cv::Scalar(0,255,255);
        break;
      case 'm': 
        scalar = cv::Scalar(255,0,255);
        break;
      case 'k': 
        scalar = cv::Scalar(0,0,0);
        break;
    }

  cv::Mat display;
  drawKeypoints(img, kpts, display, scalar);
  cv::namedWindow(title, CV_WINDOW_NORMAL);
  imshow(title, display);

  return display;
}

//----------------------------------------------------------------------------
/** 
 * Display tracked features in image (i is previous, j is current)
 */
void show_tracks   (      std::string   title,
                    const v_pt2f      & kpts_i,
                    const v_pt2f      & kpts_j,
                    const cv::Mat     & img_j)
{
  cv::Mat grey = img_j.clone();
  cv::Mat display;
  cv::cvtColor(grey, display, cv::COLOR_GRAY2BGR);
  for(int i=0; i <kpts_j.size(); i++)
  {
    circle(display, kpts_j[i], 3, cv::Scalar(0,255,0), 1, 8, 0);
    line(display, kpts_j[i], kpts_i[i], cv::Scalar(0,255,0), 1, 8, 0);
  }
  cv::namedWindow(title, CV_WINDOW_NORMAL);
  imshow(title, display);
}

void show_tracks   (      std::string   title,
                    const v_kpt       & kpts_i,
                    const v_kpt       & kpts_j,
                    const cv::Mat     & img_j)
{
  cv::Mat grey = img_j.clone();
  cv::Mat display;
  cv::cvtColor(grey, display, cv::COLOR_GRAY2BGR);
  for(int i=0; i <kpts_j.size(); i++)
  {
    circle(display, kpts_j[i].pt, 3, cv::Scalar(0,255,0), 1, 8, 0);
    line(display, kpts_j[i].pt, kpts_i[i].pt, cv::Scalar(0,255,0), 1, 8, 0);
  }
  cv::namedWindow(title, CV_WINDOW_NORMAL);
  imshow(title, display);
}

//----------------------------------------------------------------------------
/** 
 * Display tracked features in anaglyph of images where
 * (i is previous, j is current)
 * 
 */
cv::Mat show_tracks_on_anaglyph   (      std::string   title,
                                   const v_pt2f      & kpts_i,
                                   const v_pt2f      & kpts_j,
                                   const cv::Mat     & img_i,
                                   const cv::Mat     & img_j,
                                   int suppress)
{
  assert(img_i.cols == img_j.cols && img_i.rows == img_j.rows);

  cv::Mat channelR = img_i.clone();
  cv::Mat channelB = img_j.clone();
  cv::Mat channelG;
  cv::add(channelR/2, channelB/2, channelG);
  
  std::vector<cv::Mat> channels{channelR, channelG, channelB};

  cv::Mat anaglyph;
  cv::merge(channels, anaglyph);

  cv::Mat display = anaglyph.clone();
  // cv::Mat display;
  // cv::cvtColor(grey, display, cv::COLOR_GRAY2BGR);
  for(int i=0; i <kpts_j.size(); i++)
  {
    circle(display, kpts_j[i], 3, cv::Scalar(0,255,0), 1, 8, 0);
    line(display, kpts_j[i], kpts_i[i], cv::Scalar(0,255,0), 1, 8, 0);
  }

  if(suppress != 1)
  {
    cv::namedWindow(title, CV_WINDOW_NORMAL);
    imshow(title, display);
  }

  return display;
}

cv::Mat show_tracks_on_anaglyph   (      std::string   title,
                                   const v_kpt       & kpts_i,
                                   const v_kpt       & kpts_j,
                                   const cv::Mat     & img_i,
                                   const cv::Mat     & img_j,
                                   int suppress)
{
  assert(img_i.cols == img_j.cols && img_i.rows == img_j.rows);

  cv::Mat channelR = img_i.clone();
  cv::Mat channelB = img_j.clone();
  cv::Mat channelG;
  cv::add(channelR/2, channelB/2, channelG);
  
  std::vector<cv::Mat> channels{channelR, channelG, channelB};

  cv::Mat anaglyph;
  cv::merge(channels, anaglyph);

  cv::Mat grey = anaglyph.clone();
  cv::Mat display;
  cv::cvtColor(grey, display, cv::COLOR_GRAY2BGR);
  for(int i=0; i <kpts_j.size(); i++)
  {
    circle(display, kpts_j[i].pt, 3, cv::Scalar(0,255,0), 1, 8, 0);
    line(display, kpts_j[i].pt, kpts_i[i].pt, cv::Scalar(0,255,0), 1, 8, 0);
  }

  if(suppress != 1)
  {
    cv::namedWindow(title, CV_WINDOW_NORMAL);
    imshow(title, display);
  }

  return display;
}

//----------------------------------------------------------------------------
/** 
 * Save image of tracked features on anaglyph where
 * (i is previous, j is current)
 * 
 */
void save_tracks_on_anaglyph   (      std::string   title,
                                const v_pt2f      & kpts_i,
                                const v_pt2f      & kpts_j,
                                const cv::Mat     & img_i,
                                const cv::Mat     & img_j)
{
  assert(img_i.cols == img_j.cols && img_i.rows == img_j.rows);

  cv::Mat channelR = img_i.clone();
  cv::Mat channelB = img_j.clone();
  cv::Mat channelG;
  cv::add(channelR/2, channelB/2, channelG);
  
  std::vector<cv::Mat> channels{channelR, channelG, channelB};

  cv::Mat anaglyph;
  cv::merge(channels, anaglyph);

  cv::Mat display = anaglyph.clone();
  // cv::Mat display;
  // cv::cvtColor(grey, display, cv::COLOR_GRAY2BGR);
  for(int i=0; i <kpts_j.size(); i++)
  {
    circle(display, kpts_j[i], 3, cv::Scalar(0,255,0), 1, 8, 0);
    line(display, kpts_j[i], kpts_i[i], cv::Scalar(0,255,0), 1, 8, 0);
  }

  cv::imwrite(title.c_str(), display);
}

void save_tracks_on_anaglyph   (      std::string   title,
                                const v_kpt       & kpts_i,
                                const v_kpt       & kpts_j,
                                const cv::Mat     & img_i,
                                const cv::Mat     & img_j)
{
  assert(img_i.cols == img_j.cols && img_i.rows == img_j.rows);

  cv::Mat channelR = img_i.clone();
  cv::Mat channelB = img_j.clone();
  cv::Mat channelG;
  cv::add(channelR/2, channelB/2, channelG);
  
  std::vector<cv::Mat> channels{channelR, channelG, channelB};

  cv::Mat anaglyph;
  cv::merge(channels, anaglyph);

  cv::Mat grey = anaglyph.clone();
  cv::Mat display;
  cv::cvtColor(grey, display, cv::COLOR_GRAY2BGR);
  for(int i=0; i <kpts_j.size(); i++)
  {
    circle(display, kpts_j[i].pt, 3, cv::Scalar(0,255,0), 1, 8, 0);
    line(display, kpts_j[i].pt, kpts_i[i].pt, cv::Scalar(0,255,0), 1, 8, 0);
  }
  
  cv::imwrite(title.c_str(), display);
}

//----------------------------------------------------------------------------
/** 
 * Display matched features between image i and image j
 */
void show_matches  (      std::string   title,
                    const cv::Mat     & img_i, 
                    const cv::Mat     & img_j, 
                    const v_kpt       & kp_i, 
                    const v_kpt       & kp_j, 
                    const v_m         & matches)
{
  cv::Mat display;
  drawMatches(img_i, 
              kp_i, 
              img_j, 
              kp_j, 
              matches, 
              display, 
              cv::Scalar::all(-1), 
              cv::Scalar::all(-1), 
              std::vector<char>(), 
              cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  cv::namedWindow("matches", CV_WINDOW_NORMAL);
  imshow("matches", display);
}

//----------------------------------------------------------------------------
/** 
 * Write .ply file given xyz image and rgb image where xyz is CV_32F
 */
void write_ply_xyzrgb(      std::string   filename,
                      const v_pt3f      & pts3f)
{
    std::ofstream filestream;
    filestream.open(filename, std::ofstream::out | std::ofstream::trunc);

    filestream << "ply" << std::endl <<
    "format ascii 1.0" << std::endl <<
    "comment file created using code by a robot" << std::endl <<
    "element vertex " << pts3f.size() << std::endl <<
    "property float x" << std::endl <<
    "property float y" << std::endl <<
    "property float z" << std::endl <<
    "end_header" << std::endl;

    for(const auto & pt3f : pts3f) {
      filestream << pt3f.x << " "
                 << pt3f.y << " "
                 << pt3f.z << " "
                 << std::endl;
    }

    filestream.close();
};


/****************************************************************************
                            Image Processing
*****************************************************************************/
//----------------------------------------------------------------------------
/** 
 * Sharpen image
 */
void sharpen(cv::Mat & img)
{
  cv::Mat kernel = (cv::Mat_<float>(3,3) << 
                     0.0, -1.0, -0.0,
                    -1.0,  5.0, -1.0,
                     0.0, -1.0, -0.0);
  cv::filter2D(img, img, -1, kernel);
}

void smooth(cv::Mat & img)
{
  cv::Mat temp;
  cv::bilateralFilter(img, temp, 7, 15, 15);
  img = temp.clone();
}

//----------------------------------------------------------------------------
/** 
 * Compute dx of image using prewitt filter
 */
void compute_gradient_x(cv::Mat & img)
{
  cv::Mat kernel = (cv::Mat_<float>(3,3) << 
                     1.0, 0.0, -1.0,
                     1.0, 0.0, -1.0,
                     1.0, 0.0, -1.0);
  cv::filter2D(img, img, -1, kernel); 
}

//----------------------------------------------------------------------------
/** 
 * Compute dy of image using prewitt filter 
 */
void compute_gradient_y(cv::Mat & img)
{
  cv::Mat kernel = (cv::Mat_<float>(3,3) << 
                     1.0,  1.0,  1.0,
                     0.0,  0.0,  0.0,
                    -1.0, -1.0, -1.0);
  cv::filter2D(img, img, -1, kernel);
}

//----------------------------------------------------------------------------
/** 
 * Apply blob filter to image
 */
void filter_blobs(cv::Mat & img)
{
  cv::Mat kernel = (cv::Mat_<float>(5,5) << 
                    -1.0, -1.0, -1.0, -1.0, -1.0,
                    -1.0,  1.0,  1.0,  1.0, -1.0,
                    -1.0,  1.0,  8.0,  1.0, -1.0,
                    -1.0,  1.0,  1.0,  1.0, -1.0,
                    -1.0, -1.0, -1.0, -1.0, -1.0);
  cv::filter2D(img, img, -1, kernel);
}

//----------------------------------------------------------------------------
/** 
 * Apply corner filter to image
 */
void filter_corners(cv::Mat & img)
{
  cv::Mat kernel = (cv::Mat_<float>(5,5) << 
                    -1.0, -1.0,  0.0,  1.0,  1.0,
                    -1.0, -1.0,  0.0,  1.0,  1.0,
                     0.0,  0.0,  0.0,  0.0,  0.0,
                     1.0,  1.0,  0.0, -1.0, -1.0,
                     1.0,  1.0,  0.0, -1.0, -1.0);
  cv::filter2D(img, img, -1, kernel);
}
/****************************************************************************
                                Features
*****************************************************************************/
//----------------------------------------------------------------------------
/** 
 * Compute keypoints (note to adjust descriptor parameters if modified)
 */
v_kpt compute_keypoints(const cv::Mat & img)
{
  // cv::Ptr<cv::Feature2D> f2d = cv::ORB::create();
  cv::Ptr<cv::Feature2D> f2d = cv::AKAZE::create();
  // cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();
  std::vector<cv::KeyPoint> kp;
  f2d->detect(img, kp);
  return kp;
}

//----------------------------------------------------------------------------
/** 
 * Compute subpixels
 */
void compute_subpixel(const cv::Mat & img, 
                            v_kpt   & kpts)
{
  //if no keypoints, return
  if(kpts.size() == 0) {
    return;
  }

  //convert to point2f
  std::vector<cv::Point2f> pts;
  for(auto _kp : kpts)
  {
    pts.push_back(_kp.pt);
  }

  //compute subpixel
  cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 0.001);
  cornerSubPix(img, pts, cv::Size(5,5), cv::Size(0,0), criteria);

  //convert back to keypoint
  for(size_t i=0; i<pts.size(); i++)
  {
    kpts[i].pt = pts[i];
  }
}

//----------------------------------------------------------------------------
/** 
 * Compute descriptors (note to adjust keypoint parameters if modified)
 */
cv::Mat compute_descriptors(const cv::Mat & img, 
                                  v_kpt   & kpts)
{
  // cv::Ptr<cv::Feature2D> f2d = cv::ORB::create();
  cv::Ptr<cv::Feature2D> f2d = cv::AKAZE::create();
  // cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();
  cv::Mat desc;    
  f2d->compute( img, kpts, desc);
  return desc;
}

//----------------------------------------------------------------------------
/** 
 * Compute matches
 */
v_m compute_matches (cv::Mat desc_i, 
                     cv::Mat desc_j)
{
  cv::BFMatcher matcher;
  std::vector<cv::DMatch> matches;
  matcher.match( desc_i, desc_j, matches );
  return matches;
}

//----------------------------------------------------------------------------
/** 
 * Delete outliers
 */
void  delete_outliers (v_pt2f             & ipts,
                       std::vector<uchar>   inliers)
{
  std::vector<cv::Point2f> ipts_temp; ipts_temp.clear();
  for(size_t i=0; i < inliers.size(); i++)
  {
    if(inliers[i])
    {
      ipts_temp.push_back(ipts[i]);
    }
  }

  ipts.clear();
  ipts = ipts_temp;
}

void  delete_outliers (v_pt3f             & opts,
                       std::vector<uchar>   inliers)
{
  std::vector<cv::Point3f> opts_temp; opts_temp.clear();
  for(size_t i=0; i < inliers.size(); i++)
  {
    if(inliers[i])
    {
      opts_temp.push_back(opts[i]);
    }
  }

  opts.clear();
  opts = opts_temp;
}

void  delete_outliers (v_kpt              & kpts,
                       std::vector<uchar>   inliers)
{
  std::vector<cv::KeyPoint> kpts_temp; kpts_temp.clear();
  for(size_t i=0; i < inliers.size(); i++)
  {
    if(inliers[i])
    {
      kpts_temp.push_back(kpts[i]);
    }
  }

  kpts.clear();
  kpts = kpts_temp;
}

void  delete_outliers (cv::Mat            & desc,
                       std::vector<uchar>   inliers)
{
 cv::Mat desc_temp; desc_temp.release();
  for(size_t i=0; i < inliers.size(); i++)
  {
    if(inliers[i])
    {
      desc_temp.push_back( desc.row(i));
    }
  }

  desc.release();
  desc = desc_temp;
}

//----------------------------------------------------------------------------
/** 
 * Filter matches (option to convert to point2f)
 */
v_pt2f filter_matches_pt2f(v_kpt & kpts, 
                           v_m   & matches, 
                           bool    is_right)
{
  std::vector<cv::Point2f> ipts; ipts.clear();
  if(is_right)
  {
    for(auto m : matches)
    {
      ipts.push_back( kpts[ m.queryIdx ].pt );
    }
  }
  else
  {
    for(auto m : matches)
    {
      ipts.push_back( kpts[ m.trainIdx ].pt );
    }
  }
  return ipts;
}

//----------------------------------------------------------------------------
/** 
 * Filter matches
 */
void filter_matches(v_kpt & kpts, 
                    v_m   & matches, 
                    bool    is_right)
{
  std::vector<cv::KeyPoint> _kpts; _kpts.clear();
  if(is_right)
  {
    for(auto m : matches)
    {
      _kpts.push_back( kpts[ m.queryIdx ] );
    }
  }
  else
  {
    for(auto m : matches)
    {
      _kpts.push_back( kpts[ m.trainIdx ] );
    }
  }
  kpts.clear();
  kpts = _kpts;
}

void filter_matches(cv::Mat & desc, 
                    v_m     & matches, 
                    bool      is_right)
{
  cv::Mat _desc; _desc.release();
  if(is_right)
  {
    for(auto m : matches)
    {
      _desc.push_back( desc.row(m.queryIdx) );
    }
  }
  else
  {
    for(auto m : matches)
    {
      _desc.push_back( desc.row(m.trainIdx) );
    }
  }
  desc.release();
  desc = _desc;
}

void filter_matches(v_pt2f  & ipts, 
                    v_m     & matches, 
                    bool      is_right)
{
  std::vector<cv::Point2f> _ipts; _ipts.clear();
  if(is_right)
  {
    for(auto m : matches)
    {
      _ipts.push_back( ipts[ m.queryIdx ] );
    }
  }
  else
  {
    for(auto m : matches)
    {
      _ipts.push_back( ipts[ m.trainIdx ] );
    }
  }
  ipts.clear();
  ipts = _ipts;
}

void filter_matches(v_pt3f  & opts, 
                    v_m     & matches, 
                    bool      is_right)
{
  std::vector<cv::Point3f> _opts; _opts.clear();
  if(is_right)
  {
    for(auto m : matches)
    {
      _opts.push_back( opts[ m.queryIdx ] );
    }
  }
  else
  {
    for(auto m : matches)
    {
      _opts.push_back( opts[ m.trainIdx ] );
    }
  }
  opts.clear();
  opts = _opts;
}

void convert_kpts_to_ipts(v_kpt  & kpts, 
                          v_pt2f & ipts)
{
  ipts.clear();
  for(auto kpt : kpts) 
  {
    ipts.push_back(kpt.pt);
  }

}

void filter_points(      v_pt2f           & ipts,
                   const std::vector<int> & ids)
{
  v_pt2f ipts_temp;
  for(int i=0; i<ids.size(); i++) {
    ipts_temp.push_back(ipts[ids[i]]);
  }
  ipts = ipts_temp;
}

void filter_points(      v_pt3f           & opts,
                   const std::vector<int> & ids)
{
  v_pt3f opts_temp;
  for(int i=0; i<ids.size(); i++) {
    opts_temp.push_back(opts[ids[i]]);
  }
  opts = opts_temp;
}

/****************************************************************************
                            Transformations
*****************************************************************************/
//----------------------------------------------------------------------------
/** 
 * Compute euler angles (ZYX) from 4x4 transformation matrix
 */
// std::vector<double> get_euler_zyx(const cv::Mat & T)
// {
  
// }

//----------------------------------------------------------------------------
/** 
 * Compute quaternion from 4x4 transformation matrix
 */
// std::vector<double> get_rot_from_quat(const cv::Mat & T)
// {
//   double r11 =
//   double r12 =
//   double r13 =
//   double r21 =
//   double r22 =
//   double r23 =
//   double r31 =
//   double r32 =
//   double r33 =
// }

//----------------------------------------------------------------------------
/** 
 * Compute 4x4 transformation from rotation and translation vectors (see
 * opencv solvepnp output)
 */
cv::Mat compose_T (const cv::Mat & rvec,
                   const cv::Mat & tvec)
{
  cv::Mat R;
  cv::Rodrigues(rvec,R);
  cv::Mat T = (cv::Mat_<double>(4,4) << 
               R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0,0), 
               R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvec.at<double>(1,0), 
               R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2,0),
               0,                 0,                 0,                 1);
  return T;
}

//----------------------------------------------------------------------------
/** 
 * Compute xyz from 4x4 transformation
 */
std::vector<double> get_xyz(const cv::Mat & T)
{
  std::vector<double> xyz; xyz.clear();
  xyz.push_back(T.at<double>(0,3));
  xyz.push_back(T.at<double>(1,3));
  xyz.push_back(T.at<double>(2,3));
  return xyz;
}

//----------------------------------------------------------------------------
/** 
 * Compute rpy from 4x4 transformation
 */
std::vector<double> get_rpy(const cv::Mat & T)
{
  double r21 = T.at<double>(1,0);
  double r11 = T.at<double>(0,0);
  double r31 = T.at<double>(2,0);
  double r32 = T.at<double>(2,1);
  double r33 = T.at<double>(2,2);

  double r = std::atan2( r32, r33);
  double p = std::atan2(-r31, std::sqrt(r32*r32 + r33*r33) );
  double y = std::atan2( r21, r11);

  std::vector<double> rpy; rpy.clear();
  rpy.push_back(r);
  rpy.push_back(p);
  rpy.push_back(y);

  return rpy;
}

/****************************************************************************
                                 Math
*****************************************************************************/
//----------------------------------------------------------------------------
/** 
 * Compute distance between n dimensional points
 */
float dist(std::vector<float> p1, std::vector<float> p2) 
{
  if(p1.size() != p2.size()) 
  {
    std::cout << "Error! p1.size() != p2.size() for computing distance!\n";
    exit(1);
  }

  float val=0;
  for(int i=0; i<p1.size(); i++) 
  {
    float diff = p1[i] - p2[i];
    val = val + diff*diff;
  }
  val = std::sqrt(val);
  return val;
}

double dist(std::vector<double> p1, std::vector<double> p2) 
{
  if(p1.size() != p2.size()) 
  {
    std::cout << "Error! p1.size() != p2.size() for computing distance!\n";
    exit(1);
  }

  double val=0;
  for(int i=0; i<p1.size(); i++) 
  {
    double diff = p1[i] - p2[i];
    val = val + diff*diff;
  }
  val = std::sqrt(val);
  return val;
}

//----------------------------------------------------------------------------
/** 
 * Compute distance between 2 points of cv::Point2f type
 */
float dist(cv::Point2f p1, cv::Point2f p2)
{
  float diffx = p1.x - p2.x;
  float diffy = p1.y - p2.y;
  return std::sqrt( diffx*diffx + diffy*diffy );
}

double dist(cv::Point2d p1, cv::Point2d p2)
{
  double diffx = p1.x - p2.x;
  double diffy = p1.y - p2.y;
  return std::sqrt( diffx*diffx + diffy*diffy );
}

float dist(cv::Point3f p1, cv::Point3f p2)
{
  float diffx = p1.x - p2.x;
  float diffy = p1.y - p2.y;
  float diffz = p1.z - p2.z;
  return std::sqrt( diffx*diffx + diffy*diffy + diffz*diffz );
};

double dist(cv::Point3d p1, cv::Point3d p2)
{
  double diffx = p1.x - p2.x;
  double diffy = p1.y - p2.y;
  double diffz = p1.z - p2.z;
  return std::sqrt( diffx*diffx + diffy*diffy + diffz*diffz );
};

}