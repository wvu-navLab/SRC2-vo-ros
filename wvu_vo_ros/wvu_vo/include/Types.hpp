// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file  Types.hpp
 *
 * <br>@b Author(s):    Jared Strader
 * <br>@b Date:         9/14/18
 */
//-----------------------------------------------------------------------------

#ifndef Types_HPP
#define Types_HPP

namespace VO
{

struct GFTTParameters {
    int    max_features; 
    double quality;
    double min_distance;
    int    block_size;
    bool   use_harris_detector;
    double k;
    bool   empty = true;
};

struct KLTParameters {
    int  max_iter;
    int  eps;
    int  window;
    int  max_level;
    int  max_age;
    bool empty = true;
};

struct BMParameters {
    int  block_size;
    int  num_disparities;
    int  min_disparity;
    int  max_disparity;
    int  uniqueness_ratio;
    int  texture_threshold;
    int  speckle_window_size;
    int  speckle_range;
    int  prefilter_size;
    int  prefilter_cap;
    int  use_semi_global;
    bool empty = true;
};

}

#endif // Types_HPP
