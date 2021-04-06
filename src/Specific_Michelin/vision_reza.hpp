#ifndef HEADERS_H_INCLUDED
#define HEADERS_H_INCLUDED

#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <vector>
#include <iterator> // for std::size
#include <fstream>
#include <functional>   // std::minus
#include <algorithm>    // std::transform
#include <typeinfo>       // operator typeid
#include <chrono>
#include <string>
#include <numeric>
#include <stdlib.h>
#include <math.h>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/xfeatures2d/nonfree.hpp>
//#include "opencv2/features2d/features2d.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
//#include "CThinPlateSpline.h"
//#include "shape_transformer.hpp"
#include <opencv2/shape/shape_transformer.hpp>

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

typedef std::vector<double> DoubleVector1D;
typedef std::vector<std::vector<double>> DoubleVector2D;
typedef std::vector<std::vector<std::vector<double>>> DoubleVector3D; 

void reza_vision(DoubleVector2D& final_transformation, DoubleVector1D& left_point, DoubleVector1D& right_point, bool show_image);

#endif // HEADERS_H_INCLUDED
