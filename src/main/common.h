#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "CHeli.h"

#ifndef COMMON
#define COMMON
using namespace std;
using namespace cv;

/*Fligth Parameters*/
const int seg1=1000000;

/*Segmentation and characterization*/
const double PI = 2*acos(0.0);
typedef unsigned char uchar;
typedef std::vector<cv::Point> points_vector;

enum ordinary_moments_t{M00=0,M01=1,M10=2,M11=3,M02=4,M20=5};
enum hu_moments_t{fi_1=0,fi_2=1};

/*
Region entry: This structure contains all the fields required to carry out the recognition process and to identify a region both visually and numerically.
*/
typedef struct region_info{
  double hu_moments[2];
  Point centroid;
  float orientation;
} region_info;

/*Clasification*/
#define X_MOVE 0
#define RIGHT 0
#define LEFT 1

#define Y_MOVE 1
#define FRONT 0
#define BACK 1

/*Planning of paths*/
#endif 

