#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include "common.h"


using namespace std;
using namespace cv;

#define object 255
#define background 0

void segmentation_and_characterization(Mat& image2segment,Mat& segmentedImage,std::vector<region_info>& regions_table);
void water_shed(Point seed,Mat& Color,const Mat& original,long int ordinary_moments[],int n);
void characterization(region_info& region_paramters,const long int ordinary_moments[]);
void compute_invariant_parameters(double hu_moments[],Point& centroid, float& orientation,const string& compute_type,const long int ordinary_moments[]);
void online_central_moments(const long int ordinary_moments[],Point& centroid,float& orientation,double central_moments[]);
void static_central_moments(const points_vector& region_points,Point& centroid,float& orientation,double central_moments[]);
void normalized_and_hu_computation(const double central_moments[],double hu_moments[]);
void draw_parameters(const std::vector<region_info>& regions_table,Mat& segmentedImage);
static Scalar randomColor(RNG& rng );
void print_regions(const std::vector<region_info> regions_table);
