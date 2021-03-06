/************************HEADER FILE FOR PLATH PLANNING PROCESS*********************************/
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include "common.h"

#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

using namespace std;
using namespace cv;

const int square_size = 10;

void path_planning(Mat& obstacles,const uchar& direction, const uchar& front_back);

void binarize_obstacles(Mat& obstacles);

void get_characteristics(Mat& obstacles,std::vector<Point>& centroids, std::vector<int>& areas);

void obstacles_water_shed(Point seed,Mat& obstacles,Mat& color,long int ordinary_moments []);

void get_location(const long int ordinary_moments[],Point& centroid,int& radius);

void enlarge_obstacles(Mat& obstacles,std::vector<Point>& centroids,std::vector<int>& radiuses);

void prm(Mat& obstacles,const std::vector<Point>& centroids, std::vector<int>& radiuses, const uchar& direction, const uchar& front_back);

/*NOT USE UNLESS YOU HAVE A LOT OF PROJECTS :( :(*/
void shortest_path(Mat& obstacles,const uchar& direction,const uchar& front_back,const Point& start,const Point& front, const Point& back,std::vector<Point> centroids, std::vector<int> radiuses);

#endif //PATH_PLANNING_H

