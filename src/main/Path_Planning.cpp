/*****************IMPLMENTATION OF PATH PLANNING***************************************
Elaborated by: Ramiro Campos
Description: Implementation of the process of vision known as interpretation. In this case is specifically a path planning
process.*/
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include "common.h"
#include "Path_Planning.h"

using namespace std;
using namespace cv;

Vec3b obstacle_color= Vec3b(0,255,0);

/*Main function for path planning process. Altough it's intented that the function doesn't crash when more obstacles present, we assumed that we are working with two obstacles*/
void path_planning(Mat& obstacles,const uchar& direction, const uchar& front_back)
{
  std::vector<Point> centroids;
  std::vector<int> radiuses;
  
  obstacles = imread("./main/obstacles.png",CV_LOAD_IMAGE_COLOR);
  if(!obstacles.data){
    cout << "Obstacles loading failed" << endl;
    waitKey();
  }
  //Before developing the procedure it's important to binarize the image, in order to be working just with obstacles
  binarize_obstacles(obstacles);
  //Call step 1
  get_characteristics(obstacles,centroids,radiuses);
  //If this happens, what the hell is doing my program :(
  if(centroids.size() != radiuses.size()){
    return;
  }
  //Call step 2 : Obstacles enlargment based on robot parameters
  enlarge_obstacles(obstacles,centroids,radiuses);
  //Call step 3: Planning of road map
  prm(obstacles,centroids,radiuses,direction,front_back);
  //imshow("Road Map",obstacles);
  return;
}

/*STEP 1: Process the obstacles in the original image
In this part basically we obtain the characteristics of the obstacles. Obstacles are assumed to be of circular shape, 
so a modification watershed combined with online moments will be used to get the center and radius of each obstacle*/
void get_characteristics(Mat& obstacles,std::vector<Point>& centroids, std::vector<int>& radiuses)
{
  /*Regions discovery. 
  This process could be done in several ways. It depends on the number of regions desired among others parameters.
  In this case exhaustive search will be used for simplcity.
  */
  Mat Color = Mat::zeros(obstacles.rows,obstacles.cols,CV_8UC3);
  for(int x=0; x<obstacles.cols; x++){
    for(int y = 0; y<obstacles.rows; y++){	
      if(obstacles.at<Vec3b>(x,y) == obstacle_color && Color.at<Vec3b>(x,y) == Vec3b(0,0,0)){
        long int ordinary_moments[6] = {0};	
        //Call segmentation
        obstacles_water_shed(Point(x,y),obstacles,Color,ordinary_moments);
        //Call function to get centroid and radius of this region
        Point centroid;
        int radius;
        get_location(ordinary_moments,centroid,radius);
        //Now that we have the center and the radius, push it to the corresponding array
        centroids.push_back(centroid);
        radiuses.push_back(radius);
      }
    }
  }
  return;
}

void obstacles_water_shed(Point seed,Mat& obstacles,Mat& Color,long int ordinary_moments[])
{
  points_vector fo; //Temporal container of points for region interation
  points_vector q(4,Point(0,0));
  q.at(0) = Point(1,0); q.at(1) = Point(0,1); q.at(2) = Point(-1,0); q.at(3) = Point(0,-1);
  Point pa,pe;
  //Push seed and color it on segmented image
  fo.push_back(seed);
  //At the end, one point will not change the result very much and one point is not the kind of regions we are looking for,
  //but theoretically it should be done
  //m00 = sum(x*y) or Area or size of region
  ordinary_moments[M00]++;
  //Now its time to apply main algorithm to find the region
  while(!fo.empty()){
    pe = fo.front();
    fo.erase(fo.begin());
    for(uchar neighbor =0;neighbor<4;neighbor++){
      pa = pe + q[neighbor];
      //Verify if a new point has been discovered
      if(Color.at<Vec3b>(pa.x,pa.y) == Vec3b(0,0,0) && obstacles.at<Vec3b>(pa.x,pa.y) == obstacle_color){
        Color.at<Vec3b>(pa.x,pa.y) = obstacle_color;
        fo.push_back(pa);
        //Parameters that are going to be useful for center and radius
        //Online computation of second order ordinary moments
        //m00 = sum(x*y) or Area or size of region
        ordinary_moments[M00]++;
        //m10 = sum(x) m01 = sum(y) m11 = sum(x*y) m02 = sum(y^2) m20 = sum(x^2)
        ordinary_moments[M10] += pa.x;
        ordinary_moments[M01] += pa.y;
        ordinary_moments[M11] += pa.x*pa.y;
        ordinary_moments[M02] += pa.y*pa.y;
        ordinary_moments[M20] += pa.x*pa.x;
      }
    }
  }
  return;
}

void get_location(const long int ordinary_moments[],Point& centroid,int& radius)
{ 
  //centroid = (xc,yx)
  //xc = m10/m00
  double xc = (int)ordinary_moments[M10]/ordinary_moments[M00];
  //yc = m01/m00
  double yc = (int)ordinary_moments[M01]/ordinary_moments[M00];    
  centroid = Point(yc,xc);
  //radius = sqrt(Area / pi) 
  radius =  (int) sqrt(ordinary_moments[M00] / PI);
  return;
}

/*STEP 2: OBSTACLES ENLARGMENT BASED ON ROBOT CHARACTERISTICS
In this function is carried out the process of obstacles enlargment according to the robot size and shape. In this case the situation is so simple, since the parrot is a circular object and the obstacles are circular*/
void enlarge_obstacles(Mat& obstacles,std::vector<Point>& centroids,std::vector<int>& radiuses)
{
  //This is the proportion of the robot with respect to the obstacles
  int proportion = 6;
  //Now, just increase the radius for each object
  for(int i=0; i<(int)radiuses.size(); i++){
    radiuses[i] *= (1+proportion);
    //Now that we know the size, enlarge it
    circle(obstacles,centroids[i],radiuses[i],(Scalar)obstacle_color,-1);
  }
  return;
}

/*STEP 3: PLANNIG OF TRAJECTORY 
Now that the working enviroment has been modified accroding according to the characteristics of the road, it's time for planning the route it will follow. The selected algorithm is an NF1 algorithm based on 4 vecinity.*/
void prm(Mat& obstacles,const std::vector<Point>& centroids, std::vector<int>& radiuses, const uchar& direction, const uchar& front_back)
{
  //The entire map bit will be divided in squares in which each seed will be the center of the squares. Each square will be 10 pixels size, so each seed will be separated 10 pixels
  //Define limits both low and upper
  int x_low_limit = 0;
  int x_upper_limit = obstacles.cols;
  int y_low_limit =(obstacles.rows-obstacles.cols);
  int y_upper_limit = obstacles.rows;
  //Now it's time to compute the starting, front and back points in terms of the new division in squares. Consider that the central point of each square is size/2 of the border.
  //Centroid points
  int start_x = centroids[0].x - (centroids[0].x % square_size);
  int start_y = (int)(((centroids[0].y-radiuses[0])+y_low_limit)/(2*square_size))*square_size;
  Point start = Point(start_x,start_y);
  circle(obstacles,start,5,Scalar(255,0,0),-1);
  
  int front_x = (int)(centroids[0].x + centroids[1].x)/2 - ((int)(centroids[0].x + centroids[1].x)/2 % square_size);
  int front_y = ((int)((centroids[0].y+radiuses[0])+(centroids[1].y-radiuses[1]))/(2 * square_size)) * square_size;
  Point front = Point(front_x,front_y);
  circle(obstacles,front,5,Scalar(255,0,0),-1);

  int back_x = centroids[1].x - (centroids[1].x % square_size);
  int back_y = (int)((centroids[1].y+radiuses[1]+y_upper_limit)/(2 * square_size)) * square_size;
  Point back = Point(back_x,back_y);
  circle(obstacles,back,5,Scalar(255,0,0),-1);

  //Now it's time to find the shortest path
  shortest_path(obstacles,direction,front_back,start,front,back,centroids,radiuses);
  return;
}

/*This a dummy function to compute the shortest path between two points in a working environment with obstacles. We strongly
recommend not using this kind of things :(
*/
void shortest_path(Mat& obstacles,const uchar& direction,const uchar& front_back,const Point& start,const Point& front, const Point& back,std::vector<Point> centroids, std::vector<int> radiuses)
{
  //To make a road we basically need four points. We actually have just one : start. So let's find P1,P2,destiny.
  Point p1,p2,destiny;
  int p1_x,p1_y,p2_x,p2_y;
  //Let's fine destiny.Parse options
  destiny = front_back == FRONT? front : back;
  
  if(direction == LEFT){
    p1_x = (((start.x - radiuses[0]) / square_size) * square_size) - square_size*3;
    p1_y = start.y;
    p1 = Point(p1_x,p1_y);
    p2_x = p1_x;
    p2_y = destiny.y;
    p2 = Point(p2_x,p2_y);
  }
  else if(direction == RIGHT){
    p1_x = (((start.x + radiuses[1]) / square_size) * square_size) + square_size*3;
    p1_y = start.y;
    p1 = Point(p1_x,p1_y);
    p2_x = p1_x;
    p2_y = destiny.y;
    p2 = Point(p2_x,p2_y);
  }
  //Join points
  line(obstacles,start,p1,Scalar(0,0,255),3);
  line(obstacles,p1,p2,Scalar(0,0,255),3);
  line(obstacles,p2,destiny,Scalar(0,0,255),3);
  return;
}

/*This function is used to binarize the image. This is done to ensure that we are working just with obstacles*/
void binarize_obstacles(Mat& obstacles)
{
  uchar object_green,object_red,object_blue;
  for(int x = 0; x < obstacles.rows; x++){
    for(int y = 0; y < obstacles.cols; y++){
      object_blue = obstacles.at<Vec3b>(x,y)[0];
      object_green = obstacles.at<Vec3b>(x,y)[1];
      object_red = obstacles.at<Vec3b>(x,y)[2];
      if(object_green > 240 && object_blue < 200 && object_red < 200){
        obstacles.at<Vec3b>(x,y) = obstacle_color;
      }
    }
  }
  return;
}
