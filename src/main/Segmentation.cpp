/*Implementation of characterization and segmentation
Elaborated by: Ramiro Campos Flores
Description: This program implements an algorithm the part of the process of the vision corresponding to segmentation. It colors each region in a different color. It is based in the algorithm water shed*/

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include "common.h"
#include "Segmentation.h"

using namespace std;
using namespace cv;

/*This functions is in charge of the part of the vision process known as segmentation and characterization
  Actually the whole process is not carried out by this function. This function acts as an intermediary between specific
  functions such as segmentation and characterization functions.
  In a few words, this functions iterates over the input image to find foreground points or in others words, to discover
  regions. It will receive a parameter called option that will determine
*/
void segmentation_and_characterization(Mat& image2segment,Mat& segmentedImage,std::vector<region_info>& regions_table)
{
  /*Regions discovery. 
  This process could be done in several ways. It depends on the number of regions desired among others parameters.
  In this case exhaustive search will be used for simplcity.
  */
  /*printf("Image to segment rows:%d cols:%d\n",image2segment.rows,image2segment.cols);
  printf("Image Color      rows:%d cols:%d\n",segmentedImage.rows,segmentedImage.cols);
  waitKey();*/	
  for(int x=30; x<image2segment.rows-30; x++){
    for(int y = 30; y<image2segment.cols-30; y++){
      //printf("x: %d y:%d\n",x,y);
      //Verify if a new region has been discovered. If so, call segmentation and characterization, if not, do nothing.
      if(x>= 0 && y>= 0&& image2segment.at<uchar>(x,y) == object && segmentedImage.at<Vec3b>(x,y) == Vec3b(0,0,0)){
        //printf("Region %d discovered\n",regions_table.size());
        //Create a new registry in the regions table
        if(regions_table.size() >= 10){
          return;
        }
        region_info region_parameters;
        long int ordinary_moments[6] = {0};	
        regions_table.push_back(region_parameters);
        //Call segmentation
        water_shed(Point(x,y),segmentedImage,image2segment,ordinary_moments,x*y);
        //Discard regions of noise
        if(ordinary_moments[M00] < 500){
          regions_table.pop_back();
        }
        if(regions_table.empty()){
          return;
        }
          //Call characterization
          characterization(regions_table.back(),ordinary_moments);
      }
    }
  }
  //After the process is important to give a visual demonstration of the results
  draw_parameters(regions_table,segmentedImage);
  return;
}

/*
This function is used to carry out the segmentation process. Basically it is a straight implementation of the
well known water hed algortithm. In order to achieve performance, some modularity has been lost. The statement before 
is because ordinary moments computation has been added in this function to avoid store the points in a vector.

TODO:Comparison needs to be done about the effect of each of the two ways of computing ordinary moments in order to establish if the sacrifice of modularity for performance is worthy.
*/
void water_shed(Point seed,Mat& Color,const Mat& original,long int ordinary_moments [],int n)
{
  RNG rng(n);
  Scalar color = randomColor(rng);
  static int ctr = 0;
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
  Color.at<Vec3b>(seed)[0] = color[0];
  Color.at<Vec3b>(seed)[1] = color[1];
  Color.at<Vec3b>(seed)[2] = color[2];
  //Now its time to apply main algorithm to find the region
  while(!fo.empty()){
    pe = fo.front();
    fo.erase(fo.begin());
    for(uchar neighbor =0;neighbor<4;neighbor++){
      pa = pe + q[neighbor];
      //Verify if a new point has been discovered
      //printf("Pa.x:%d Pa.y: %d\n",pa.x,pa.y);
      if(pa.x >= 1 && pa.y >= 1 && Color.at<Vec3b>(pa.x,pa.y)[0] == 0 && Color.at<Vec3b>(pa.x,pa.y)[1] == 0 && Color.at<Vec3b>(pa.x,pa.y)[2] == 0 && original.at<uchar>(pa.x,pa.y) == object){
        Color.at<Vec3b>(pa.x,pa.y)[0] = color[0];
        Color.at<Vec3b>(pa.x,pa.y)[1] = color[1];
        Color.at<Vec3b>(pa.x,pa.y)[2] = color[2];
        //region_points.push_back(pa);
        fo.push_back(pa);
        //Characterization intrusion
        //Online computation of second order ordinary moments
        //m00 = sum(x*y) or Area or size of region
        ordinary_moments[M00]++;
        //m10 = sum(x) m01 = sum(y) m11 = sum(x*y) m02 = sum(y^2) m20 = sum(x^2)
        ordinary_moments[M10] += pa.x;
        ordinary_moments[M01] += pa.y;
        ordinary_moments[M11] += pa.x*pa.y;
        ordinary_moments[M02] += pa.y*pa.y;
        ordinary_moments[M20] += pa.x*pa.x;
        //End of characterization intrusion
      }
    }
  }
  return;
}

/*
This function is in charge of the process of the vision called characterization
It main objective is to compute the invariant moments that will represent a region in order to allow recognition when the system is operating
*/
void characterization(region_info& region_parameters,const long int ordinary_moments[])
{
  Point centroid;
  float orientation; 
  double hu_moments[2] = {0};
  //double hu_moments_static[2] = {0};
  //We want to compute the translation,scale and rotation invariant moments of Hu. Furthermore we are intersted in the centroid and the principal angle of the region
  compute_invariant_parameters(hu_moments,centroid,orientation,"online",ordinary_moments);
  //compute_invariant_parameters(hu_moments_static,centroid_static,orientation_static,"static",ordinary_moments,region_points);
  //Introduce computed parameters to region entry
  region_parameters.hu_moments[fi_1] = hu_moments[fi_1];
  region_parameters.hu_moments[fi_2] = hu_moments[fi_2];
  region_parameters.centroid = centroid;
  region_parameters.orientation = orientation;
  //printf("  For online computation => Centroid:(%d,%d) Orientation: %f hu1: %f hu2: %f\n",centroid.x,centroid.y,90-orientation*180/3.14159,hu_moments[fi_1],hu_moments[fi_2]);
  //printf("For static computation => Centroid:(%d,%d) Orientation: %f hu1: %f hu2: %f\n",centroid_static.x,centroid_static.y,(orientation_static*180/3.14159),hu_moments_static[fi_1],hu_moments_static[fi_2]);
  return;
}

void compute_invariant_parameters(double hu_moments[],Point& centroid, float& orientation,const string& compute_type,const long int ordinary_moments[])
{
  //So following are the steps to get the invariant moments of Hu,centroid and orientation
  //1-.Computation of ordinary moments and translational invariant central moments,centroid and orientation. For this we have two options. Both of them are investigated to verify correctness.
  //Central translation invariant moments(M00,M01,M10,M02,M20,M11)
  double central_moments[6] = {0};
  if(compute_type == "online")
    online_central_moments(ordinary_moments,centroid,orientation,central_moments);
  //else 
    //static_central_moments(region_points,centroid,orientation,central_moments);
  //2-.Computation of translational and scale invariant normalized moments and translational,scale and rotaion invariant hu moments.
  normalized_and_hu_computation(central_moments,hu_moments);
  return;
}

/*
This function computes the invariant moments of Hu and related parameters such as centroid and orientation using
an online approach. The only part is carried out during segmentation, while computing each iterarion he ordinary moments.
In this part, central,normalizaed and invariant moments of Hu are computed based on those previously computed ordinary moments.
*/
void online_central_moments(const long int ordinary_moments[],Point& centroid,float& orientation,double central_moments[])
{
   //printf("m11: %d m01: %d m10: %d m00: %d\n",ordinary_moments[M11],ordinary_moments[M01],ordinary_moments[M10],ordinary_moments[M00]);
  //M00 = m00
  central_moments[M00] = ordinary_moments[M00];
  //M01,M10 = 0
  central_moments[M01] = central_moments[M10] = 0;
  //M02 = m02 - (m01 ^ 2)/m00
  central_moments[M02] = ordinary_moments[M02] - (double)((ordinary_moments[M01]*ordinary_moments[M01])/ordinary_moments[M00]);
  //M20 = m20 - (m10 ^ 2)/m00
  central_moments[M20] = ordinary_moments[M20] - (double)((ordinary_moments[M10]*ordinary_moments[M10])/ordinary_moments[M00]);
  //M11 = m10*m01*[(m00-1)/m00] 
  central_moments[M11] = ordinary_moments[M11]-(ordinary_moments[M10]*ordinary_moments[M01]/(double)ordinary_moments[M00]);
  //centroid = (xc,yx)
  //xc = m10/m00
  double xc = (int)ordinary_moments[M10]/ordinary_moments[M00];
  //yc = m01/m00
  double yc = (int)ordinary_moments[M01]/ordinary_moments[M00];    
  centroid = Point(xc,yc);
  //orientation
  //printf("Parameters for atan function: M11*2: %f M20-M02: %f\n",2*central_moments[M11],central_moments[M20]-central_moments[M02]);
  //orientation = 0.5*atan(2*central_moments[M11]/(central_moments[M20]-central_moments[M02]));
  orientation = 0.5*atan2(2*central_moments[M11],(central_moments[M20]-central_moments[M02]));
  //printf("  Orientation_before: %f \n",orientation*180/3.14159);
  orientation = orientation < 0?(PI/2 + orientation):(-PI/2 + orientation);
  //printf("  Orientation: %f \n",orientation*180/3.14159);
  //printf("  Ordinary moments by online computation => m00: %ld m01: %ld m10: %ld m02: %d m20: %ld m11: %ld\n",ordinary_moments[M00],ordinary_moments[M01],ordinary_moments[M10],ordinary_moments[M02],ordinary_moments[M20],ordinary_moments[M11]);
  //printf("  Central moments by online computation => M00: %f M02: %f M20: %f M11: %f\n",central_moments[M00],central_moments[M02],central_moments[M20],central_moments[M11]);
  //printf("  Centroid: (%f,%f) => xc: %df yc: %d\n",centroid.x,centroid.y,xc,yc);*/
  return;
}

/*
This function computes the invariant moments of Hu and related parameters such as centroid and orientation using
a static approach. Static means, that the function will receive a vector of points from which it will compute the ordinary,central,normalizaed and invariant Hu moments
*/
void static_central_moments(const points_vector& region_points,Point& centroid,float& orientation,double central_moments[])
{
  //Ordinary moments
  long int ordinary_moments[6]= {0};
  //m00 = Area of the region or size of the region
  ordinary_moments[M00] = region_points.size();
  //m01 = sum(yi) m10 = sum(xi) m11 = sum(xi*yi) m02=sum(y^2) m20=sum(x^2)
  for(int i=0;i<region_points.size();i++){
    ordinary_moments[M01] += region_points[i].y;
    ordinary_moments[M10] += region_points[i].x;
    ordinary_moments[M11] += region_points[i].x*region_points[i].y;
    ordinary_moments[M02] += region_points[i].y*region_points[i].y;
    ordinary_moments[M20] += region_points[i].x*region_points[i].x; 
  }
  //Translation invariant central moments
  //M00 = m00
  central_moments[M00] = ordinary_moments[M00];
  //M01 = M10 = 0
  central_moments[M01] = central_moments[M10] = 0;
  //xc = m10/m00
  double xc = (double)ordinary_moments[M10]/ordinary_moments[M00];
  //yc = m01/m00
  double yc = (double)ordinary_moments[M01]/ordinary_moments[M00];  
  //M02 = sum(y-yc)^2 M20 = sum(x-xc)^2 M11 = sum[(x-xi)(y-yc)]
  for(int i=0;i<region_points.size();i++){
    central_moments[M02] += (region_points[i].y - yc)*(region_points[i].y - yc);
    central_moments[M20] += (region_points[i].x - xc)*(region_points[i].x - xc);
    central_moments[M11] += (region_points[i].x - xc)*(region_points[i].y - yc);
  }	
  //centroid = (xc,yx)
  centroid = Point(xc,yc);
  //orientation
  orientation = 0.5*atan(2*central_moments[M11]/(central_moments[M20]-central_moments[M02]));
  //printf("  Ordinary moments by static computation => m00: %ld m01: %ld m10: %ld m02: %d m20: %ld m11: %d\n",ordinary_moments[M00],ordinary_moments[M01],ordinary_moments[M10],ordinary_moments[M02],ordinary_moments[M20],ordinary_moments[M11]);
  //printf("  Central moments by static computation => M00: %f M02: %f M20: %f M11: %f\n",central_moments[M00],central_moments[M02],central_moments[M20],central_moments[M11]);
  //printf("  Centroid: (%f,%f) => xc: %d yc: %d\n",centroid.x,centroid.y,xc,yc);*/ 
  return;
}

void normalized_and_hu_computation(const double central_moments[],double hu_moments[])
{
  //Translational and scale invariant normalized moments(N00,N01,N10,N02,N20,N11)
  double normalized_moments[6] = {0};
  //N00 = M00/M00 = 1
  normalized_moments[M00] = 1;
  //N02 = M02/m00^2
  normalized_moments[M02] = central_moments[M02]/(central_moments[M00]*central_moments[M00]);
  //N20 = M20/m00^2
  normalized_moments[M20] = central_moments[M20]/(central_moments[M00]*central_moments[M00]);
  //N11 = M11/m00^2
  normalized_moments[M11] = central_moments[M11]/(central_moments[M00]*central_moments[M00]);

  //Translational, scale and time invariant Hu moments
  double pi[2] = {0};
  hu_moments[fi_1] = normalized_moments[M20] + normalized_moments[M02];
  hu_moments[fi_2] = ((normalized_moments[M20]-normalized_moments[M02])*(normalized_moments[M20]-normalized_moments[M02])) + (4*normalized_moments[M11]*normalized_moments[M11]);
  return;
}

#define LINE_LENGTH 50
void draw_parameters(const std::vector<region_info>& regions_table,Mat& segmentedImage)
{
    //Draw centroid and principal angle for each region
    for(int i=0;i<regions_table.size();i++){
      //Draw centroid
      circle(segmentedImage,Point(regions_table[i].centroid.y,regions_table[i].centroid.x),5,Scalar(0,0,255),5);
      //circle(segmentedImage,Point(regions_table[i].centroid.x,regions_table[i].centroid.y),5,Scalar(0,0,255),5);
      //Draw principal angle using y = x*tan(orientation)
        //First compute angle supposing that the center of the image is in the centroid.
      int x1,x2,y1,y2;
      //printf("Tangent of B: %f\n",tan(regions_table[i].orientation));
      x1 = (int)round(LINE_LENGTH*cos(regions_table[i].orientation));
      x2 = (int)round(-LINE_LENGTH*cos(regions_table[i].orientation));
      y1 = (int)round(LINE_LENGTH*sin(regions_table[i].orientation));
      y2 = (int)round(-LINE_LENGTH*sin(regions_table[i].orientation));
        //Since in the image plane, the y axis is positive in downward direction it's neccessary to invert previously 
        //computed coordinate, since the convention was positive upward
      y1 = -y1;
      y2 = -y2;
        //Now it's time to make the correction,so the image can be placed in the corresponding location of the region in the 
        //image
      x1 += regions_table[i].centroid.y;
      x2 += regions_table[i].centroid.y;
      y1 += regions_table[i].centroid.x;
      y2 += regions_table[i].centroid.x;
      
      //The only thing to do is to draw the line
      line(segmentedImage,Point(x1,y1),Point(x2,y2),Scalar(0,0,255),5,8);
      //printf("Region %d: p1:(%d,%d) p2:(%d,%d) %d %d\n",(int)i,x1,y1,x2,y2,segmentedImage.rows,segmentedImage.cols);
    } 
      
  return;
}

static Scalar randomColor(RNG& rng )
{
  int icolor = (unsigned) rng;
  return Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
}

void print_regions(const std::vector<region_info> regions_table)
{
  for(int i=0;i<regions_table.size();i++){
    printf("Region: %d ===> Hu1: %f Hu2: %f orientation: %f\n",i,regions_table[i].hu_moments[0],regions_table[i].hu_moments[1],regions_table[i].orientation);
 
  }
  return;
}
