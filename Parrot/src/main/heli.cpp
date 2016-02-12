#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "SDL/SDL.h"
#include <stdlib.h>
#include "CHeli.h"
#include <unistd.h>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

bool stop = false;
CRawImage *image;
CHeli *heli;
int Px;
int Py;
vector<Point> points; // Here we will store points
int image_h = 240;
int image_w = 320;
Mat imagenClick;
Mat flippedImage; //Flipped image matrix declaration and function definition
//Histogram calculation variables
vector<Mat> planes;
//Mat hist_plane1,hist_plane2,hist_plane3;
bool shape_selected = false;
bool shape_complete = false;

//Flip function definition
/*
 * This method flips horizontally the sourceImage into destinationImage. Because it uses 
 * "Mat::at" method, its performance is low (redundant memory access searching for pixels).
 */
void flipImageBasic(const Mat &sourceImage, Mat &destinationImage)
{
	if (destinationImage.empty())
		destinationImage = Mat(sourceImage.rows, sourceImage.cols, sourceImage.type());

	for (int y = 0; y < sourceImage.rows; ++y)
		for (int x = 0; x < sourceImage.cols / 2; ++x)
			for (int i = 0; i < sourceImage.channels(); ++i)
			{
				destinationImage.at<Vec3b>(y, x)[i] = sourceImage.at<Vec3b>(y, sourceImage.cols - 1 - x)[i];
				destinationImage.at<Vec3b>(y, sourceImage.cols - 1 - x)[i] = sourceImage.at<Vec3b>(y, x)[i];
			}
}

// Convert CRawImage to Mat
void rawToMat( Mat &destImage, CRawImage* sourceImage)
{	
	uchar *pointerImage = destImage.ptr(0);
	
	for (int i = 0; i < 240*320; i++)
	{
		pointerImage[3*i] = sourceImage->data[3*i+2];
		pointerImage[3*i+1] = sourceImage->data[3*i+1];
		pointerImage[3*i+2] = sourceImage->data[3*i];
	}
}

void mouseCoordinatesPractice1Callback(int event, int x, int y, int flags, void* param);
void draw_shape(Mat& imagenClick);
void get_range(Vec3b& max, Vec3b& min, Mat& imagenClick);
void get_range_coordinates(int& max_x, int& max_y, int& min_x, int& min_y);
void draw_search_region(int& max_x, int& max_y, int& min_x, int& min_y, Mat& imagenClick);
void get_range_color(Vec3b& max,Vec3b& min, int& max_x, int& max_y, int& min_x, int& min_y,Vec3b& channels_size,Mat& imagenClick);
void get_histogram(Mat& histImage,Mat& plane,Scalar color,int histSize,int bin_w);
void mark_range(Mat& histogramImage, const int& min, const int& max,int bin_w);



int main(int argc,char* argv[])
{
  //Image variables
  int max_x,max_y,min_x,min_y;
  Vec3b max_color,min_color;
  Vec3b channels_size;
  //Histogram variables
  int histSize = 256;
  int bin_w = cvRound((double) image_w/histSize);
  Mat histImage1(image_h,image_w,CV_8UC3,Scalar(0,0,0));
  Mat histImage2(image_h,image_w,CV_8UC3,Scalar(0,0,0));
  Mat histImage3(image_h,image_w,CV_8UC3,Scalar(0,0,0));
  namedWindow("Histogram1", CV_WINDOW_AUTOSIZE);
  namedWindow("Histogram2", CV_WINDOW_AUTOSIZE);
  namedWindow("Histogram3", CV_WINDOW_AUTOSIZE);

  //establishing connection with the quadcopter
  heli = new CHeli(); 
	
  //this class holds the image from the drone	
  image = new CRawImage(320,240);

  // Destination OpenCV Mat	
  Mat currentImage = Mat(240, 320, CV_8UC3);  

  //Windows to show working images	
  imshow("ParrotCam", currentImage);
  namedWindow("Click");
  namedWindow("Flipped");

  setMouseCallback("Click",mouseCoordinatesPractice1Callback);

  while(stop == false){
    // Clear the console
    printf("\033[2J\033[1;1H");
    //cout<<"Pos X: "<<Px<<" Pos Y: "<<Py<<endl;
   
    //Clear histograms
    histImage1 = Scalar(0,0,0);
    histImage2 = Scalar(0,0,0);
    histImage3 = Scalar(0,0,0);		
	
    //image is captured
    heli->renewImage(image);

    // Copy to OpenCV Mat
    rawToMat(currentImage, image);
    imshow("ParrotCam", currentImage);

    //Image flipping
    flipImageBasic(currentImage,flippedImage);
    imshow("Flipped",flippedImage);

    //If the user don't want to continue, let him go
    cout << "Options: " << endl;
    cout << "Get out : press escape" << endl;
    cout << "Press y to begin to draw a region for image filtering" << endl;
    cout << "Press right button when finishing region selection" << endl;
    cout << "Prees c to clear data" << endl;
    char key = waitKey(5);
      switch (key) {
        case 27:   stop = true; 
                   break;
        case 121:  shape_selected = true;
                   break;
        case 99:   shape_selected = false;
                   shape_complete = false;
                   points.clear();
                   histImage1 = Scalar(0,0,0);
                   histImage2 = Scalar(0,0,0);
                   histImage3 = Scalar(0,0,0);
                   break;
	default: ;
      }
    
    //If the user haven't selected any image to filter why bothering the system doing histograms
    imagenClick = currentImage;
    if(!shape_selected){
      imshow("Click", imagenClick);
      imshow("Histogram1", histImage1);
      imshow("Histogram2", histImage2);
      imshow("Histogram3", histImage3);
      usleep(15000);
      continue;
    }
    //Line drawing
    draw_shape(imagenClick);
    //figure showing after drawing
    imshow("Click", imagenClick);
    //If user haven't complety defined shape, why computing histograms?
    if(!shape_complete){
      continue;
    }
    //Getting maximum and minimum values for coordinates of the selected region to establish a searching region
    get_range_coordinates(max_x,max_y,min_x,min_y);
    draw_search_region(max_x,max_y,min_x,min_y,imagenClick);
    imshow("Click", imagenClick);
    //Get maximum and minimum in terms of channels
    channels_size[0] = 255;
    channels_size[1] = 255;
    channels_size[2] = 255;
    get_range_color(max_color,min_color,max_x,max_y,min_x,min_y,channels_size,imagenClick);
    imshow("Click", imagenClick);

    //Histogram calculation
    split( currentImage, planes );
    get_histogram(histImage1,planes[0],Scalar(255,0,0),histSize,bin_w);
    get_histogram(histImage2,planes[1],Scalar(0,255,0),histSize,bin_w);
    get_histogram(histImage3,planes[2],Scalar(0,0,255),histSize,bin_w);

    //Mark selected range for filtering
    mark_range(histImage1,(int)max_color[0],(int)min_color[0],bin_w);
    mark_range(histImage2,(int)max_color[1],(int)min_color[1],bin_w);
    mark_range(histImage3,(int)max_color[2],(int)min_color[2],bin_w);

    //Display histograms
    imshow("Histogram1", histImage1);
    imshow("Histogram2", histImage2);
    imshow("Histogram3", histImage3);

    //Sleep the system to avoid conflicts
    usleep(15000);
  }
	
    heli->land();
    delete heli;
	delete image;
	return 0;
}

//Mouse coordinate displaying function
void mouseCoordinatesPractice1Callback(int event, int x, int y, int flags, void* param)
{
    switch (event)
    {
        case CV_EVENT_LBUTTONDOWN:
              if(!shape_complete and shape_selected){
              Px = x;
              Py = y;
              points.push_back(Point(x, y));
            }
            break;
        case CV_EVENT_MOUSEMOVE:
            break;
        case CV_EVENT_LBUTTONUP:
            break;
        case CV_EVENT_RBUTTONDOWN:
            shape_complete = true;
    }
}

void draw_shape(Mat& imagenClick)
{
  if(points.size() > 1){
    //cout << "Last Point Pressed: " <<points.size()<<" .Coordinates are : " << Px << ", " << Py << endl;
    circle(imagenClick, (Point)points[0], 5, Scalar( 0,0,255), CV_FILLED);                    
    for(int i = 0; i < (int)(points.size()-1); i++){
      line(imagenClick,
          (Point)points[i],
          (Point)points[i+1],
          Scalar( 0,0,255));
      }
    }
  else if (points.size() == 1){
    circle(imagenClick, (Point)points[0], 5, Scalar( 0,0,255), CV_FILLED);
    }
  return;
}

void get_range_colors(Vec3b& max, Vec3b &min)
{
  return;
}

void get_range_coordinates(int& max_x, int& max_y, int& min_x, int& min_y)
{
  max_x = 0;
  max_y = 0;
  min_x = image_w;
  min_y = image_h;
  //Iterare over each triangle of the selected shape
  for(int vertix=0; vertix < points.size()-1; vertix++){
    //Looking for maxs
    if(points[vertix].x > max_x)
      max_x = points[vertix].x;
    if(points[vertix].y > max_y)
      max_y = points[vertix].y;
    //Looking for mins
    if(points[vertix].x < min_x)
      min_x = points[vertix].x;
    if(points[vertix].y < min_y)
      min_y = points[vertix].y;
  }
  return;
} 

void draw_search_region(int& max_x, int& max_y, int& min_x, int& min_y, Mat& imagenClick)
{
  //Draw line from point (min_x,min_y) to point (max_x,miny)
  line(imagenClick,
       Point(min_x,min_y),
       Point(max_x,min_y),
       Scalar( 0,255,0));  
  //Draw line from point (max_x,min_y) to point(max_x, max_y)
  line(imagenClick,
       Point(max_x,min_y),
       Point(max_x,max_y),
       Scalar( 0,255,0));
  //Draw line from point (max_x,max_y) to point(min_x, max_y)
  line(imagenClick,
       Point(max_x,max_y),
       Point(min_x,max_y),
       Scalar( 0,255,0));
  //Draw line from point (min_x,min_y) to point(min_x, min_y)
  line(imagenClick,
       Point(min_x,max_y),
       Point(min_x,min_y),
       Scalar( 0,255,0));
  
    
  return;
}

void get_range_color(Vec3b& max,Vec3b& min, int& x_max, int& y_max, int& x_min, int& y_min,Vec3b& channels_size,Mat& imagenClick)
{
  max[0] = 0;
  min[0] = channels_size[0];
  max[1] = 0;
  min[1] = channels_size[1];
  max[2] = 0;
  min[2] = channels_size[2];

  Vec3b image_values;
  for(int x = x_min;x < x_max+1;x++){
    for(int y = y_min; y < y_max+1;y++){
        image_values = imagenClick.at<Vec3b>(x,y);
        //circle(imagenClick, Point(x,y), 1, Scalar( 0,0,255), CV_FILLED);
        //Channel 1 comparison
        if(image_values[0] > max[0])
          max[0] = image_values[0];
        if(image_values[0] < min[0])
          min[0] = image_values[0];
        //Channel 2 comparison
        if(image_values[1] > max[1])
          max[1] = image_values[1];
        if(image_values[1] < min[1])
          min[1] = image_values[1];
        //Channel 3 comparison
        if(image_values[2] > max[2])
          max[2] = image_values[2];
        if(image_values[2] < min[2])
          min[2] = image_values[2];
          
    }
  }
  return;
}

void get_histogram(Mat& histImage,Mat& plane, Scalar color,int histSize, int bin_w)
{
  Mat hist_plane;
  float range [] = {0,(float)histSize};
  const float*  histRange = {range};
  bool uniform = true;
  bool accumulate = false;
  calcHist(&plane,1,0,Mat(),hist_plane,1,&histSize,&histRange,uniform,accumulate);
  normalize(hist_plane,hist_plane,0,histImage.rows,NORM_MINMAX,-1,Mat());
  for( int i = 1; i < histSize; i++ ){
      line( histImage, Point( bin_w*(i-1), image_h - cvRound(hist_plane.at<float>(i-1)) ) ,
                       Point( bin_w*(i), image_h - cvRound(hist_plane.at<float>(i)) ),
                       color, 2, 8, 0  );
  }
  return;
}

void mark_range(Mat& histogramImage, const int& min, const int& max,int bin_w)
{
  line(histogramImage,Point(bin_w*min,0),Point(bin_w*min,image_h),Scalar(255,0,255),5,8,0);
  line(histogramImage,Point(bin_w*max,0),Point(bin_w*max,image_h),Scalar(255,0,255),5,8,0);
  return;
}
