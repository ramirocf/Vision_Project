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

//Color spaces
const unsigned char bgr = 0;
const unsigned char xyz = 0x01;
const unsigned char ycrcb = 0x02;
const unsigned char hsv = 0x03;
const unsigned char hls = 0x04;
const unsigned char luv = 0x05;
unsigned char color_spaces[] = {bgr,xyz,ycrcb,hsv,hls,luv};
const unsigned char num_color_spaces = 6;
//Parameter for bgr
unsigned char blue = 0; unsigned char blue_size = 0xFF;
unsigned char green = 0x01; unsigned char green_size = 0xFF;
unsigned char red = 0x02; unsigned char red_size = 0xFF;
//Parameter for xyz
unsigned char x = 0; unsigned char x_size = 0xFF;
unsigned char y = 0x01; unsigned char y_size = 0xFF;
unsigned char z = 0x02; unsigned char z_size = 0xFF;
//Parameters for yCrCb
unsigned char ycrcb_y = 0; unsigned char ycrcb_y_size = 0;
unsigned char cr = 0x01; unsigned char cr_size = 0xFF;
unsigned char cb = 2; unsigned char cb_size = 0xFF;
//Parameters for hsv
unsigned char hsv_h = 0; unsigned char hsv_h_size = 180;
unsigned char hsv_s = 1; unsigned char hsv_s_size = 255;
unsigned char hsv_v = 2; unsigned char hsv_v_size = 255;
//Parameters for hls
unsigned char hls_h = 0; unsigned char hls_h_size = 180;
unsigned char hls_l = 1; unsigned char hls_l_size = 255;
unsigned char hls_s = 2; unsigned char hls_s_size = 255;
//Parameters for luv
unsigned char luv_l = 0; unsigned char luv_l_size = 255;
unsigned char luv_u = 1; unsigned char luv_u_size = 255;
unsigned char luv_v = 2; unsigned char luv_v_size = 255; 

bool stop = false;
CRawImage *image;
CHeli *heli;
int Px;
int Py;
vector<Point> points;
int image_h = 240;
int image_w = 320;
Mat currentImage;
Mat flippedImage;
Mat imageFiltered;
vector<Mat> spacesImage(num_color_spaces);
Mat bgrImage,xyzImage,ycbcrImage,hsvImage,hlsImage,luvImage;
Mat current_space_image;
//Histogram calculation variables
vector<Mat> bgrPlane,xyzPlane,ycrcbPlane,hsvPlabe,hlsPlane,luvPlane;
vector< vector<Mat> > spaces_planes(num_color_spaces);
vector<Mat> bgrHistogram,xyzHistogram,ycrcbHistogram,hsvHistogram,hlsHistogram,luvHistogram;
vector< vector<Mat> > spaces_histograms(num_color_spaces, vector<Mat>(3));
vector<Mat> current_hist_image(3);
Mat hist1(image_h,image_w,CV_8UC3,Scalar(0,0,0));
Mat hist2(image_h,image_w,CV_8UC3,Scalar(0,0,0));
Mat hist3(image_h,image_w,CV_8UC3,Scalar(0,0,0));
vector<Mat> current_planes;
bool shape_selected = false;
bool shape_complete = false;


void filter_option(int& max_x,int& max_y, int& min_x, int& min_y,Vec3b& max_color, Vec3b& min_color, Vec3b& channels_size,
                   Mat& imageFiltered, const int& histSize, Vec3b& bin_w, const unsigned char& space,Mat& imagenClick,
                   vector<string>& active_windows);
void flip_option(vector<string>& active_windows);
void binarize_option(Mat& binarizeImage,vector<string>& active_windows,const unsigned char& treshold);
void flipImageBasic(const Mat &sourceImage, Mat &destinationImage);
void rawToMat( Mat &destImage, CRawImage* sourceImage);
void mouseCoordinatesPractice1Callback(int event, int x, int y, int flags, void* param);
void draw_shape(Mat& imagen);
void get_range(Vec3b& max, Vec3b& min, Mat& imagen);
void get_range_coordinates(int& max_x, int& max_y, int& min_x, int& min_y);
void draw_search_region(int& max_x, int& max_y, int& min_x, int& min_y, Mat& imagen);
void get_range_color(Vec3b& max,Vec3b& min, int& max_x, int& max_y, int& min_x, int& min_y,Vec3b& channels_size,Mat& imagen);
void get_histogram(Mat& histImage,Mat& plane,unsigned char channel,int histSize,int bin_w);
void mark_range(Mat& histogramImage, const int& min, const int& max,int bin_w);
void filter_image(Mat& imageFiltered, Vec3b& max, Vec3b& min );
void add_window(vector<string>& active_windows, string new_window);
void delete_active_windows(vector<string>& active_windows);
void transform_to_gray(Mat& source, Mat& destination);
void apply_treshold(Mat& image2Filter, const unsigned char& treshold);


int main(int argc,char* argv[])
{
  //Program option
  Mat imagenClick(image_h,image_w,CV_8UC3,Scalar(0,0,0));
  unsigned char option = 0;
  const unsigned char clear_key = 99;
  const unsigned char stop_key = 27;
  const unsigned char filter_key = 102;
  const unsigned char flip_key = 114;
  const unsigned char binarize_key = 98;
  const unsigned char idle = 0;
  const unsigned char filter = 0x01;
  const unsigned char flip = 0x03;
  const unsigned char binarize = 0x04;
  unsigned char space_option;
  unsigned char treshold;
  char key_treshold;
  char key_space;
  //Image variables
  Mat binarizeImage(image_h,image_w,CV_8U);
  vector<string> active_windows;
  int max_x,max_y,min_x,min_y;
  Vec3b max_color,min_color;
  Vec3b channels_size;
  spacesImage.push_back(bgrImage);
  spacesImage.push_back(xyzImage);
  spacesImage.push_back(ycbcrImage);
  spacesImage.push_back(hsvImage);
  spacesImage.push_back(hlsImage);
  spacesImage.push_back(luvImage);
  //Histogram variables
  int histSize = 256;
  Vec3b bin_w;
  Mat histImage1(image_h,image_w,CV_8UC3,Scalar(0,0,0));
  Mat histImage2(image_h,image_w,CV_8UC3,Scalar(0,0,0));
  Mat histImage3(image_h,image_w,CV_8UC3,Scalar(0,0,0));

  //establishing connection with the quadcopter
  heli = new CHeli(); 
	
  //this class holds the image from the drone	
  image = new CRawImage(320,240);

  // Destination OpenCV Mat	
  currentImage = Mat(240, 320, CV_8UC3);  
  namedWindow("Click");

  while(stop == false){
    // Clear the console
    printf("\033[2J\033[1;1H");		

    //Capture image and transform it to manipulate it
    heli->renewImage(image);
    rawToMat(currentImage, image);

    //User options
    cout <<"PROGRAM FOR FIRST PROJECT OF COMPUTER VISION"<<endl;
    cout << "Options: " << endl;
    cout << "esc: Get out of here" << endl;
    cout << "f: This option allows to filter a region in the image captured with the dron's camera.";
    cout << "Once pressed you need to draw a region using the mouse to select the points delimiting the region.";
    cout << "Once the points has been selected please click rigth button.";
    cout << "The program will generate the filtered image and histograms for the following color spaces. User needs to select one of them: \n";
    cout << "    0-RGB\n    1-CIE XYZ\n    2-YCrCb\n    3-HSV\n    4-HLS\n    5-CIE LUV\n";
    cout << "c: Clear data and start again" << endl;
    char key = waitKey(5);
      switch (key) {
        case stop_key:   
                   stop = true; 
                   break;

        case filter_key:  
                   option = filter;
                   cout << "Press key of color model" << endl;
                   key_space = waitKey();
                   space_option = key_space - 0x30;
                   break;

        case flip_key:
                   option = flip;
                   break;

        case binarize_key:
                   option = binarize;
                   treshold = 0;
                   cout << "Introduce treshold value as a three digit decimal number in the range 0 to 255" << endl;
                   key_treshold = waitKey();
                   treshold = (key_treshold - 48) * 100;
                   key_treshold = waitKey();
                   treshold += (key_treshold - 48) * 10;
                   key_treshold = waitKey();
                   treshold += key_treshold - 48;
                   break;

        case clear_key:   
                   option = idle;
                   shape_complete = false;
                   points.clear();
                   delete_active_windows(active_windows);
                   break;
	default: ;
      }
    //Working image assignation
    imagenClick = currentImage.clone();
    //Based on user input do an activity
    switch (option) {
      case idle:     imshow("Click", imagenClick);
                     usleep(15000);
                     break;

      case filter :  filter_option(max_x,max_y,min_x,min_y,max_color,min_color,channels_size,imageFiltered,histSize,
                                   bin_w,space_option,imagenClick,active_windows);
                     usleep(15000);
                     break;
    
      case flip   :
                     flip_option(active_windows);
                     usleep(15000);
                     break;

      case binarize:
                     cout << "Treshold value: " << (int)treshold <<endl;
                     binarize_option(binarizeImage,active_windows,treshold);
                     usleep(15000);
                     break;

      default     :  imshow("Click", imagenClick);
                     usleep(15000);
    
    }
    //If the user haven't selected any image to filter why bothering the system doing histograms
    
  }
	
    heli->land();
    delete heli;
    delete image;
    return 0;
}

void filter_option(int& max_x,int& max_y, int& min_x, int& min_y,Vec3b& max_color, Vec3b& min_color, Vec3b& channels_size,
                   Mat& imageFiltered, const int& histSize, Vec3b& bin_w,const unsigned char& space,Mat& imagenClick,vector<string>& active_windows)
{
    string space_str ,channel0,channel1,channel2,hist1_name,hist2_name,hist3_name;
    int conv_code;
    setMouseCallback("Click",mouseCoordinatesPractice1Callback);
    
    //Line drawing
    draw_shape(imagenClick);
    //figure showing after drawing
    imshow("Click", imagenClick);
    //If user haven't complety defined shape, why computing histograms?
    if(!shape_complete){
      return;
    }
    //Getting maximum and minimum values for coordinates of the selected region to establish a searching region
    get_range_coordinates(max_x,max_y,min_x,min_y);
    draw_search_region(max_x,max_y,min_x,min_y,imagenClick);
    imshow("Click", imagenClick);
    
    //Now it is time to filter the image for each color space
      switch (space){
        case bgr:    channels_size[blue] = blue_size;
                     channels_size[green] = green_size;
                     channels_size[red] = red_size;
                     space_str = "RGB";channel0 = "Blue"; channel1 = "Green"; channel2 = "Red";
                     current_space_image = currentImage;
                     break;

        case xyz:    channels_size[x] = x_size;
                     channels_size[y] = y_size;
                     channels_size[z] = z_size;
                     conv_code = CV_XYZ2BGR;
                     cvtColor(currentImage,spacesImage[xyz],CV_BGR2XYZ);
                     space_str = "CIE XYZ"; channel0 = "X"; channel1 = "Y"; channel2 = "Z";
                     current_space_image = spacesImage[xyz];
                     break;

        case ycrcb:  channels_size[ycrcb_y] = ycrcb_y_size;
                     channels_size[cr] = cr_size;
                     channels_size[cb] = cb_size;
                     conv_code = CV_YCrCb2BGR;
                     cvtColor(currentImage,spacesImage[ycrcb],CV_BGR2YCrCb);
                     space_str = "YCrCb"; channel0 = "Y"; channel1 = "Cr"; channel2 = "Cb";
                     current_space_image = spacesImage[ycrcb];
                     break;

        case hsv:    channels_size[hsv_h] = hsv_h_size;
                     channels_size[hsv_s] = hsv_s_size;
                     channels_size[hsv_v] = hsv_v_size;
                     conv_code = CV_HSV2BGR;
                     cvtColor(currentImage,spacesImage[hsv],CV_BGR2HSV);
                     space_str = "HSV"; channel0 = "Hue"; channel1 = "Saturation"; channel2 = "Value";
                     current_space_image = spacesImage[hsv];
                     break;

        case hls:    channels_size[hls_h] = hls_h_size;
                     channels_size[hls_l] = hls_l_size;
                     channels_size[hls_s] = hls_s_size;
                     conv_code = CV_HLS2BGR;
                     cvtColor(currentImage,spacesImage[hls],CV_BGR2HLS);
                     space_str = "HLS"; channel0 = "Hue"; channel1 = "Lightness"; channel2 = "Saturation";
                     current_space_image = spacesImage[hls];
                     break;

        case luv:    channels_size[luv_l] = luv_l_size;
                     channels_size[luv_u] = luv_u_size;
                     channels_size[luv_v_size] = luv_v_size;
                     conv_code = CV_Luv2BGR;
                     cvtColor(currentImage,spacesImage[luv],CV_BGR2Luv);
                     space_str = "CIE LUV "; channel0 = "Ligthness"; channel1 = "Chrominance component U"; channel2 = "Chrominance component V";
                     current_space_image = spacesImage[luv];
                     break;
      }
      get_range_color(max_color,min_color,max_x,max_y,min_x,min_y,channels_size,current_space_image);
      imshow("Click",imagenClick);
      //Histogram calculation
      split( current_space_image, current_planes );
      bin_w[0] = cvRound((double) image_w/channels_size[0]);
      hist1 = Scalar(0,0,0);
      current_hist_image[0] = hist1;
      get_histogram(current_hist_image[0],current_planes[0],blue,(int)channels_size[0]+1,bin_w[0]);
      bin_w[1] = cvRound((double) image_w/channels_size[1]);
      hist2 = Scalar(0,0,0);
      current_hist_image[1] = hist2;         
      get_histogram(current_hist_image[1],current_planes[1],green,(int)channels_size[1]+1,bin_w[1]);
      bin_w[2] = cvRound((double) image_w/channels_size[0]);
      hist3 = Scalar(0,0,0);
      current_hist_image[2] = hist3;
      get_histogram(current_hist_image[2],current_planes[2],red,(int)channels_size[2]+1,bin_w[2]);

      //Mark selected range for filtering
      mark_range(current_hist_image[0],(int)max_color[0],(int)min_color[0],bin_w[0]);
      mark_range(current_hist_image[1],(int)max_color[1],(int)min_color[1],bin_w[0]);
      mark_range(current_hist_image[2],(int)max_color[2],(int)min_color[2],bin_w[0]);

      //Display histograms
      hist1_name = "Histogram for " + space_str + " -- " + channel0;
      namedWindow(hist1_name, CV_WINDOW_AUTOSIZE);
      imshow(hist1_name, current_hist_image[0]);
      add_window(active_windows,hist1_name);
      hist2_name = "Histogram for " + space_str + " -- " + channel1;      
      namedWindow(hist2_name, CV_WINDOW_AUTOSIZE);
      imshow(hist2_name, current_hist_image[1]);
      add_window(active_windows,hist2_name);
      hist3_name = "Histogram for " + space_str + " -- " + channel2;
      namedWindow(hist3_name, CV_WINDOW_AUTOSIZE);
      imshow(hist3_name, current_hist_image[2]);
      add_window(active_windows,hist3_name);

    //Filter image according to the ranges found above
    imageFiltered = current_space_image.clone();
    filter_image(imageFiltered,max_color,min_color);
    //Display image before filtering
    //First it is neccesary to convert the image to bgr format
    if(space != bgr)
      cvtColor(current_space_image,current_space_image,conv_code);
    namedWindow("Image before filtering",CV_WINDOW_AUTOSIZE);    
    imshow("Image before filtering", current_space_image);
    add_window(active_windows,"Image before filtering");
    //Now it's tme display filtered image
    //Again we need to convert image
    if(space != bgr)
      cvtColor(imageFiltered,imageFiltered,conv_code);
    namedWindow("Filtered image for space " + space_str,CV_WINDOW_AUTOSIZE);
    imshow("Filtered image for space " + space_str,imageFiltered);
    add_window(active_windows,"Filtered image for space " + space_str);
 return; 
}

void flip_option(vector<string>& active_windows)
{
  flipImageBasic(currentImage,flippedImage);
  namedWindow("Flipped Image");
  imshow("Flipped Image",flippedImage);
  add_window(active_windows,"Flipped Image");
  return;
}

void binarize_option(Mat& binarizeImage,vector<string>& active_windows,const unsigned char& treshold)
{
  string window_name;
  transform_to_gray(currentImage,binarizeImage);
  window_name = "Image in gray scale before treshold";
  namedWindow(window_name);
  add_window(active_windows,window_name);
  imshow(window_name,binarizeImage);
  apply_treshold(binarizeImage,treshold);
  window_name = "Image in gray scale after treshold";
  namedWindow(window_name);
  add_window(active_windows,window_name);
  imshow(window_name,binarizeImage);
  return;
}
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

//Mouse coordinate displaying function
void mouseCoordinatesPractice1Callback(int event, int x, int y, int flags, void* param)
{
    switch (event)
    {
        case CV_EVENT_LBUTTONDOWN:
              
              if(!shape_complete){
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


void get_range_coordinates(int& max_x, int& max_y, int& min_x, int& min_y)
{
  max_x = 0;
  max_y = 0;
  min_x = image_w;
  min_y = image_h;
  //Iterare over each triangle of the selected shape
  for(int vertix=0; vertix < (int)points.size()-1; vertix++){
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

void get_range_color(Vec3b& max,Vec3b& min, int& x_max, int& y_max, int& x_min, int& y_min,Vec3b& channels_size,Mat& imagen)
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
        image_values = imagen.at<Vec3b>(x,y);
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

void get_histogram(Mat& histImage,Mat& plane, unsigned char channel,int histSize, int bin_w)
{
  Mat hist_plane;
  float range [] = {0,(float)histSize};
  const float*  histRange = {range};
  bool uniform = true;
  bool accumulate = false;
  Scalar color = Scalar(0,0,0);
  calcHist(&plane,1,0,Mat(),hist_plane,1,&histSize,&histRange,uniform,accumulate);
  normalize(hist_plane,hist_plane,0,histImage.rows,NORM_MINMAX,-1,Mat());
  for( int i = 1; i < histSize; i++ ){
      color[channel] = i;
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

void filter_image(Mat& image2Filter, Vec3b& max, Vec3b& min)
{
 Vec3b value;
 //Iterate over the whole image in order to eliminate those values outside the range
 for(int x=0; x < image2Filter.rows; x++)
    for(int y=0; y < image2Filter.cols; y++){
      value = image2Filter.at<Vec3b>(x,y);
      //Filter channel 0
      if(value[0] < min[0] || value[0] > max[0])
        value[0] = 0;
      //Filter channel 1
      if(value[1] < min[1] || value[1] > max[1])
        value[1] = 0;
      //Filter channel 2
      if(value[2] < min[2] || value[2] > max[2])
        value[2] = 0;
      image2Filter.at<Vec3b>(x,y) = value;
    }
 return;
}

void add_window(vector<string>& active_windows, string new_window)
{
 //Iterate over active windows first to verify if window is still in the vector. We don't want to add thousand of times the same windows !!!!!
  bool present = false;
  for(unsigned int it=0; it < active_windows.size();it++){
    if(active_windows[it] == new_window){
      present = true;
      break;
    }
  }
  if(!present){
    active_windows.push_back(new_window); 
  }
 return;
}

void delete_active_windows(vector<string>& active_windows)
{
  for(unsigned int it=0;it<active_windows.size();it++){
    destroyWindow(active_windows[it]);
  }
  active_windows.clear();
  return;
}

void transform_to_gray(Mat& source, Mat& destination)
{
  //Iterate over the whole image to add up individual channel values
  for(int x=0;x<source.rows;x++){
    for(int y=0;y<source.cols;y++){
        destination.at<unsigned char>(x,y) = source.at<Vec3b>(x,y)[0]/3;
        destination.at<unsigned char>(x,y) += source.at<Vec3b>(x,y)[1]/3;
        destination.at<unsigned char>(x,y) += source.at<Vec3b>(x,y)[2]/3; 
    }
  }
 return;
}

void apply_treshold(Mat& image2Filter, const unsigned char& treshold)
{
 //Iterate over the whole image
   for(int x=0;x<image2Filter.rows;x++){
    for(int y=0;y<image2Filter.cols;y++){
       if(image2Filter.at<unsigned char>(x,y) < treshold){
         image2Filter.at<unsigned char>(x,y) = 0;
       } 
    }
  }
 return;
}
