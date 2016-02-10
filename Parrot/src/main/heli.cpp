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
Mat imagenClick;
Mat flippedImage; //Flipped image matrix declaration and function definition
//Histogram calculation variables
vector<Mat> planes;
Mat hist_plane1,hist_plane2,hist_plane3;

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
            cout << "  Mouse X, Y: " << x << ", " << y ;
            cout << endl;
            /*  Draw a point */
            points.push_back(Point(x, y));
            break;
        case CV_EVENT_MOUSEMOVE:
            break;
        case CV_EVENT_LBUTTONUP:
            break;
    }
}



int main(int argc,char* argv[])
{
  //Histogram variables
  int histSize = 256;
  float range [] = {0,256};
  const float*  histRange = {range};
  bool uniform = true;
  bool accumulate = false;
  int hist_w = 512; int hist_h = 400;
  int bin_w = cvRound( (double) hist_w/histSize );
  Mat histImage1(hist_h,hist_w,CV_8UC3,Scalar(0,0,0));
  Mat histImage2(hist_h,hist_w,CV_8UC3,Scalar(0,0,0));
  Mat histImage3(hist_h,hist_w,CV_8UC3,Scalar(0,0,0));
  namedWindow("Histogram1", CV_WINDOW_AUTOSIZE);
  namedWindow("Histogram2", CV_WINDOW_AUTOSIZE);
  namedWindow("Histogram3", CV_WINDOW_AUTOSIZE);

  //establishing connection with the quadcopter
  heli = new CHeli(); 
	
  //this class holds the image from the drone	
  image = new CRawImage(320,240);

  // Destination OpenCV Mat	
  Mat currentImage = Mat(240, 320, CV_8UC3);

  setMouseCallback("Click",mouseCoordinatesPractice1Callback);  

  //Windows to show working images	
  imshow("ParrotCam", currentImage);
  namedWindow("Click");
  namedWindow("Flipped");

  while(stop == false){
    // Clear the console
    printf("\033[2J\033[1;1H");
    cout<<"Pos X: "<<Px<<" Pos Y: "<<Py<<endl;
   
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

    //Line drawing
    imagenClick=currentImage;
    //cout << "Points size: " << points.size()<< endl;
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
    //figure showing
    imshow("Click", imagenClick);

    //Histogram calculation
    split( currentImage, planes );
    calcHist(&planes[0],1,0,Mat(),hist_plane1,1,&histSize,&histRange,uniform,accumulate);
    calcHist(&planes[1],1,0,Mat(),hist_plane2,1,&histSize,&histRange,uniform,accumulate);
    calcHist(&planes[2],1,0,Mat(),hist_plane3,1,&histSize,&histRange,uniform,accumulate);

    //Normalize histograms
    normalize(hist_plane1,hist_plane1,0,histImage1.rows,NORM_MINMAX,-1,Mat());
    normalize(hist_plane2,hist_plane2,0,histImage2.rows,NORM_MINMAX,-1,Mat());
    normalize(hist_plane3,hist_plane3,0,histImage3.rows,NORM_MINMAX,-1,Mat());

    //Draw for each channel
    for( int i = 1; i < histSize; i++ )
    {
      line( histImage1, Point( bin_w*(i-1), hist_h - cvRound(hist_plane1.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(hist_plane1.at<float>(i)) ),
                       Scalar( 255, 0, 0), 2, 8, 0  );
      line( histImage2, Point( bin_w*(i-1), hist_h - cvRound(hist_plane2.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(hist_plane2.at<float>(i)) ),
                       Scalar( 0, 255, 0), 2, 8, 0  );
      line( histImage3, Point( bin_w*(i-1), hist_h - cvRound(hist_plane3.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(hist_plane3.at<float>(i)) ),
                       Scalar( 0, 0, 255), 2, 8, 0  );
    }   

    //Display histograms
    imshow("Histogram1", histImage1);
    imshow("Histogram2", histImage2);
    imshow("Histogram3", histImage3);

	  char key = waitKey(5);
	  switch (key) {
	    case 27: stop = true; break;
	    default: ;
	  }

	  usleep(15000);
  }
	
    heli->land();
    delete heli;
	delete image;
	return 0;
}

