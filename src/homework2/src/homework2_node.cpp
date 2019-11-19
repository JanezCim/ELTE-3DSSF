#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "ros/ros.h"
#include "math.h"

using namespace std;
using namespace cv;

// ***************************CHANGABLES

int KERNEL_SIZE = 9; //this should be an odd number
float IMG_SCALE = 0.5;

vector<double > variancesD {30.0, 40.0, 50.0, 60.0};
vector<double > variancesR {10.0, 20.0, 30.0, 40.0};

//**************************************



// function that calculates distance btw point a and b
float dis (Point a, Point b){
  int dx = a.x-b.x;
  int dy = a.y-b.y;
  return sqrt((float)(dx*dx+dy*dy));
}

// here im basically implementing formulas with this page and sticking to their nomenclature
// source [1]: http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/MANDUCHI1/Bilateral_Filtering.html
// spartial gauss
double gaussC (Point a, Point b, double varD){
  double tempp = (dis(a,b)/varD);
  double ret = exp(-0.5*(tempp*tempp)); 
  return ret;
}

//spectral gauss see source [1]
double gaussS (int a, int b, double varR){
  double tempp = (fabs(a-b)/varR);
  return exp(-0.5*(tempp*tempp));
}

void BilateralFilter(Mat& imgIN,
                     Mat& imgOUT,
                     int ker_size,
                     double varianceR,
                     double varianceD){
  int cols = imgIN.cols;
  int rows = imgIN.rows;

  Mat win;
  Mat OUT(imgIN.rows-ker_size, imgIN.cols-ker_size, CV_8UC1, Scalar(0));
  double k = 0;
  double tempp = 0;
  double sum = 0;

  // iterate through the image 
  for(int x = ker_size/2; x<(cols-ker_size/2-1); x++){
    for(int y = ker_size/2; y<(rows-ker_size/2-1); y++){

      // extracting the kernel arround the current pixle
      win = imgIN(Range(y-ker_size/2, y+ker_size/2+1), Range(x-ker_size/2, x+ker_size/2+1));
      k = 0;
      sum = 0;

      // calculation of the filter summed up outcome for one kernel
      for(int u = 0; u<win.cols; u++){
        for (int v = 0; v<win.rows; v++){
          // for explanation of these formulas see link [1]
          tempp = gaussC(Point(u, v),Point(ker_size/2,ker_size/2), varianceD) *
                  gaussS(win.at<uchar>(v,u), win.at<uchar>(ker_size/2,ker_size/2), varianceR);
          k += tempp;
          sum += tempp*win.at<uchar>(v,u);
        }
      }

      // normalize and save the filtered pixle
      OUT.at<uchar>(y-ker_size/2,x-ker_size/2) = (int)(sum/k);
    }
  }

  imgOUT = OUT;
}

//**********************************************************MAIN****************************************************

int main(int argc, char ** argv){
  if (argc < 2){
    cout << " Usage: display_image ImageToLoadAndDisplay" << endl;
    return -1;
  }

  Mat RS_LIMG;
  Mat GS_LIMG;
  RS_LIMG = imread(argv[1]);

  if (!RS_LIMG.data){
    cout << "Could not open or find the left image" << endl;
    return -1;
  }
  else{
    cout << "Imported the left image :)" << endl;
  }

  // resizing the images down for debugging
  resize(RS_LIMG, RS_LIMG, cv::Size(), IMG_SCALE, IMG_SCALE);

  // conver to grayscale
  cvtColor(RS_LIMG, GS_LIMG, CV_BGR2GRAY);

  Mat filteredIMG;

  for(double r:variancesR){
    for(double d:variancesD){
      BilateralFilter(GS_LIMG, filteredIMG, KERNEL_SIZE, r, d);
      imwrite( "./././filtered_r"+to_string((int)r)+"_d"+to_string((int)d)+".jpg", filteredIMG);
    }
  }


  int key;
  while (1){
    key = waitKey(30);

    if (key == 27){
      // if "esc" is pressed end the program
      std::cout << "Closing the program because esc pressed \n";
      break;
    }
    imshow("original", GS_LIMG);
    imshow("filtered", filteredIMG);
    
  }

  return 0;
}