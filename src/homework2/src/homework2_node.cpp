#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "ros/ros.h"
#include "math.h"

using namespace std;
using namespace cv;

int KERNEL_SIZE = 9;
float IMG_SCALE = 0.5;

float dis (Point a, Point b){
  int dx = a.x-b.x;
  int dy = a.y-b.y;
  return sqrt((float)(dx*dx+dy*dy));
}


// here im basically implementing formulas with this page and sticking to their nomenclature
// http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/MANDUCHI1/Bilateral_Filtering.html
// spartial gauss
double varD = 50.0;
double gaussC (Point a, Point b){
  double tempp = (dis(a,b)/varD);
  double ret = exp(-0.5*(tempp*tempp)); 
  return ret;
}

//photometric gauss
double varR = 20.0;
double gaussS (int a, int b){
  double tempp = (fabs(a-b)/varR);
  return exp(-0.5*(tempp*tempp));
}


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

  cvtColor(RS_LIMG, GS_LIMG, CV_BGR2GRAY);

  int cols = GS_LIMG.cols;
  int rows = GS_LIMG.rows;

  Mat win;
  Mat filteredIMG(GS_LIMG.rows-KERNEL_SIZE, GS_LIMG.cols-KERNEL_SIZE, CV_8UC1, Scalar(0));
  double k = 0;
  double tempp = 0;
  double sum = 0;

  // iterate through the image 
  for(int x = KERNEL_SIZE/2; x<(cols-KERNEL_SIZE/2-1); x++){
    for(int y = KERNEL_SIZE/2; y<(rows-KERNEL_SIZE/2-1); y++){

      win = GS_LIMG(Range(y-KERNEL_SIZE/2, y+KERNEL_SIZE/2+1), Range(x-KERNEL_SIZE/2, x+KERNEL_SIZE/2+1));
      k = 0;
      sum = 0;

      // calculation of the filter outcome
      for(int u = 0; u<win.cols; u++){
        for (int v = 0; v<win.rows; v++){
          // sum+=(win.at<uchar>(v,u)-win.at<uchar>(KERNEL_SIZE/2,KERNEL_SIZE/2))*gKernel[u][v]; // debugging function
          // sum+=dis(Point(v,u), Point(KERNEL_SIZE/2,KERNEL_SIZE/2))*gKernel[u][v]; // debugging function
          tempp = gaussC(Point(u, v),Point(KERNEL_SIZE/2,KERNEL_SIZE/2)) * gaussS(win.at<uchar>(v,u), win.at<uchar>(KERNEL_SIZE/2,KERNEL_SIZE/2));
          k += tempp;
          sum += tempp*win.at<uchar>(v,u);
        }
      }

      filteredIMG.at<uchar>(y-KERNEL_SIZE/2,x-KERNEL_SIZE/2) = (int)(sum/k);

    }
  }

  Mat OrigCombined;

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


// not used
void logging(std::string&filename, std::string &text) {
    try{
        std::ofstream myfile;
        myfile.open (filename, std::ios::app);
        myfile << text << "\n";
        myfile.close();
    }
    catch (const std::exception& e) { /* */ } 
}