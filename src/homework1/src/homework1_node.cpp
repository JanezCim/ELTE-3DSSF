#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "ros/ros.h"

using namespace std;
using namespace cv;

// focal lenght in px
int FL = 3740;

// baseline (dist btw cameras) in mm
float BL = 160.0;

// window size () in px to convolute with
int KERNEL_RADIUS = 8;

// TODO
float units  = 0.001;

// number of disparities to consider
int MAX_DISPARITY = 64;

int main(int argc, char ** argv){
  if (argc != 3){
    cout << " Usage: display_image ImageToLoadAndDisplay" << endl;
    return -1;
  }

  Mat LIMG;
  Mat RIMG;
  Mat GS_LIMG;
  Mat GS_RIMG;
  LIMG = imread(argv[1]);
  RIMG = imread(argv[2]);

  if (!LIMG.data){
    cout << "Could not open or find the left image" << endl;
    return -1;
  }
  else{
    cout << "Imported the left image :)" << endl;
  }
  
  if (!RIMG.data){
    cout << "Could not open or find the right image" << endl;
    return -1;
  }
  else{
    cout << "Imported the right image :)" << endl;
  }

  // convert to Grayscale
  cvtColor(LIMG, GS_LIMG, CV_BGR2GRAY);
  cvtColor(RIMG, GS_RIMG, CV_BGR2GRAY);

  int cols = GS_LIMG.cols;
  int rows = GS_LIMG.rows;

  // define a vector to save disperities (pixels with lowest SSD)
  vector<vector<int> > disparities (rows-(2*KERNEL_RADIUS), vector<int>(cols-(2*KERNEL_RADIUS))); // Defaults to zero initial value
  
  int minSSD;
  int currentSSD;
  int minSSDdisparity;

  Mat testIMG(rows, cols, CV_8UC1, Scalar(0));
  // iterate through image pixles and ignore the borders of width KERNEL_RADIUS
  for(int i = KERNEL_RADIUS; i<(rows-KERNEL_RADIUS-1); i++){
    minSSD = 100000;
    minSSDdisparity = -1;
    for(int j = KERNEL_RADIUS; j<(cols-KERNEL_RADIUS-1); j++){
      // iterating through the columns of the same row for right image
      for(int k = KERNEL_RADIUS; k<(cols-KERNEL_RADIUS-1); k++){
        currentSSD = 0;
      // calculate SSD in a current kernel
        for(int u = -KERNEL_RADIUS; u<KERNEL_RADIUS-1; u++){
          for(int v = -KERNEL_RADIUS; v<KERNEL_RADIUS-1; v++){
            currentSSD += pow(GS_LIMG.at<char>(u+i,v+j) - GS_RIMG.at<char>(u+i,v+k),2);
          }
        }

        // check if SSD for current pixel is smaller then the smallest known  
        if(currentSSD<minSSD){
          // save the values if its found
          minSSD = currentSSD;
          minSSDdisparity = abs(k-j);
        }
      }
      disparities[i][j] = minSSDdisparity;
    }    
  }




  Mat scaledOrigCombined;
  Mat scaledLIMG;
  Mat scaledRIMG;
  Mat scaledTestIMG;
  
  resize(LIMG, scaledLIMG, cv::Size(), 0.25, 0.25);
  resize(RIMG, scaledRIMG, cv::Size(), 0.25, 0.25);

  resize(testIMG, scaledTestIMG, cv::Size(), 0.25, 0.25);
  
  hconcat(scaledLIMG,scaledRIMG,scaledOrigCombined);

  int key;
  while (1){
    key = waitKey(30);

    if (key == 27){
      // if "esc" is pressed end the program
      std::cout << "Closing the program because esc pressed \n";
      break;
    }
    imshow("original", scaledOrigCombined);
    imshow("test", scaledTestIMG);

    
  }

  return 0;
}