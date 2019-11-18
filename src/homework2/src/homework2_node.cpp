#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "ros/ros.h"

using namespace std;
using namespace cv;

int KERNEL_SIZE = 5;
float IMG_SCALE = 0.5;

// Function to create Gaussian filter
// Source: https://www.geeksforgeeks.org/gaussian-filter-generation-c/
void FilterCreation(double GKernel[][5]) 
{ 
    // intialising standard deviation to 1.0 
    double sigma = 1.0; 
    double r, s = 2.0 * sigma * sigma; 
  
    // sum is for normalization 
    double sum = 0.0; 
  
    // generating 5x5 kernel 
    for (int x = -2; x <= 2; x++) { 
        for (int y = -2; y <= 2; y++) { 
            r = sqrt(x * x + y * y); 
            GKernel[x + 2][y + 2] = (exp(-(r * r) / s)) / (M_PI * s); 
            sum += GKernel[x + 2][y + 2]; 
        } 
    } 
  
    // normalising the Kernel 
    for (int i = 0; i < 5; ++i) 
        for (int j = 0; j < 5; ++j) 
            GKernel[i][j] /= sum; 
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

  double gKernel[KERNEL_SIZE][5]; 
  FilterCreation(gKernel); 

  int cols = GS_LIMG.cols;
  int rows = GS_LIMG.rows;

  Mat win;
  Mat filteredIMG(GS_LIMG.rows-KERNEL_SIZE, GS_LIMG.cols-KERNEL_SIZE, CV_8UC1, Scalar(0));
  double sum = 0;

  // iterate through the image 
  for(int x = KERNEL_SIZE/2; x<(cols-KERNEL_SIZE/2-1); x++){
    for(int y = KERNEL_SIZE/2; y<(rows-KERNEL_SIZE/2-1); y++){

      win = GS_LIMG(Range(y-KERNEL_SIZE/2, y+KERNEL_SIZE/2+1), Range(x-KERNEL_SIZE/2, x+KERNEL_SIZE/2+1)); //TODO are these ranges ok?
      sum = 0;

      // calculation of the filter outcome
      for(int u = 0; u<win.cols; u++){
        for (int v = 0; v<win.rows; v++){
          sum+=win.at<uchar>(v,u)*gKernel[u][v];
        }
      }

      filteredIMG.at<uchar>(y-KERNEL_SIZE/2,x-KERNEL_SIZE/2) = (int)sum;

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