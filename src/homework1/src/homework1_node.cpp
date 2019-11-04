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

// function that remaps a value from range a1-a2 to b1-b2
float remap(float s, float a1, float a2, float b1, float b2){
    return b1+(s-a1)*(b2-b1)/(a2-a1);
}
 

int main(int argc, char ** argv){
  if (argc != 3){
    cout << " Usage: display_image ImageToLoadAndDisplay" << endl;
    return -1;
  }

  Mat RS_LIMG;
  Mat RS_RIMG;
  Mat LIMG;
  Mat RIMG;
  Mat GS_LIMG;
  Mat GS_RIMG;
  RS_LIMG = imread(argv[1]);
  RS_RIMG = imread(argv[2]);

  if (!RS_LIMG.data){
    cout << "Could not open or find the left image" << endl;
    return -1;
  }
  else{
    cout << "Imported the left image :)" << endl;
  }
  
  if (!RS_RIMG.data){
    cout << "Could not open or find the right image" << endl;
    return -1;
  }
  else{
    cout << "Imported the right image :)" << endl;
  }

  // resizing the images down for debugging
  resize(RS_LIMG, LIMG, cv::Size(), 0.25, 0.25);
  resize(RS_RIMG, RIMG, cv::Size(), 0.25, 0.25);

  // convert to Grayscale
  cvtColor(LIMG, GS_LIMG, CV_BGR2GRAY);
  cvtColor(RIMG, GS_RIMG, CV_BGR2GRAY);

  int cols = GS_LIMG.cols;
  int rows = GS_LIMG.rows;

  // define a vector to save disperities (pixels with lowest SSD)
  vector<vector<int> > disparities (rows-(2*KERNEL_RADIUS), vector<int>(cols-(2*KERNEL_RADIUS))); // Defaults to zero initial value
  Mat disparitiesIMG(rows-(2*KERNEL_RADIUS), cols-(2*KERNEL_RADIUS), CV_8UC1, Scalar(0));

  int minSSD;
  int currentSSD;
  int minSSDdisparity;
  int minSSDcol;

  Mat lWin(KERNEL_RADIUS*2, KERNEL_RADIUS*2, CV_8UC1, Scalar(0));
  Mat rWin(KERNEL_RADIUS*2, KERNEL_RADIUS*2, CV_8UC1, Scalar(0));
  Mat sumWin(KERNEL_RADIUS*2, KERNEL_RADIUS*2, CV_8UC1, Scalar(0));
  // Mat depth(rows-(2*KERNEL_RADIUS), cols-(2*KERNEL_RADIUS), CV_8UC1, Scalar(0));

  for(int i = KERNEL_RADIUS; i<(rows-KERNEL_RADIUS-1); i++){
    for(int j = KERNEL_RADIUS; j<(cols-KERNEL_RADIUS-1); j++){
      minSSD = 1000000;
      minSSDcol = 0;
      for(int k = KERNEL_RADIUS; k<(cols-KERNEL_RADIUS-1); k++){
        // if the pixles are too far appart, do not check for disparity
        if(abs(k-j)>MAX_DISPARITY){      
          continue;
        }

        lWin = GS_LIMG(Range(i-KERNEL_RADIUS, i+KERNEL_RADIUS), Range(j-KERNEL_RADIUS, j+KERNEL_RADIUS));
        rWin = GS_RIMG(Range(i-KERNEL_RADIUS, i+KERNEL_RADIUS), Range(k-KERNEL_RADIUS, k+KERNEL_RADIUS));
        absdiff(lWin, rWin, sumWin);
        currentSSD = sum(sumWin)[0];

        // hold track of the pixel with lowest SSD error
        if(currentSSD<minSSD){
          minSSD = currentSSD; 
          minSSDcol = k; 
        }
      }
      // save lowest disparity
      disparities[i-KERNEL_RADIUS][j-KERNEL_RADIUS] = abs(j-minSSDcol);
      disparitiesIMG.at<char>(i-KERNEL_RADIUS, j-KERNEL_RADIUS) = (int)(remap(disparities[i-KERNEL_RADIUS][j-KERNEL_RADIUS], 0, 40, 0, 255));


      // // convert disparities to depth      
      // depth.at<char>(i-KERNEL_RADIUS, j-KERNEL_RADIUS) = (FL * BL) / (units * disparities[i-KERNEL_RADIUS][j-KERNEL_RADIUS]);

      // if(i%100 == 0 && j%100 == 0){
      //   cout << ".";
      // }
    }
  }

  Mat depthIMG(disparities.size(), disparities[0].size(), CV_8UC1, Scalar(0));

  for(int x = 0; x<disparities.size(); x++){
    for(int y=0; y<disparities[0].size(); y++){
      depthIMG.at<char>(x,y) = disparities[x][y];
    }
  }


  Mat OrigCombined;
  Mat scaledLIMG;
  Mat scaledRIMG;
  Mat scaledTestIMG;
    
  hconcat(LIMG,RIMG,OrigCombined);

  int key;
  while (1){
    key = waitKey(30);

    if (key == 27){
      // if "esc" is pressed end the program
      std::cout << "Closing the program because esc pressed \n";
      break;
    }
    imshow("original", OrigCombined);
    imshow("test", disparitiesIMG);

    
  }

  return 0;
}