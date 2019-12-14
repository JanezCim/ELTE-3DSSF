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


// ************************************



// function that calculates distance btw point a and b
float dis (Point a, Point b){
  int dx = a.x-b.x;
  int dy = a.y-b.y;
  return sqrt((float)(dx*dx+dy*dy));
}


// **********************************************************MAIN****************************************************

int main(int argc, char ** argv){
  

  if (argc < 2){
    cout << " Usage: display_image ImageToLoadAndDisplay" << endl;
    return -1;
  }

  // import the rgb image
  Mat RS_IMG;
  Mat GS_IMG;
  RS_IMG = imread(argv[1]);

  // import depth image¸
  Mat DEPTH_IMG;
  DEPTH_IMG = imread(argv[2]);

  if (!RS_IMG.data){
    cout << "Could not open or find the first image" << endl;
    return -1;
  }
  else{
    cout << "Imported the first image :)" << endl;
  }

  if (!DEPTH_IMG.data){
    cout << "Could not open or find the depth image" << endl;
    return -1;
  }
  else{
    cout << "Imported the depth image :)" << endl;
  }


  int key;
  while (1){
    key = waitKey(30);

    if (key == 27){
      // if "esc" is pressed end the program
      std::cout << "Closing the program because esc pressed \n";
      break;
    }
    imshow("original", GS_IMG);
    imshow("unfiltered depth", DEPTH_IMG);
    // imshow("filtered", resultingIMG); 
  }
  return 0;
}