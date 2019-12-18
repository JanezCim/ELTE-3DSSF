#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "ros/ros.h"
#include "math.h"
#include "homework3/nanoflann.hpp"

using namespace std;
using namespace cv;
using namespace nanoflann;

// ***************************CHANGABLES


// ************************************



// function that calculates distance btw point a and b
float dis (Point a, Point b){
  int dx = a.x-b.x;
  int dy = a.y-b.y;
  return sqrt((float)(dx*dx+dy*dy));
}


// **********************************************************MAIN****************************************************


int import3dPointsFromFile(string file_path, vector<Point3d >& out_points){
  try{
    ifstream file(file_path);
    vector<Point3d > tmpv;
    Point3d tmp;

    double meh;
     
    while (file >> tmp.x && file >> tmp.y && file >> tmp.z && file >> meh && file >> meh && file >> meh){
      // add a copy of tmp to points
      tmpv.push_back(tmp);
    }
    file.close();
    out_points = tmpv;
    return 1;
  }
  catch (const std::exception& e) {
    cout << "error opening the xyz file" << endl;
    return -1;
  } 
}

int main(int argc, char ** argv){
  

  if (argc < 3){
    cout << " Usage: display_image ImageToLoadAndDisplay" << endl;
    return -1;
  }
  
  // the containers in which we will store all the points
  vector<Point3d> points1;
  vector<Point3d> points2;

  if(!import3dPointsFromFile(argv[1], points1)){
    return 0;
  }
  
  if(!import3dPointsFromFile(argv[2], points2)){
    return 0;
  }



  // int key;
  // while (1){
  //   key = waitKey(30);

  //   if (key == 27){
  //     // if "esc" is pressed end the program
  //     std::cout << "Closing the program because esc pressed \n";
  //     break;
  //   }
  //   // imshow("original", GS_IMG);
  //   // imshow("unfiltered depth", DEPTH_IMG);
  // }
  return 0;
}