#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "ros/ros.h"

using namespace std;
using namespace cv;

std::string file = "/home/janez/Desktop/3d.xyz";

// focal lenght in px
double FL = 3740.0;

// baseline (dist btw cameras) in mm
double BL = 160.0;

// window size () in px to convolute with
int KERNEL_RADIUS = 8;

// number of disparities to consider
int MAX_DISPARITY = 20;

void logging(std::string&filename, std::string &text) {
    try{
        std::ofstream myfile;
        myfile.open (filename, std::ios::app);
        myfile << text << "\n";
        myfile.close();
    }
    catch (const std::exception& e) { /* */ } 
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

  int minSSD;
  int currentSSD;
  int minSSDdisparity;
  int minSSDcol;

  Mat lWin(KERNEL_RADIUS*2, KERNEL_RADIUS*2, CV_8UC1, Scalar(0));
  Mat rWin(KERNEL_RADIUS*2, KERNEL_RADIUS*2, CV_8UC1, Scalar(0));
  Mat sumWin(KERNEL_RADIUS*2, KERNEL_RADIUS*2, CV_8UC1, Scalar(0));
  // Mat depth(rows-(2*KERNEL_RADIUS), cols-(2*KERNEL_RADIUS), CV_8UC1, Scalar(0));

  double x = 0;
  double y = 0;
  double z = 0;

  for(int i = KERNEL_RADIUS; i<(rows-KERNEL_RADIUS); i++){
    for(int j = KERNEL_RADIUS; j<(cols-KERNEL_RADIUS); j++){
      minSSD = 1000000;
      minSSDcol = 0;
      for(int k = KERNEL_RADIUS; k<(cols-KERNEL_RADIUS); k++){
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
      int disp = abs(j-minSSDcol);
      int pos_disp = j+minSSDcol;
      disparities[i-KERNEL_RADIUS][j-KERNEL_RADIUS] = disp;
      
      if(disp!=0){
        x = -(BL*(double)pos_disp)/(2*(double)disp);
        y = BL*(double)i/(double)disp;
        z = BL*FL/((double)disp*10.0);
      }
      else{
        x = 0;
        y = 0;
        z = 0;
      }

      // logging
      std::string text = std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " 0 0 0";
      logging(file, text);
    }
  }

  Mat disparitiesIMG(disparities.size(), disparities[0].size(), CV_8UC1, Scalar(0));
  vector<vector<double> > depths (rows-(2*KERNEL_RADIUS), vector<double>(cols-(2*KERNEL_RADIUS))); // Defaults to zero initial value


  // resize disparities and calculate depths
  for(int x = 0; x<disparities.size(); x++){
    for(int y=0; y<disparities[0].size(); y++){
      disparitiesIMG.at<char>(x,y) = disparities[x][y]*(int)(254/MAX_DISPARITY);
    }
  }


  Mat OrigCombined;

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

