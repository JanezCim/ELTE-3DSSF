#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "ros/ros.h"

using namespace std;
using namespace cv;

// focal lenght in px
double FL = 3740.0;

// baseline (dist btw cameras) in mm
double BL = 160.0;

// window radius (its a square) in px to convolute with
int KERNEL_RADIUS = 16;

// number of disparities to consider
int MAX_DISPARITY = 40;

int PX_PER_MM = 10;

// scale the image before computing on it (faster computation for debugging)
double IMG_SCALE = 0.25;

// file to output the xyz file to
std::string file = "3d_win" + to_string(KERNEL_RADIUS*2) + "_max_disp" + to_string(MAX_DISPARITY) + "_img_scale" + to_string(IMG_SCALE) + ".xyz";

int main(int argc, char ** argv){
  if (argc < 3){
    cout << " Usage: display_image ImageToLoadAndDisplay" << endl;
    return -1;
  }

  if(argc == 4 || argc>5){
    cout << "num of added parameters must be eather 1 or 3. See README" << endl;
  }

  char *p;

  if(argc == 5){
    KERNEL_RADIUS = strtol(argv[3], &p, 10);
    MAX_DISPARITY = strtol(argv[4], &p, 10);
  }

  cout << "using kernel radius: " << to_string(KERNEL_RADIUS) << endl;
  cout << "using max disperity: " << to_string(MAX_DISPARITY) << endl;

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
  resize(RS_LIMG, LIMG, cv::Size(), IMG_SCALE, IMG_SCALE);
  resize(RS_RIMG, RIMG, cv::Size(), IMG_SCALE, IMG_SCALE);

  // convert to Grayscale
  cvtColor(LIMG, GS_LIMG, CV_BGR2GRAY);
  cvtColor(RIMG, GS_RIMG, CV_BGR2GRAY);

  int cols = GS_LIMG.cols;
  int rows = GS_LIMG.rows;

  int yOffset = rows/2;
  int xOffset = cols/2;

  // define a vector to save disperities (pixels with lowest SSD)
  vector<vector<int> > disparities (rows-(2*KERNEL_RADIUS), vector<int>(cols-(2*KERNEL_RADIUS))); // Defaults to zero initial value

  int minSSD;
  int currentSSD;
  int minSSDdisparity;
  int minSSDcol;

  Mat lWin(KERNEL_RADIUS*2, KERNEL_RADIUS*2, CV_8UC1, Scalar(0));
  Mat rWin(KERNEL_RADIUS*2, KERNEL_RADIUS*2, CV_8UC1, Scalar(0));
  Mat sumWin(KERNEL_RADIUS*2, KERNEL_RADIUS*2, CV_8UC1, Scalar(0));

  double x = 0;
  double y = 0;
  double z = 0;

  //logging
  std::ofstream myfile;
  try{
    myfile.open (file, std::ios::trunc);
  }
  catch (const std::exception& e) {
    cout << "error opening the logging file" << endl;
  } 

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
      
      // if disparitiy is not zero, calculate x,y,z coordinates of the current pixel
      // offsets are included to move from OpenCV format (coordinate system in upper left corner) to poincloud format (CS in center) 
      if(disp!=0){
        x = -(BL*(double)(pos_disp-2*xOffset))/(2*(double)disp);
        y = BL*(double)(-i+yOffset)/(double)disp;
        z = BL*FL/((double)(disp*PX_PER_MM));
      }
      else{
        x = 0;
        y = 0;
        z = 0;
      }

      // logging
      std::string text = std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " 0 0 0";
      //write into file
      myfile << text << "\n";
    }
  }

  // close the file
  myfile.close();

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