#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "ros/ros.h"

using namespace std;
using namespace cv;

// if 1, calculates dynamic, if 0 calculates naive disparities
bool CALCULATE_DYNAMIC_DISPARITY = 1;

// focal lenght in px
double FL = 3740.0;
int PX_PER_MM = 20;
// baseline (dist btw cameras) in mm
double BL = 160.0;
// scale the image before computing on it (faster computation for debugging)
double IMG_SCALE = 0.25;


// DEFAULT NAIVE PARAMETERS
// window radius (its a square) in px to convolute with
int NAIVE_KERNEL_RADIUS = 16;
// number of disparities to consider
int MAX_DISPARITY = 40;

// DEFAULT DYNAMIC PARAMTERS
int DYNAMIC_KERNEL_RADIUS = 5;
float LAMBDA = 100;


int DMIN = 230;

// file to output the xyz file to from naive disparities
std::string out_file = "NAIVE_DISPARITIRES_3d_win" + to_string(NAIVE_KERNEL_RADIUS*2) + "_max_disp" + to_string(MAX_DISPARITY) + "_img_scale" + to_string(IMG_SCALE) + ".xyz";

// file to output the xyz file to from dynamic disparities
std::string dynamic_file = "DYNAMIC_DISPARITIRES_3d_win" + to_string(NAIVE_KERNEL_RADIUS*2) + "_max_disp" + to_string(MAX_DISPARITY) + "_img_scale" + to_string(IMG_SCALE) + ".xyz";

/**
 * @brief caluclates naive dispartiy
 * 
 * @param l_img   left input image
 * @param r_img   right input image
 * @param kernel_radius   the radius of the kernel for the convolution
 * @param max_disp  maximum allowed disparity
 * @param disp_img  output non- normalized disparty image
 */
void naiveDisparity(const Mat &l_img, const Mat &r_img, const int kernel_radius, const int max_disp, Mat &disp_img);

/**
 * @brief calculates dynamic disparity
 * 
 * @param l_img   left input image
 * @param r_img   right input image
 * @param kernel_radius   the radius of the kernel for the convolution
 * @param lambda  lambda for the dynamic calculation
 * @param disp_img  output non- normalized disparty image
 */
void dynamicDisparity(const Mat &l_img, const Mat &r_img, const int kernel_radius, const float lambda, Mat &disp_img);

/**
 * @brief takes normalized disparities and converts them to 3D points which are written into the file
 * 
 * @param disp  normalized input disparities
 * @param file  output file for 3d points to be saved to
 */
void disparitiesTo3D(const Mat &disp, string file);


/*************************** MAIN **************************************/

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
    NAIVE_KERNEL_RADIUS = strtol(argv[3], &p, 10);
    MAX_DISPARITY = strtol(argv[4], &p, 10);
  }

  cout << "using kernel radius: " << to_string(NAIVE_KERNEL_RADIUS) << endl;
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

  Mat disparitities;
  Mat norm_disparitities;

  if(!CALCULATE_DYNAMIC_DISPARITY){
    naiveDisparity(GS_LIMG, GS_RIMG, NAIVE_KERNEL_RADIUS, MAX_DISPARITY, disparitities);
    cv::normalize(disparitities, norm_disparitities, 255, 0, cv::NORM_MINMAX);
  }

  if(CALCULATE_DYNAMIC_DISPARITY){
    dynamicDisparity(GS_LIMG, GS_RIMG, DYNAMIC_KERNEL_RADIUS, LAMBDA, disparitities);
    cv::normalize(disparitities, norm_disparitities, 255, 0, cv::NORM_MINMAX);
    disparitiesTo3D(norm_disparitities, dynamic_file);
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
    imshow("disparities", norm_disparitities);
    
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

void naiveDisparity(const Mat &l_img, const Mat &r_img, const int kernel_radius, const int max_disp, Mat &disp_img){
  int cols = l_img.cols;
  int rows = l_img.rows;

  int yOffset = rows/2;
  int xOffset = cols/2;

  // define a vector to save disperities (pixels with lowest SSD)
  vector<vector<int> > disparities (rows-(2*kernel_radius), vector<int>(cols-(2*kernel_radius))); // Defaults to zero initial value

  int minSSD;
  int currentSSD;
  int minSSDdisparity;
  int minSSDcol;

  Mat lWin(kernel_radius*2, kernel_radius*2, CV_8UC1, Scalar(0));
  Mat rWin(kernel_radius*2, kernel_radius*2, CV_8UC1, Scalar(0));
  Mat sumWin(kernel_radius*2, kernel_radius*2, CV_8UC1, Scalar(0));


  double x = 0;
  double y = 0;
  double z = 0;

  //logging
  std::ofstream myfile;
  try{
    myfile.open (out_file, std::ios::trunc);
  }
  catch (const std::exception& e) {
    cout << "error opening the logging file" << endl;
  } 

  for(int i = kernel_radius; i<(rows-kernel_radius); i++){
    for(int j = kernel_radius; j<(cols-kernel_radius); j++){
      minSSD = 1000000;
      minSSDcol = 0;
      for(int k = kernel_radius; k<(cols-kernel_radius); k++){
        // if the pixles are too far appart, do not check for disparity
        if(abs(k-j)>max_disp){
          continue;
        }

        lWin = l_img(Range(i-kernel_radius, i+kernel_radius), Range(j-kernel_radius, j+kernel_radius));
        rWin = r_img(Range(i-kernel_radius, i+kernel_radius), Range(k-kernel_radius, k+kernel_radius));
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
      disparities[i-kernel_radius][j-kernel_radius] = disp;
      
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

  // resize disparities
  for(int x = 0; x<disparities.size(); x++){
    for(int y=0; y<disparities[0].size(); y++){
      disparitiesIMG.at<uchar>(x,y) = disparities[x][y];
    }
  }
  disp_img = disparitiesIMG;
}


void disparitiesTo3D(const Mat &disp, string file){
  //logging
  std::ofstream myfile;
  try{
    myfile.open (file, std::ios::trunc);
  }
  catch (const std::exception& e) {
    cout << "error opening the logging file" << endl;
    return;
  } 


  int xOffset = disp.cols/2;
  int yOffset = disp.rows/2;
  float x,y,z;

  cv::Mat disp_tmp;
  disp.convertTo(disp_tmp, CV_32FC1);
  disp_tmp = disp_tmp + DMIN;

  for(int u = 0; u<disp.cols; u++){
    for(int v = 0; v<disp.rows; v++){
      float d = disp_tmp.at<float>(v,u);
      if(d!=0){
        x = -(BL*(double)(u-xOffset))/(2*d);
        y = BL*(double)(-v+yOffset)/d;
        z = BL*FL/((double)(d*PX_PER_MM));
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
}

void dynamicDisparity(const Mat &l_img, const Mat &r_img, const int kernel_radius, const float lambda, Mat &disp_img){
  int cols = l_img.cols;
  int rows = l_img.rows;
  int matr_size = cols-2*kernel_radius;
  Mat disparities = cv::Mat(l_img.size(), CV_8UC1, cv::Scalar(0));

  int minSSD;
  int currentSSD;
  int minSSDdisparity;
  int minSSDcol;

  Mat lWin(kernel_radius*2, kernel_radius*2, CV_8UC1, Scalar(0));
  Mat rWin(kernel_radius*2, kernel_radius*2, CV_8UC1, Scalar(0));
  Mat sumWin(kernel_radius*2, kernel_radius*2, CV_8UC1, Scalar(0));

  int min_c = 0, min_i = 0;

  int disp_y, disp_x = 0;
  int x,y,z = 0;

  int yOffset = rows/2;
  int xOffset = cols/2;

  //logging
  std::ofstream myfile;
  try{
    myfile.open (out_file, std::ios::trunc);
  }
  catch (const std::exception& e) {
    cout << "error opening the logging file" << endl;
  } 

  // go through image, excluding the kernel width of edges
  for (int row = kernel_radius; row < rows-kernel_radius; ++row){
    cv::Mat c = cv::Mat(Size(matr_size,matr_size), CV_16UC1, cv::Scalar(0));
    cv::Mat m = cv::Mat(Size(matr_size,matr_size), CV_8UC1, cv::Scalar(0));

    // fill the cols costs and matches with innitial values
    for (int i = 1; i < matr_size; ++i){
      c.at<ushort>(0, i) = i * lambda;
      m.at<uchar>(0, i) = 2;
    }

    // fill the rows costs and matches with innitial values
    for (int i = 1; i < matr_size; ++i){
      c.at<ushort>(i, 0) = i * lambda;
      m.at<uchar>(i, 0) = 1;
    }

    // calculate m and c for each pixel for lesft and right image
    for (int left_x = 1; left_x < matr_size; ++left_x){
      for (int right_x = 1; right_x < matr_size; ++right_x){
        // convolution windows
        // lWin = l_img(Range(right_x, row-kernel_radius), Range(2*kernel_radius+1, 2*kernel_radius+1));
        // rWin = r_img(Range(left_x, row-kernel_radius), Range(2*kernel_radius+1, 2*kernel_radius+1));
      
        lWin = l_img(Rect(right_x, row-kernel_radius, 2*kernel_radius+1, 2*kernel_radius+1));
        rWin = r_img(Rect(left_x, row-kernel_radius, 2*kernel_radius+1, 2*kernel_radius+1));

        absdiff(lWin, rWin, sumWin);
        currentSSD = sum(sumWin)[0];

        // determine costs
        int costs[3] = {c.at<ushort>(right_x-1, left_x-1) + currentSSD,
                        c.at<ushort>(right_x-1, left_x) + (int)lambda,
                        c.at<ushort>(right_x, left_x-1) + (int)lambda};
        
        // cout << costs << endl;


        int min_c = costs[0];
        int min_i = 0;
        for (int i = 0; i<3; i++){
          if (costs[i] < min_c){
            min_c = costs[i];
            min_i = i;
          }
        }
        m.at<uchar>(right_x, left_x) = min_i;
        c.at<ushort>(right_x, left_x) = min_c;
      }
    }

    // cout << m << endl;

    // starting from the bottom right
    disp_y = matr_size-1;
    disp_x = matr_size-1;

    int cur_disp = 0;
    // if the x dispartity is not zero, continue checking
    while (disp_x > 0){
      // match
      if (m.at<uchar>(disp_y, disp_x) == 0){
        cur_disp = abs(disp_y - disp_x);
        disp_x--;
        disp_y--;
      }
      else if (m.at<uchar>(disp_y, disp_x) == 2){
        cur_disp = 0;
        disp_x--;
      }
      else if (m.at<uchar>(disp_y, disp_x) == 1){
        disp_y--;
      }
      disparities.at<uchar>(row, disp_x) = cur_disp;
    }
  }
  // cout << disparities << endl;
  disp_img = disparities;
}
