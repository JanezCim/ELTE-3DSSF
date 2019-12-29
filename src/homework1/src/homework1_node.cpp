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
int PX_PER_MM = 10;
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

// file to output the xyz file to from naive disparities
std::string out_file = "NAIVE_DISPARITIRES_3d_win" + to_string(NAIVE_KERNEL_RADIUS*2) + "_max_disp" + to_string(MAX_DISPARITY) + "_img_scale" + to_string(IMG_SCALE) + ".xyz";

// file to output the xyz file to from dynamic disparities
std::string dynamic_file = "DYNAMIC_DISPARITIRES_3d_win" + to_string(NAIVE_KERNEL_RADIUS*2) + "_max_disp" + to_string(MAX_DISPARITY) + "_img_scale" + to_string(IMG_SCALE) + ".xyz";


void naiveDisparity(const Mat &l_img, const Mat &r_img, const int kernel_radius, const int max_disp, Mat &disp_img);

void dynamicDisparity(const Mat &l_img, const Mat &r_img, const int kernel_radius, const float lambda, Mat &disp_img);

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

  // CHOOSE WHEATHER YOU WANT THE NAIVE OR THE DYNAMIC DISPARITY CALCULATION
  // naiveDisparity(GS_LIMG, GS_RIMG, NAIVE_KERNEL_RADIUS, MAX_DISPARITY, disparitities);
  dynamicDisparity(GS_LIMG, GS_RIMG, DYNAMIC_KERNEL_RADIUS, LAMBDA, disparitities);

  Mat norm_disparitities;
  cv::normalize(disparitities, norm_disparitities, 255, 0, cv::NORM_MINMAX);

  // disparitiesTo3D(norm_disparitities, out_file);

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


// void disparitiesTo3D(const Mat &disp, string file){
//   //logging
//   std::ofstream myfile;
//   try{
//     myfile.open (file, std::ios::trunc);
//   }
//   catch (const std::exception& e) {
//     cout << "error opening the logging file" << endl;
//     return;
//   } 


//   int xOffset = disp.cols/2;
//   int yOffset = disp.rows/2;
//   float x,y,z;

//   // float and adjusted disparity
//   cv::Mat disp_float;
//   disp.convertTo(disp_float, CV_32FC1);
//   cv::Mat disp_adjusted = disp_float + dmin_;

//   for(int u = 0; u<disp.cols; u++){
//     for(int v = 0; v<disp.rows; v++){
//       // if disparitiy is not zero, calculate x,y,z coordinates of the current pixel
//       // offsets are included to move from OpenCV format (coordinate system in upper left corner) to poincloud format (CS in center) 
//       float d = disp_adjusted.at<float>(v,u);
//       if(d!=0){
//         x = -BL * (u+xOffset * 2 - d)/(2 * d);  //-(BL*(double)(pos_disp-2*xOffset))/(2*(double)d);
//         y = (BL*(v-yOffset))/d;  //BL*(double)(-v+yOffset)/(double)d;
//         z = (BL*FL)/d  //BL*FL/((double)(d*PX_PER_MM));
//       }
//       else{
//         x = 0;
//         y = 0;
//         z = 0;
//       }

//       // logging
//       std::string text = std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " 0 0 0";
//       //write into file
//       myfile << text << "\n";  
    
//     }
//   }
// }

void dynamicDisparity(const Mat &l_img, const Mat &r_img, const int kernel_radius, const float lambda, Mat &disp_img){
  // init
  Size size = l_img.size();
  int cols = l_img.cols;
  int rows = l_img.rows;
  Size mat_size = cv::Size(size.width - 2 * kernel_radius, size.width - 2 * kernel_radius);
  Mat abs_diff;


  Mat disp = cv::Mat(l_img.size(), CV_8UC1, cv::Scalar(0));

  // calculate C and M for every row
  for (int row = kernel_radius; row < rows - kernel_radius; ++row){
    // init 2.0
    cv::Mat cost = cv::Mat(mat_size, CV_16UC1, cv::Scalar(0));
    cv::Mat match = cv::Mat(mat_size, CV_8UC1, cv::Scalar(0));

    for (size_t cost_row_idx = 1; cost_row_idx < mat_size.height; ++cost_row_idx){
      cost.at<ushort>(cost_row_idx, 0) = cost_row_idx * lambda;
      match.at<uchar>(cost_row_idx, 0) = 1;
    }

    for (size_t cost_col_idx = 1; cost_col_idx < mat_size.width; ++cost_col_idx){
      cost.at<ushort>(0, cost_col_idx) = cost_col_idx * lambda;
      match.at<uchar>(0, cost_col_idx) = 2;
    }

    // calculating cost and match matrix for current row based on row in left and row in right image
    for (size_t left_col_idx = 1; left_col_idx < mat_size.width; ++left_col_idx){
      for (size_t right_col_idx = 1; right_col_idx < mat_size.height; ++right_col_idx){
        // left and right region of interests
        cv::Rect l_roi = cv::Rect(left_col_idx, row - kernel_radius, 2 * kernel_radius + 1, 2 * kernel_radius + 1);
        cv::Rect r_roi = cv::Rect(right_col_idx, row - kernel_radius, 2 * kernel_radius + 1, 2 * kernel_radius + 1);

        // left and right windows
        cv::Mat leftWindow = l_img(l_roi);
        cv::Mat rightWindow = r_img(r_roi);

        // calculating sad
        cv::absdiff(leftWindow, rightWindow, abs_diff);
        int sad = cv::sum(abs_diff)[0];

        // costs [0] - match, [1] - left occl, [2] - right occl
        int costs[3] = { 0, 0, 0 };
        costs[0] = cost.at<ushort>(right_col_idx - 1, left_col_idx - 1) + sad;
        costs[1] = cost.at<ushort>(right_col_idx - 1, left_col_idx) + lambda;
        costs[2] = cost.at<ushort>(right_col_idx, left_col_idx - 1) + lambda;

        // searching for min cost based on cells calculated before
        int min_cost = costs[0], min_idx = 0;
        for (size_t cost_idx = 0; cost_idx < 3; cost_idx++)
        {
          if (costs[cost_idx] < min_cost)
          {
            min_cost = costs[cost_idx];
            min_idx = cost_idx;
          }
        }

        // assigning the cost and match to the minimal cost
        cost.at<ushort>(right_col_idx, left_col_idx) = min_cost;
        match.at<uchar>(right_col_idx, left_col_idx) = min_idx;
      }
    }

    // cout << match << endl;

    // path planning starting from bottom right
    int disp_row_idx = mat_size.height - 1;
    int disp_col_idx = mat_size.width - 1;

    // iterating over columns
    while (disp_col_idx > 0){
      // match -> step into upper left cell, setting disp
      if (match.at<uchar>(disp_row_idx, disp_col_idx) == 0){
        disp.at<uchar>(row, disp_col_idx) = abs(disp_row_idx - disp_col_idx);
        disp_row_idx--;
        disp_col_idx--;
      }
      // left occl -> step into upper cell
      else if (match.at<uchar>(disp_row_idx, disp_col_idx) == 1){
        disp_row_idx--;
      }
      // right occl -> step into left cell, setting disp
      else if (match.at<uchar>(disp_row_idx, disp_col_idx) == 2){
        disp.at<uchar>(row, disp_col_idx) = 0;
        disp_col_idx--;
      }
    }
  }
  // cout << disp << endl;
  disp_img = disp;
}
