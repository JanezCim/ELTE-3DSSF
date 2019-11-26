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

int KERNEL_SIZE = 3; //this should be an odd number
float IMG_SCALE = 1;
bool SAVE_IMG = 0;

// vector<double > variancesInt {1.0, 2.0, 3.0, 5.0, 10.0, 20.0, 30.0};
// vector<double > variancesDist {1.0, 2.0, 3.0, 5.0, 10.0, 20.0, 30.0};

vector<double > variancesInt {0.1}; 
vector<double > variancesDist {5.1};

// ************************************



// function that calculates distance btw point a and b
float dis (Point a, Point b){
  int dx = a.x-b.x;
  int dy = a.y-b.y;
  return sqrt((float)(dx*dx+dy*dy));
}

// here im basically implementing formulas with this page and sticking to their nomenclature
// source [1]: http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/MANDUCHI1/Bilateral_Filtering.html
// spartial gauss (based on distance of pixles)
double gaussC (Point a, Point b, double varD){
  double tempp = (dis(a,b)/varD);
  double ret = exp(-0.5*(tempp*tempp)); 
  ret = ret * (1/(varD*sqrt(2*CV_PI)));
  return ret;
}

//spectral gauss (based on intensity difference btw pixles) see source [1
double gaussS (int a, int b, double varR){
  double tempp = (fabs(a-b)/varR);
  return (1/(varR*sqrt(2*CV_PI))) * exp(-0.5*(tempp*tempp));
}


/**
 * @brief Applies Bilateral filtering to the input image. Theory explained here: http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/MANDUCHI1/Bilateral_Filtering.html
 * 
 * @param imgIN Input image
 * @param imgOUT Output image
 * @param kerSize Size of the kernel to be used
 * @param distanceVar variance to be used on distance (spartial) gauss
 * @param intensityVar variance to be used on intencitiy difference (spectral) gauss
 */
void BilateralFilter(Mat& imgIN,
                     Mat& imgOUT,
                     int kerSize,
                     double distanceVar,
                     double intensityVar){
  int cols = imgIN.cols;
  int rows = imgIN.rows;

  Mat win;
  Mat OUT(imgIN.rows-kerSize, imgIN.cols-kerSize, CV_8UC1, Scalar(0));
  double k = 0;
  double tempp = 0;
  double sum = 0;

  // iterate through the image 
  for(int x = kerSize/2; x<(cols-kerSize/2-1); x++){
    for(int y = kerSize/2; y<(rows-kerSize/2-1); y++){

      // extracting the kernel arround the current pixle
      win = imgIN(Range(y-kerSize/2, y+kerSize/2+1), Range(x-kerSize/2, x+kerSize/2+1));
      k = 0;
      sum = 0;

      // calculation of the filter summed up outcome for one kernel
      for(int u = 0; u<win.cols; u++){
        for (int v = 0; v<win.rows; v++){
          // for explanation of these formulas see link [1]
          tempp = gaussC(Point(u, v),Point(kerSize/2,kerSize/2), distanceVar) *
                  gaussS(win.at<float>(v,u), win.at<float>(kerSize/2,kerSize/2), intensityVar);
          k += tempp;
          sum += tempp*win.at<float>(v,u);
        }
      }

      // normalize and save the filtered pixle
      OUT.at<float>(y-kerSize/2,x-kerSize/2) = (sum/k);
    }
  }

  imgOUT = OUT;
}

//img1 is gonna be assesed based on clr similarity

/**
 * @brief Applies joint bilateral filtering on the imageIN1 and imageIN2. Its a Bilateral filtering but on two images instead of one.
 * Both input images have to be the same resolution.
 * 
 * @param imgIN1 Input image one. This is regarded as guidance image
 * @param imgIN2 Input image two. This is regarded as the image to be guided by the guidance image (imgIN1)
 * @param imgOUT Resulting image after joint bilateral filter is applied
 * @param kerSize Size of the kernel to be used (has to be an odd number)
 * @param distanceVar variance to be used on distance (spartial) gauss
 * @param intensityVar variance to be used on intencitiy difference (spectral) gauss
 */
void JointBilateralFilter(Mat& imgIN1,
                          Mat& imgIN2,
                          Mat& imgOUT,
                          int kerSize,
                          double distanceVar,
                          double intensityVar){
  int cols = imgIN1.cols;
  int rows = imgIN1.rows;

  // checking if both input images are the same resolution
  if(cols != imgIN2.cols || rows != imgIN2.rows){
    cout << "Two input images of joint bilateral filter are not the same resolution!" << endl;
    return;
  }

  Mat win1;
  Mat win2;
  Mat OUT(imgIN1.rows-kerSize, imgIN1.cols-kerSize, CV_32FC1, Scalar(0));
  double k = 0;
  double tempp = 0;
  double sum = 0;

  // iterate through the image 
  for(int x = kerSize/2; x<(cols-kerSize/2-1); x++){
    for(int y = kerSize/2; y<(rows-kerSize/2-1); y++){

      // extracting the kernel arround the current pixle
      win1 = imgIN1(Range(y-kerSize/2, y+kerSize/2+1), Range(x-kerSize/2, x+kerSize/2+1));
      win2 = imgIN2(Range(y-kerSize/2, y+kerSize/2+1), Range(x-kerSize/2, x+kerSize/2+1));
      k = 0;
      sum = 0;

      // calculation of the filter summed up outcome for one kernel
      for(int u = 0; u<win1.cols; u++){
        for (int v = 0; v<win1.rows; v++){
          // for explanation of these formulas see link [1]
          tempp = gaussC(Point(u, v),Point(kerSize/2,kerSize/2), distanceVar) *
                  gaussS(win1.at<float>(v,u), win1.at<float>(kerSize/2,kerSize/2), intensityVar); //TODO check if this actualy works
          k += tempp;
          sum += tempp*win2.at<float>(v,u);
        }
      }

      // normalize and save the filtered pixle
      OUT.at<float>(y-kerSize/2,x-kerSize/2) = (sum/k);
    }
  }

  imgOUT = OUT;
}

/**
 * @brief Uses Joint Bilateral Filtering to upsample (output in better resolution) the depth image according to the aligned guidanceIMG
 * 
 * @param guidanceIMG Image that the output will be guided by
 * @param depthIMG Image that will be upsampled
 * @param upsampledIMG Resulting output image
 * @param distVar variance to be used on distance (spartial) gauss
 * @param intVar variance to be used on intencitiy difference (spectral) gauss
 */
void Upsample(Mat& guidanceIMG,
              Mat& depthIMG,
              Mat& upsampledIMG,
              double distVar,
              double intVar){
    
    Mat tempDep = depthIMG;
    Mat tempGuid = guidanceIMG;
    Mat outIMG;

    double temp = guidanceIMG.size().area() / (double)depthIMG.size().area()-1;
    int uf = log2(temp); //TODO is it int? double?
    for(int i = 0; i<uf-1; i++){
      resize(tempDep, tempDep, Size(), 2.0, 2.0);
      resize(guidanceIMG, tempGuid, tempDep.size());
      JointBilateralFilter(tempGuid, tempDep, tempDep, KERNEL_SIZE, distVar, intVar);
    }
    resize(tempDep, tempDep, guidanceIMG.size());
    JointBilateralFilter(guidanceIMG, tempDep, tempDep, KERNEL_SIZE, distVar, intVar);
    upsampledIMG = tempDep;
  }

//**********************************************************MAIN****************************************************

int main(int argc, char ** argv){
  if(KERNEL_SIZE%2 == 0){
    cout << "KERNEL_SIZE is not an odd number! Exiting" << endl;
    return -1;
  }

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

  // resizing the images down for debugging
  resize(RS_IMG, RS_IMG, Size(), IMG_SCALE, IMG_SCALE);
  resize(DEPTH_IMG, DEPTH_IMG, Size(), IMG_SCALE, IMG_SCALE);

  // conver to grayscale
  cvtColor(RS_IMG, GS_IMG, CV_BGR2GRAY);

  Mat float_GS_IMG = Mat::zeros(GS_IMG.size(), CV_32FC1);
  Mat float_DEPTH_IMG = Mat::zeros(DEPTH_IMG.size(), CV_32FC1);

  GS_IMG.convertTo(float_GS_IMG, CV_32FC1, 1.0/255.0);
  DEPTH_IMG.convertTo(float_DEPTH_IMG, CV_32FC1, 1.0/255.0);
  /*
  JointBilateralFilter(GS_IMG, DEPTH_IMG, filteredIMG, KERNEL_SIZE, 1.0, 10.0);
  if(filteredIMG.empty()){
    cout << "Joint bilateral filter returned an empty image. Exiting" << endl;
    return -1;
  }
  */

  // resize the depth image again to make it lower resolution
  // resize(DEPTH_IMG, DEPTH_IMG, cv::Size(), 0.25, 0.25, INTER_NEAREST);
  Mat resultingIMG;

  for(double d:variancesDist){
    for(double i:variancesInt){
      // BilateralFilter(GS_IMG, resultingIMG, KERNEL_SIZE, d, i);
      Upsample(float_GS_IMG, float_DEPTH_IMG, resultingIMG, d, i);
      if(SAVE_IMG){
        imwrite( "./././filtered_d"+to_string((int)d)+"_i"+to_string((int)i*10)+".jpg", resultingIMG);
      }
    }
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
    imshow("filtered", resultingIMG); 
  }
  return 0;
}