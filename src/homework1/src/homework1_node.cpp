#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "ros/ros.h"

using namespace std;
using namespace cv;

int main(int argc, char ** argv)
{
  if (argc != 2)
  {
    cout << " Usage: display_image ImageToLoadAndDisplay" << endl;
    return -1;
  }

  Mat RGBimage;
  Mat GSimage;
  RGBimage = imread(argv[1]);  // Read the file

  if (!RGBimage.data)
  {
    cout << "Could not open or find the image" << endl;
    return -1;
  }
  else
  {
    cout << "Imported the image :)" << endl;
  }
}