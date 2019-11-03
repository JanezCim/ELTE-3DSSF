#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "ros/ros.h"

using namespace std;
using namespace cv;

int main(int argc, char ** argv){
  if (argc != 3){
    cout << " Usage: display_image ImageToLoadAndDisplay" << endl;
    return -1;
  }

  Mat LIMG;
  Mat RIMG;
  Mat GSimage;
  LIMG = imread(argv[1]);
  RIMG = imread(argv[2]);

  if (!LIMG.data){
    cout << "Could not open or find the left image" << endl;
    return -1;
  }
  else{
    cout << "Imported the left image :)" << endl;
  }
  
  if (!RIMG.data){
    cout << "Could not open or find the right image" << endl;
    return -1;
  }
  else{
    cout << "Imported the right image :)" << endl;
  }



  Mat origCombined;
  Mat scaledLIMG;
  Mat scaledRIMG;
  
  resize(LIMG, scaledLIMG, cv::Size(), 0.25, 0.25);
  resize(RIMG, scaledRIMG, cv::Size(), 0.25, 0.25);
  
  hconcat(scaledLIMG,scaledRIMG,origCombined);

  int key;
  while (1){
    key = waitKey(30);

    if (key == 27){
      // if "esc" is pressed end the program
      std::cout << "Closing the program because esc pressed \n";
      break;
    }
    imshow("original", origCombined);

    
  }

  return 0;
}

/*
import numpy
 
fx = 942.8        # lense focal length
baseline = 54.8   # distance in mm between the two cameras
disparities = 64  # num of disparities to consider
block = 15        # block size to match
units = 0.001     # depth units
 
for i in xrange(block, left.shape[0] - block - 1):
    for j in xrange(block + disparities, left.shape[1] - block - 1):
        ssd = numpy.empty([disparities, 1])
 
        # calc SSD at all possible disparities
        l = left[(i - block):(i + block), (j - block):(j + block)]
        for d in xrange(0, disparities):
            r = right[(i - block):(i + block), (j - d - block):(j - d + block)]
            ssd[d] = numpy.sum((l[:,:]-r[:,:])**2)
 
        # select the best match
        disparity[i, j] = numpy.argmin(ssd)
 
# Convert disparity to depth
depth = np.zeros(shape=left.shape).astype(float)
depth[disparity &gt; 0] = (fx * baseline) / (units * disparity[disparity &gt; 0])

*/