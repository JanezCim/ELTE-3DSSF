#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "ros/ros.h"
#include "math.h"
#include "homework3/nanoflann.hpp"
#include <eigen3/Eigen/Dense>


using namespace std;
using namespace cv;
using namespace nanoflann;

// ***************************CHANGABLES
const int SAMPLES_DIM = 15;

// ************************************

void searchNN(const Eigen::MatrixXf & cloud1, const Eigen::MatrixXf & cloud2, const size_t k, Eigen::MatrixXi &indices, Eigen::MatrixXf &dists){
  // Eigen::MatrixXf uses colMajor as default
  // copy the coords to a RowMajor matrix and search in this matrix
  // the nearest neighbors for each datapoint
  // Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> coords = cloud.leftCols(3);

  // different max_leaf values only affect the search speed 
  // and any value between 10 - 50 is reasonable
  const int max_leaf = 10;
  typedef Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> RowMatX3f;
  RowMatX3f coords1 = cloud1.leftCols(3);
  RowMatX3f coords2 = cloud2.leftCols(3);
  nanoflann::KDTreeEigenMatrixAdaptor<RowMatX3f> mat_index(3, coords1, max_leaf);
  mat_index.index->buildIndex();
  indices.resize(cloud1.rows(), k);
  dists.resize(cloud1.rows(), k);
  // do a knn search
  for (int i = 0; i < coords1.rows(); ++i) {
    // coords is RowMajor so coords.data()[i*3+0 / +1  / +2] represents the ith row of coords
    std::vector<float> query_pt{ coords1.data()[i*3+0], coords1.data()[i*3+1], coords1.data()[i*3+2] };

    std::vector<size_t> ret_indices(k);
    std::vector<float> out_dists_sqr(k);
    nanoflann::KNNResultSet<float> resultSet(k);
    resultSet.init(&ret_indices[0], &out_dists_sqr[0]);
    mat_index.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));
    for (size_t j = 0; j < k; ++j) {
        indices(i, j) = ret_indices[j];
        dists(i, j) = std::sqrt(out_dists_sqr[j]);
    }
  }
}

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

  Eigen::MatrixXf m1(points1.size(), 3);
  for(int i = 0; i<points1.size(); i++){
    m1(i,0) = points1[i].x;
    m1(i,1) = points1[i].y;
    m1(i,2) = points1[i].z;
  }
  
  if(!import3dPointsFromFile(argv[2], points2)){
    return 0;
  }

  Eigen::MatrixXf m2(points2.size(), 3);
  for(int i = 0; i<points2.size(); i++){
    m2(i,0) = points2[i].x;
    m2(i,1) = points2[i].y;
    m2(i,2) = points2[i].z;
  }

  Eigen::MatrixXi indices;
  Eigen::MatrixXf dists;
  searchNN(m1, m2, 5, indices, dists);

  for(int i = 0; i<5; i++){
    cout << m1(indices(0,i),0) << " " << m1(indices(0,i),1) << " " << m1(indices(0,i),2) << endl;
  }

  return 0;
}