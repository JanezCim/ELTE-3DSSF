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
const int K = 5; //how many neighbors to find
const int MAX_ITERATIONS = 200;
const double ERROR_DROP_THRESH = 0.005;
string OUTPUT_FILE = "src/homework3/src/output.xyz";


// for tr_icp
const double DIST_DROP_THRESH = 0.01;
const double ERROR_LOW_THRESH = 0.0004;
// ************************************

double prev_error = 0;

/**
 * @brief Performs the K nearest neighbors search between two 3d pointclouds
 * 
 * @param cloud1      input pointcloud in which we search for neighbors
 * @param cloud2      input pointcloud through which we iterate and find k nearest neighbors from the cloud1
 * @param k     number of neighbors to be found
 * @param indices     output indices of the nearest neighbors 
 * @param dists       output distances to the nearest neighbors
 */
void searchNN(const Eigen::MatrixXf & cloud1, const Eigen::MatrixXf & cloud2, const size_t k, Eigen::MatrixXi &indices, Eigen::MatrixXf &dists, const bool return_dist_squared = 0){
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
  indices.resize(cloud2.rows(), k);
  dists.resize(cloud2.rows(), k);
  // do a knn search
  for (int i = 0; i < coords2.rows(); ++i) {
    // coords is RowMajor so coords.data()[i*3+0 / +1  / +2] represents the ith row of coords
    std::vector<float> query_pt{ coords2.data()[i*3+0], coords2.data()[i*3+1], coords2.data()[i*3+2] };

    std::vector<size_t> ret_indices(k);
    std::vector<float> out_dists_sqr(k);
    nanoflann::KNNResultSet<float> resultSet(k);
    resultSet.init(&ret_indices[0], &out_dists_sqr[0]);
    mat_index.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));
    for (size_t j = 0; j < k; ++j) {
        indices(i, j) = ret_indices[j];
        if(return_dist_squared){
          dists(i, j) = out_dists_sqr[j];
        }
        else{
          dists(i, j) = std::sqrt(out_dists_sqr[j]);
        }
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

int best_fit_transform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, Eigen::Matrix3d &R, Eigen::Vector3d &t ){
    /*
    Notice:
    1/ JacobiSVD return U,S,V, S as a vector, "use U*S*Vt" to get original Matrix;
    2/ matrix type 'MatrixXd' or 'MatrixXf' matters.
    */
    Eigen::Vector3d centroid_A(0,0,0);
    Eigen::Vector3d centroid_B(0,0,0);
    Eigen::MatrixXd AA = A;
    Eigen::MatrixXd BB = B;
    int row = A.rows();

    for(int i=0; i<row; i++){
        centroid_A += A.block<1,3>(i,0).transpose();
        centroid_B += B.block<1,3>(i,0).transpose();
    }
    centroid_A /= row;
    centroid_B /= row;
    for(int i=0; i<row; i++){
        AA.block<1,3>(i,0) = A.block<1,3>(i,0) - centroid_A.transpose();
        BB.block<1,3>(i,0) = B.block<1,3>(i,0) - centroid_B.transpose();
    }

    Eigen::MatrixXd H = AA.transpose()*BB;
    Eigen::MatrixXd U;
    Eigen::VectorXd S;
    Eigen::MatrixXd V;
    Eigen::MatrixXd Vt;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    S = svd.singularValues();
    V = svd.matrixV();
    Vt = V.transpose();

    R = Vt.transpose()*U.transpose();

    if (R.determinant() < 0 ){
        Vt.block<1,3>(2,0) *= -1;
        R = Vt.transpose()*U.transpose();
    }

    t = centroid_B - R*centroid_A;

    return 1;

}

int icp(const Eigen::MatrixXf &src, const Eigen::MatrixXf &dst, Eigen::MatrixXf &src_trans, const int max_itreations, const double error_drop_thresh){
  Eigen::MatrixXi indices;
  Eigen::MatrixXf dists;
  src_trans = src; 

  // create a new matrix that is gonna be src, but ordered as closest to dst points - it has dst number of rows
  Eigen::MatrixXf src_neighbours(dst.rows(),3);
  double mean_error = 0;


  // iterate throug optimisation till you eather reach max iterations or break out
  for(int i=0; i<max_itreations; i++){
    searchNN(src_trans, dst, K, indices, dists);

    // calculate error btw points src, dst
    mean_error = 0;
    for(int d=0; d<dists.size(); d++){
      mean_error+=dists(d);
    }
    mean_error = mean_error/dists.size();
    // check if error is dropping as fast as it should, if not, finish search
    if(abs(prev_error-mean_error) < error_drop_thresh){
      cout << "ICP has sucessfully reached the neceserry error_drop_thresh." << endl;
      return 1;
    }
    
    // reorder src to fit to the nearest neighbour scheme
    for(int j=0; j<src_neighbours.rows(); j++){
      int ind = indices(j,3);
      src_neighbours(j,0) = src_trans(ind,0);
      src_neighbours(j,1) = src_trans(ind,1);
      src_neighbours(j,2) = src_trans(ind,2);
    }

    // find transform matrix
    Eigen::Matrix3d tR;
    Eigen::Vector3d tt;
    best_fit_transform(src_neighbours.cast <double> (), dst.cast <double> (), tR, tt);
    Eigen::Matrix3f R = tR.cast<float>();
    Eigen::Vector3f t = tt.cast<float>();

    // rotation
    src_trans = (R*src_trans.transpose()).transpose();

    // translation
    for(int fs = 0; fs<src_trans.rows();fs++){
      for(int a = 0; a<3; a++){
        src_trans(fs,a) = src_trans(fs,a)+t(a);   
      }
    }

    cout << "********Cycle "+ to_string(i)+ "*****" << endl;
    cout << "mean_error: "+ to_string(mean_error)+", error difference: "+to_string(abs(prev_error-mean_error)) << endl;

    prev_error = mean_error;
  }
}

int matr_min(Eigen::MatrixXf &matr){
  int cur_min;
  float min_val = FLT_MAX;
  for(int i =0; i<matr.rows(); i++){
    if(matr(i,0)<min_val){
      min_val = matr(i,0);
      cur_min = i;
    }
  }
  return cur_min;
}

void sort_matr(const Eigen::MatrixXf &m, Eigen::MatrixXf &out, Eigen::MatrixXi &indexes, int to_save = -1){
  vector<pair<float, int> > vp;

  // if to_save parameter is default (-1), all elements of the original matrix are saved
  // otherwise the number of elements that is passed is preseved 
  if(to_save == -1){
    to_save = m.rows();
  }


  // Inserting element in pair vector 
  // to keep track of previous indexes 
  for (int i = 0; i < m.rows(); i++) { 
    float vaaal = m(i);
    vp.push_back(make_pair(vaaal, i)); 
  }  

  // Sorting pair vector 
  std::stable_sort(vp.begin(), vp.end(),
                 [](const auto& a, const auto& b){return a.first < b.first;});

  Eigen::MatrixXi ind(to_save,1);
  Eigen::MatrixXf tmp_out(to_save,1);
  for (int i=0; i<to_save; i++){
    tmp_out(i,0)= m(vp[i].second);
    ind(i,0) = vp[i].second;
  }

  out = tmp_out;
  indexes = ind;
}

float prev_dist_sum = FLT_MAX;
int tr_icp(const Eigen::MatrixXf &src,
           const Eigen::MatrixXf &dst,
           Eigen::MatrixXf &src_trans, 
           const int max_itreations,
           const double error_low_thresh, 
           const double dist_drop_thresh){
  // here for testing -- TODO insert into parameters
  int Npo = 1000;
  
  Eigen::MatrixXi knn_indices;
  Eigen::MatrixXf sq_dists, tr_sq_dists(Npo, 1);
  Eigen::MatrixXf tr_dst(Npo,3), tr_src(Npo, 3);
  src_trans = src; 

  Eigen::MatrixXf src_neighbours(dst.rows(),3);
  double mean_error = 0;

  // iterate throug optimisation till you eather reach max iterations or break out
  for(int i=0; i<max_itreations; i++){
    searchNN(src_trans, dst, K, knn_indices, sq_dists, 1);


    // sort and trimm distances
    Eigen::MatrixXi old_dst_ind;
    Eigen::MatrixXf sorted_tr_distances;
    sort_matr(sq_dists, sorted_tr_distances, old_dst_ind, Npo);

    // save trimmed source and destination
    for(int i =0; i<tr_dst.rows(); i++){
      int bla = old_dst_ind(i);
      tr_src.block<1,3>(i,0) = src.block<1,3>(knn_indices(bla),0);
      tr_dst.block<1,3>(i,0) = dst.block<1,3>(bla,0);
    }

    // calculate stopping conditions ************************
    // sum up all the smallest distances
    float dist_sum = sorted_tr_distances.sum();

    // trimmed mean square error
    float e = dist_sum/Npo;

    // if mse is lower then thresh or if distance drop if below the thresh, stop the tr_icp
    //TODO for now exiting only with error low thresh, when this works, add dist dtop thresh
    if(e<error_low_thresh){  //|| abs(prev_dist_sum-dist_sum)<dist_drop_thresh){
      return 1;
    }

    // calculate translation ************************************
    // find transform matrix
    Eigen::Matrix3d tR;
    Eigen::Vector3d tt;
    best_fit_transform(tr_src.cast<double>(), tr_dst.cast<double>(), tR, tt);
    Eigen::Matrix3f R = tR.cast<float>();
    Eigen::Vector3f t = tt.cast<float>();

    // rotation
    src_trans = (R*src_trans.transpose()).transpose();

    // translation
    for(int fs = 0; fs<src_trans.rows();fs++){
      for(int a = 0; a<3; a++){
        src_trans(fs,a) = src_trans(fs,a)+t(a);   
      }
    }

    // TODO delete this output - its for debug
    // save the result into output file
    ofstream outputFile11(OUTPUT_FILE);
    for(int g = 0; g<src_trans.rows(); g++){
      for(int gh = 0; gh<3; gh++){
        outputFile11 << src_trans(g,gh) << " ";
      }
      outputFile11 << endl;
    }
    outputFile11.close();

    cout <<"********Cycle "+ to_string(i)+ "*****" << endl;
    cout <<"trimmed MSE: "+ to_string(e)+ "/" + to_string(error_low_thresh) <<endl;
    cout <<"Change of trimmed MSE: "+to_string(abs(prev_dist_sum-dist_sum))+ "/" + to_string(dist_drop_thresh) << endl;
    cout <<"WARNING - now only exiting when MSE error is reached, not checking change of MSE";

    prev_dist_sum = dist_sum;
  }
  
  return 1;
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

  Eigen::MatrixXf src(points1.size(), 3);
  for(int i = 0; i<points1.size(); i++){
    src(i,0) = points1[i].x;
    src(i,1) = points1[i].y;
    src(i,2) = points1[i].z;
  }
  
  ofstream outputFile3("src/homework3/src/src.xyz");
  for(int g = 0; g<src.rows(); g++){
    for(int gh = 0; gh<3; gh++){
      outputFile3 << src(g,gh) << " ";
    }
    outputFile3 << endl;
  }
  outputFile3.close();

  if(!import3dPointsFromFile(argv[2], points2)){
    return 0;
  }

  // Point3d dst_center;
  Eigen::MatrixXf dst(points2.size(), 3);
  for(int i = 0; i<points2.size(); i++){
    dst(i,0) = points2[i].x;
    dst(i,1) = points2[i].y+1;
    dst(i,2) = points2[i].z+1;
    // dst_center += points2[i];
  }
  // dst_center.x = dst_center.x/points2.size();
  // dst_center.y = dst_center.y/points2.size();
  // dst_center.z = dst_center.z/points2.size();



  ofstream outputFile("src/homework3/src/dst.xyz");
  for(int g = 0; g<dst.rows(); g++){
    for(int gh = 0; gh<3; gh++){
      outputFile << dst(g,gh) << " ";
    }
    outputFile << endl;
  }
  outputFile.close();

  // // execute icp
  // if(!icp(src, dst, src, MAX_ITERATIONS, ERROR_DROP_THRESH)){
  //   cout << "Error while execution of ICP" << endl;
  //   return -1;
  // }
  
  // execute trimmed icp
  Eigen::MatrixXf out(dst.rows(), 3);
  if(!tr_icp(src, dst, out, MAX_ITERATIONS, ERROR_LOW_THRESH, DIST_DROP_THRESH)){
    cout << "Error while execution of ICP" << endl;
    return -1;
  }
  
  cout << "Finished and saved result into: " + OUTPUT_FILE << endl;

  // save the result into output file
  ofstream outputFile1(OUTPUT_FILE);
  for(int g = 0; g<src.rows(); g++){
    for(int gh = 0; gh<3; gh++){
      outputFile1 << out(g,gh) << " ";
    }
    outputFile1 << endl;
  }
  outputFile1.close();


  return 0;
}