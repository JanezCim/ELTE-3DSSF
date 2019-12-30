# Homework 3: Advanced point cloud registration


nice thing to know: https://www.cc.gatech.edu/classes/AY2016/cs4496_spring/Eigen.html

    rosrun homework3 homework3_node src/homework3/src/1.xyz src/homework3/src/1.xyz

if its to slow, make sure you are building for release and for cpp standard 14

    catkin_make -DCMAKE_CXX_STANDARD=14 -DCMAKE_BUILD_TYPE=RELEASE