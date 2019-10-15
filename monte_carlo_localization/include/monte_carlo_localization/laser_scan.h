#ifndef _LASER_SCAN_H_
#define _LASER_SCAN_H_

#include <vector>
#include "eigen3/Eigen/Dense"

using namespace std;
using namespace Eigen;

class LaserScan
{
public:
    LaserScan(){}
    ~LaserScan(){}

    std::vector<double> data;
    vector<Vector2d> scan_wall;
    double ranges;
    int sample_num;
};

#endif