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

    std::vector<Vector2d> laser_wall;
    std::vector<double> data;
    double ranges;
    int sample_num;
};

#endif