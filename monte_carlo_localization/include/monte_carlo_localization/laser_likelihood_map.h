#ifndef _LASERLIKELIHOODMAP_H
#define _LASERLIKELIHOODMAP_H

#include <iostream> // cout, cerr
#include <vector>
#include "laser_map.h"
using namespace std;

// static int likelihood_map_nums = 0;

class LaserLikelihoodMap
{
public:
	LaserLikelihoodMap(){}
	~LaserLikelihoodMap(){}
    LaserLikelihoodMap(LaserMap map)
    {
        height_ = map.height_;
        width_ = map.width_;
        resolution_ = map.resolution_;
        origin_ = map.origin_;
        // data_ = map.data_;
    }

	int height_;
	int width_;
    
	double resolution_;
    vector<double> origin_;
	vector<vector<double>> data_;      // for storage the pixel value
};
#endif