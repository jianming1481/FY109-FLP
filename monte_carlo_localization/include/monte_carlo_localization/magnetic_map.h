#ifndef _MAGNETICMAP_H
#define _MAGNETICMAP_H

#include <iostream> // cout, cerr
#include <vector>

using namespace std;

class MagneticMap
{
public:
	MagneticMap()
	{
	origin_ = vector<double>(3, 0.0);
	}
	~MagneticMap(){}

	int height_;
	int width_;
	string resolution_;
    vector<double> origin_;
	vector<vector<double>> mag_x_data_;
    vector<vector<double>> mag_y_data_;
    vector<vector<double>> mag_z_data_;
};
#endif