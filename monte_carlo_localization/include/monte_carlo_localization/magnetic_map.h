#ifndef _MAGNETIC_MAP_H_
#define _MAGNETIC_MAP_H_

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

	int height;
	int width;
	string resolution;
    vector<double> origin;
	vector<vector<double>> mag_x_data;
    vector<vector<double>> mag_y_data;
    vector<vector<double>> mag_z_data;
};
#endif