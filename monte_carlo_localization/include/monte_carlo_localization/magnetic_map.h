#ifndef _MAGNETIC_MAP_H_
#define _MAGNETIC_MAP_H_

#include <iostream> 
#include <vector>
using namespace std; 

class magnetic_map
{
private:
    /* data */
public:
    magnetic_map(/* args */){}
    ~magnetic_map(){}
    int width;
    int height;
    double max_value;
    vector<vector<double>> data;
};
#endif