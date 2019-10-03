#include "monte_carlo_localization/csv_reader.h"
#include "iostream"
#include "monte_carlo_localization/magnetic_map.h"
using namespace std;

int main(int argc, char** argv)
{
    CSVReader reader("/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/predic/mag_pred_x.csv");
    vector<vector<double>> data;
    magnetic_map map;
    map = reader.get_data();
    int height = map.height;
    int width = map.width;
    cout << "height: " << height << " width: " << width << endl;
    for(int i=0; i<height;i++)
    {
        for(int j=0; j<width;j++)
        {
            // i*data.size()+j << << ": "
            // cout  << data[i][j] << " ";
        }
        // cout << endl;
    }
    return 0;
}