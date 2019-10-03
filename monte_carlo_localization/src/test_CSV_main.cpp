#include "monte_carlo_localization/csv_reader.h"
#include "iostream"

using namespace std;

int main(int argc, char** argv)
{
    CSVReader reader("/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/predic/mag_pred_x.csv");
    vector<vector<double>> data;
    data = reader.get_data();
    int height = data.size();
    int width = data[0].size();
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