#ifndef _CSV_READER_H_
#define _CSV_READER_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include "magnetic_map.h"

class CSVReader
{
    std::string fileName;
    std::string delimeter;
    MagneticMap map;
public:
    CSVReader(){}
    CSVReader(std::string filename, std::string delm = ",") :
        fileName(filename), delimeter(delm)
    { }
    void set_filename(std::string file_name)
    {
        fileName = file_name;
        delimeter = ",";
    }
    // Function to fetch data from a CSV File
    MagneticMap get_data()
    {
        std::ifstream file(fileName);
        std::vector<std::vector<std::string> > dataList;
        std::string line = "";
        double max_value = 0.0;
        double min_value = 0.0;
        // Iterate through each line and split the content using delimeter
        while (getline(file, line))
        {
            std::vector<std::string> vec;
            boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
            dataList.push_back(vec);
        }
        // Close the File
        file.close();
        std::vector<std::vector<double> > m_data;
        std::vector<double> tmp_row;
        for(int i=0; i<dataList.size();i++)
        {
            for(int j=0; j<dataList[0].size();j++)
            {
                tmp_row.push_back(std::stod(dataList[i][j]));
                if (max_value < std::stod(dataList[i][j]))
                {
                    max_value = std::stod(dataList[i][j]);
                }
                    
                if (min_value > std::stod(dataList[i][j]))
                {
                    min_value = std::stod(dataList[i][j]);
                }
            }
            map.data.push_back(tmp_row);
            tmp_row.clear();
        }
        for(int i=0; i<dataList.size();i++)
        {
            for(int j=0; j<dataList[0].size();j++)
            {
                map.data[i][j] = (map.data[i][j]-min_value)/max_value*255;
            }
        }
        map.height = dataList.size();
        map.width = dataList[0].size();

        return map;
    }
    MagneticMap get_origin_data()
    {
        std::ifstream file(fileName);
        std::vector<std::vector<std::string> > dataList;
        std::string line = "";
        // Iterate through each line and split the content using delimeter
        while (getline(file, line))
        {
            std::vector<std::string> vec;
            boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
            dataList.push_back(vec);
        }
        // Close the File
        file.close();
        std::vector<std::vector<double> > m_data;
        std::vector<double> tmp_row;
        for(int i=0; i<dataList.size();i++)
        {
            for(int j=0; j<dataList[0].size();j++)
            {
                tmp_row.push_back(std::stod(dataList[i][j]));
            }
            map.data.push_back(tmp_row);
            tmp_row.clear();
        }

        map.height = dataList.size();
        map.width = dataList[0].size();

        return map;
    }
};

#endif