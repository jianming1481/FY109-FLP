#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>

class CSVReader
{
    std::string fileName;
    std::string delimeter;
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
    std::vector<std::vector<double> > getData();
};

std::vector<std::vector<double>> CSVReader::getData()
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
            tmp_row.data.clear();
            tmp_row.push_back(std::stod(dataList[i][j]));
        }
        m_data.push_back(tmp_row);
    }
    return m_data;
}