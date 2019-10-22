#ifndef _LASER_MAP_READER_H_
#define _LASER_MAP_READER_H_

#include <iostream> // cout, cerr
#include <vector>
#include <fstream> // ifstream
#include <sstream> // stringstream
#include "jsoncpp/json/json.h"
#include "yaml-cpp/yaml.h"
#include "laser_map.h"

using namespace std;

class LaserMapReader
{
public:
  LaserMapReader(){}
  ~LaserMapReader(){}
  
  void print_data()
  {
    cout << "Width: " << width << "\tHeight: " << height << endl;
    cout << "pgm version: " << pgm_version << endl;
    cout << "Max Gray Value: " << max_grayscale_value << endl;

    for(int i = 0; i < height; i++) 
    {
      for(int j = 0; j < width; j++) 
      {
        cout << (int)data[i][j] << " ";
      }
      cout << endl;
    }
  }
  
  vector<vector<double>> get_data()
  {
    vector<vector<double>> output_data;
    vector<double> tmp;

    for(int i=0;i<height;i++)
    {
      for(int j=0;j<width;j++)
      {
        tmp.push_back(data[i][j]);
      }
      output_data.push_back(tmp);
      tmp.clear();
    }
    return output_data;
  }
  
  LaserMap get_map()
  {
    LaserMap map;
    map.height = height;
    map.width = width;
    map.resolution = resolution;
    map.origin = origin;
    map.negate = negate;
    map.occupied_thresh = occupied_thresh;
    map.free_thresh = free_thresh;
    size_t lastindex = image_path.find_last_of(".");
    map.map_path = image_path.substr(0,lastindex);

    map.data = this->get_data();
    return map;
  }

  void read_yaml(std::string yaml_path)
  {
    node = YAML::LoadFile(yaml_path);
    std::cout << "Load map: " << yaml_path << std::endl;

    size_t lastindex = yaml_path.find_last_of("."); 
    string rawname = yaml_path.substr(0, lastindex); 

    image_path = rawname;
    resolution = node["resolution"].as<double>();
    origin = node["origin"].as<std::vector<double>>();
    negate = node["negate"].as<int>();
    occupied_thresh = node["occupied_thresh"].as<double>();
    free_thresh = node["free_thresh"].as<double>();

    read_pgm();
  }
  
private:
  bool read_pgm()
  {
    file_name = image_path;
    string extension = ".pgm";

    // file_name = "/home/lui/"+file_name;
    std::cout << "Load Image from: " << file_name+extension << std::endl;
    ifstream infile(file_name+extension);
    stringstream ss;
    string inputLine = "";
    // First line : version
    getline(infile,inputLine);
    if(inputLine.compare("P5") != 0) 
    {
      cerr << "Version error" << endl;
      return false;
    }
    pgm_version = inputLine;
    getline(infile,inputLine);
    pgm_comment = inputLine;
    // Continue with a stringstream
    ss << infile.rdbuf();
    // Third line : size
    ss >> width >>  height >> max_grayscale_value;
    data = new unsigned char*[height];
    for (int i = 0; i < height; i++)
    {
      data[i] = new unsigned char[width];
    }
    // Following lines : data
    for(int j=height-1; j>=0; j--)
    {
      for (int i= 0; i<width; i++)
      {
        ss >> data[j][i];
      }
    }
    infile.close();
    return true;
  }


  // YAML
  YAML::Node node;

  // Map Data
  std::string image_path;
  double resolution;
  std::vector<double> origin;
  int negate;
  double occupied_thresh;
  double free_thresh;
  int height;
	int width;
	unsigned char **data;      // for storage the pixel value

  // PGM
  string pgm_version;        // P2 or P5
  string pgm_comment;
  string file_name;
  stringstream string_stream;// Buffer, for storage the data after text header in pgm file
  string input_line;         // For storage data temporarily
  ifstream infile;           // Read pgm file
  int max_grayscale_value;

};
#endif