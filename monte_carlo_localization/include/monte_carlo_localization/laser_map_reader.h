#ifndef _LASER_MAP_READER_H
#define _LASER_MAP_READER_H

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
    cout << "Width: " << width_ << "\tHeight: " << height_ << endl;
    cout << "pgm version: " << pgm_version_ << endl;
    cout << "Max Gray Value: " << max_grayscale_value_ << endl;

    for(int i = 0; i < height_; i++) 
    {
      for(int j = 0; j < width_; j++) 
      {
        cout << (int)data_[i][j] << " ";
      }
      cout << endl;
    }
  }
  
  vector<vector<double>> get_data()
  {
    vector<vector<double>> output_data;
    vector<double> tmp;

    for(int i=0;i<height_;i++)
    {
      for(int j=0;j<width_;j++)
      {
        tmp.push_back(data_[i][j]);
      }
      output_data.push_back(tmp);
      tmp.clear();
    }
    return output_data;
  }
  
  LaserMap get_map()
  {
    LaserMap map;
    map.height_ = height_;
    map.width_ = width_;
    map.resolution_ = resolution_;
    map.origin_ = origin_;
    map.negate_ = negate_;
    map.occupied_thresh_ = occupied_thresh_;
    map.free_thresh_ = free_thresh_;

    map.data_ = this->get_data();
    return map;
  }

  void read_yaml(std::string yaml_path)
  {
    node = YAML::LoadFile(yaml_path);
    std::cout << "Load map: " << yaml_path << std::endl;

    image_path = node["image"].as<std::string>();
    resolution_ = node["resolution"].as<double>();
    origin_ = node["origin"].as<std::vector<double>>();
    negate_ = node["negate"].as<int>();
    occupied_thresh_ = node["occupied_thresh"].as<double>();
    free_thresh_ = node["free_thresh"].as<double>();

    read_pgm();
  }
  
private:
  bool read_pgm()
  {
    file_name_ = image_path;
    file_name_ = "/home/lui/"+file_name_;
    std::cout << "Load Image from: " << file_name_ << std::endl;
    ifstream infile(file_name_);
    stringstream ss;
    string inputLine = "";
    // First line : version
    getline(infile,inputLine);
    if(inputLine.compare("P5") != 0) 
    {
      cerr << "Version error" << endl;
      return false;
    }
    pgm_version_ = inputLine;
    getline(infile,inputLine);
    pgm_comment = inputLine;
    // Continue with a stringstream
    ss << infile.rdbuf();
    // Third line : size
    ss >> width_ >>  height_ >> max_grayscale_value_;
    data_ = new unsigned char*[height_];
    for (int i = 0; i < height_; i++)
    {
      data_[i] = new unsigned char[width_];
    }
    // Following lines : data
    for(int j=height_-1; j>=0; j--)
    {
      for (int i= 0; i<width_; i++)
      {
        ss >> data_[j][i];
      }
    }
    infile.close();
    return true;
  }


  // YAML
  YAML::Node node;

  // Map Data
  std::string image_path;
  double resolution_;
  std::vector<double> origin_;
  int negate_;
  double occupied_thresh_;
  double free_thresh_;
  int height_;
	int width_;
	unsigned char **data_;      // for storage the pixel value

  // PGM
  string pgm_version_;        // P2 or P5
  string pgm_comment;
  string file_name_;
  stringstream string_stream_;// Buffer, for storage the data after text header in pgm file
  string input_line_;         // For storage data temporarily
  ifstream infile_;           // Read pgm file
  int max_grayscale_value_;

};
#endif