#ifndef _LASERLIKELIHOODMAPBUILDER_H
#define _LASERLIKELIHOODMAPBUILDER_H

#include <iostream> // cout, cerr
#include <vector>
#include "laser_map.h"
#include "laser_likelihood_map.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/math/distributions/normal.hpp>  // for pdf distribution

using namespace std;
using namespace cv;
using boost::math::normal;

class LaserLikelihoodMapBuilder
{
public:
    LaserLikelihoodMapBuilder()
    {
        srand (time(NULL));
        s=normal(0,50);
        normal_max_value = pdf(s,0);
        range = 1;
    }
	~LaserLikelihoodMapBuilder(){}

    void print_map()
    {
        // cout << "Print map " << endl;

        for(int i=0;i<likelihood_map_.height_;i++)
        {
            for(int j=0;j<likelihood_map_.width_;j++)
            {
                cout << likelihood_map_.data_[i][j] << " ";
            }
            cout << endl;
        }
    }

    void build_likelihood_map(LaserMap map)
    {
        std::cout << "Building likelihood map " << std::endl;
        
        cv_likelihood_map_ = imread("/home/lui/likelihood_img.jpg", 0 );
        if(!cv_likelihood_map_.data)                                           // Haven't build likelihood map before
        {
            cout << "Haven't build likelihood map before, building likelihood map" << endl;
            init_map(map);
            markup_probability();
        }else{                                                                  // Loading old likelihood map
            cout << "Loading likelihood map..." << endl;
            init_map(map, cv_likelihood_map_);
            load_likelihood_map();
        }
    }

    LaserLikelihoodMap get_likelihood_map(){return likelihood_map_;}
    void save_cv_likelihood_map(){        
        flip(cv_likelihood_map_,cv_likelihood_map_, 0);
        imwrite("/home/lui/test_likelihood.jpg", cv_likelihood_map_);
    }

private:
    void init_map(LaserMap map)
    {
        laser_map_ = map;
        likelihood_map_ = map;
        mapH = likelihood_map_.height_;
        mapW = likelihood_map_.width_;
        cv_likelihood_map_=cv::Mat(cv::Size(likelihood_map_.width_,likelihood_map_.height_),CV_8UC1,Scalar(0));
        likelihood_map_.data_.resize(likelihood_map_.height_, vector<double>(likelihood_map_.width_, 0.001));
    }
    void init_map(LaserMap map, Mat cv_map)
    {
        laser_map_ = map;
        likelihood_map_ = map;
        mapH = cv_likelihood_map_.rows; 
        mapW = cv_likelihood_map_.cols;
        likelihood_map_.data_.resize(likelihood_map_.height_, vector<double>(likelihood_map_.width_, 0.001));
    }

    double Gaussion(double distance)
    {
        return pdf(s,distance)/normal_max_value;
    }

    void markup_probability()
    {
        std::cout << " Markup the probablity of obstacle " << std::endl;
        // Markup the probablity of obstacle
        for(int i=0;i<mapH;i++)
        {
            for(int j=0;j<mapW;j++)
            {
                if(laser_map_.data_[i][j] == 0)
                {
                    for(int m_i=-1*range; m_i<=range; m_i++)
                    {
                        for(int m_j=-1*range; m_j<=range; m_j++)
                        {
                            double dist = hypot(m_i,m_j);
                            int markup_i = i+m_i;
                            int markup_j = j+m_j;
                            if(markup_i<0 || markup_j<0 || markup_i>= mapH || markup_j>=mapW)
                            {
                                // cout << "Markup index (" << i << ", " << j  << ") wrong in likelihood map" << endl;
                                continue;
                            }
                            double markNum = Gaussion(dist);

                            if(likelihood_map_.data_[markup_i][markup_j] < markNum)
                            {
                                // cout << "Markup index (" << markup_i << ", " << markup_j << ") with " << markNum << endl;
                                likelihood_map_.data_[markup_i][markup_j] = markNum;
                                // cout << "double check: " << likelihood_map_.data_[markup_i][markup_i] << endl;
                                markNum = markNum*255;
                                cv_likelihood_map_.data[markup_i*mapW+markup_j] = markNum;
                            }else{
                                // std::cout << "err " ; 
                            }
                        }
                    }
                }else if(laser_map_.data_[i][j] == 205){
                    if(likelihood_map_.data_[i][j] <= 0.001)
                    {
                        likelihood_map_.data_[i][j] = 0.0;
                    }
                }
            }
        }
    }

    void load_likelihood_map()
    {
        for(int i=0;i<mapH;i++)
        {
            for(int j=0;j<mapW;j++)
            {
                likelihood_map_.data_[i][j] = cv_likelihood_map_.data[i*mapW+j]/255.0;
            }
        }
    }

    int range;
    int mapH;
    int mapW;

    // for Gaussian Distribution
    normal s;
    double normal_max_value;

    LaserMap laser_map_;
    Mat cv_likelihood_map_;                // for showing map
	LaserLikelihoodMap likelihood_map_;     // for Monte Carlo Localization Algorithm
};
#endif