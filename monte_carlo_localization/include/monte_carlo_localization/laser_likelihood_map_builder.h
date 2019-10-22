#ifndef _LASER_LIKELIHOOD_MAP_BUILDER_H_
#define _LASER_LIKELIHOOD_MAP_BUILDER_H_

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
        range = 20; // How wide the likelihood filed should expand

        s=normal(0,range);
        normal_max_value = pdf(s,0);

    }
	~LaserLikelihoodMapBuilder(){}

    void print_map()
    {
        // cout << "Print map " << endl;

        for(int i=0;i<likelihood_map.height;i++)
        {
            for(int j=0;j<likelihood_map.width;j++)
            {
                cout << likelihood_map.data[i][j] << " ";
            }
            cout << endl;
        }
    }

    void build_likelihood_map(LaserMap map)
    {
        std::cout << "Building likelihood map " << std::endl;
        string extension = ".jpg";
        string path = map.map_path+extension;
        cout << "Loading " << path << "..." << endl;
        cv_likelihood_map = imread(path, 0 );
        if(!cv_likelihood_map.data)                                           // Haven't build likelihood map before
        {
            cout << "Haven't build likelihood map before, building likelihood map" << endl;
            init_map(map);
            markup_probability();
        }else{                                                                  // Loading old likelihood map
            cout << "Loading likelihood map..." << endl;
            init_map(map, cv_likelihood_map);
            load_likelihood_map();
        }
    }

    LaserLikelihoodMap get_likelihood_map(){return likelihood_map;}
    void save_cv_likelihood_map(){        
        // flip(cv_likelihood_map,cv_likelihood_map, 0);
        string extension = ".jpg";
        string path = laser_map.map_path+extension;
        imwrite(path, cv_likelihood_map);
    }

private:
    void init_map(LaserMap map)
    {
        laser_map = map;
        likelihood_map = map;
        mapH = likelihood_map.height;
        mapW = likelihood_map.width;
        cv_likelihood_map=cv::Mat(cv::Size(likelihood_map.width,likelihood_map.height),CV_8UC1,Scalar(0));
        likelihood_map.data.resize(likelihood_map.height, vector<double>(likelihood_map.width, 0.001));
    }
    void init_map(LaserMap map, Mat cv_map)
    {
        laser_map = map;
        likelihood_map = map;
        mapH = cv_likelihood_map.rows; 
        mapW = cv_likelihood_map.cols;
        likelihood_map.data.resize(likelihood_map.height, vector<double>(likelihood_map.width, 0.001));
    }

    double Gaussion(double distance)
    {
        return pdf(s,distance)/normal_max_value;
    }

    void markup_probability()
    {
        std::cout << "Markup the probablity of obstacle " << std::endl;
        // Markup the probablity of obstacle
        for(int i=0;i<mapH;i++)
        {
            for(int j=0;j<mapW;j++)
            {
                if(laser_map.data[i][j] == 0)
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

                            if(likelihood_map.data[markup_i][markup_j] < markNum)
                            {
                                // cout << "Markup index (" << markup_i << ", " << markup_j << ") with " << markNum << endl;
                                likelihood_map.data[markup_i][markup_j] = markNum;
                                // cout << "double check: " << likelihood_map_.data_[markup_i][markup_i] << endl;
                                markNum = markNum*255;
                                cv_likelihood_map.data[markup_i*mapW+markup_j] = markNum;
                            }else{
                                // std::cout << "err " ; 
                            }
                        }
                    }
                }else if(laser_map.data[i][j] == 205){
                    // likelihood_map.data[i][j] = 0.001;
                    if(likelihood_map.data[i][j] <= 0.001)
                    {
                        likelihood_map.data[i][j] = 0.0;
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
                likelihood_map.data[i][j] = cv_likelihood_map.data[i*mapW+j]/255.0;
            }
        }
        // cout << "Likelihoodmap[10][10]: " << likelihood_map.data[10][10] << endl;
    }

    int range;
    int mapH;
    int mapW;

    // for Gaussian Distribution
    normal s;
    double normal_max_value;

    LaserMap laser_map;
    Mat cv_likelihood_map;                // for showing map
	LaserLikelihoodMap likelihood_map;     // for Monte Carlo Localization Algorithm
};
#endif