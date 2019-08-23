/*
 *   particle_filter.cpp
 *
 *   Date:2019
 *
 *   Author: Chien-Ming Lin
 *
 */

#include "iostream"
#include "monte_carlo_localization/particle_filter.h"
#define trust_imu false

Particle tmp_p;
bool reflash;

ParticleFilter::ParticleFilter(int p_Num)
{
    pNum = p_Num;
    srand (time(NULL));
    s=normal(0,50);
    normal_max_value = pdf(s,0);
}

double ParticleFilter::Gaussion(double input)
{
    return pdf(s,input)/normal_max_value;
}

void ParticleFilter::build_likelihoodMap()
{
    int map_index;
    likelihood_img = imread("/home/lui/likelihood_img.jpg", 0 );
    cv::flip(likelihood_img, likelihood_img, 0);
    if(!likelihood_img.data)
    {
        std::cout << " building likelihood map " << std::endl;
        image = imread( "/home/lui/map.jpg", 1 );
        cv::flip(image, image, 0);
        cv::Mat gray_img;
        cv::Mat binary_img;

        mapW = image.cols;
        mapH = image.rows;

        // For likelihood field generate range
        int range = 80;
        // For OpenCV Save Map
        cv::cvtColor(image,gray_img,CV_RGB2GRAY);
        cv::threshold(gray_img,binary_img,10,255,CV_THRESH_BINARY);
        // imwrite("/home/lui/binary_img.jpg",binary_img);
        // std::cout << "Save Binary Image Done!" << std::endl;

        map = new bool[mapW*mapH];
        likelihood_mapW = mapW+range*2;
        likelihood_mapH = mapH+range*2;
        cv_likelihood_map=new cv::Mat(cv::Size(likelihood_mapW,likelihood_mapH),CV_8UC1,Scalar(0));
        likeliHood_map = new double[likelihood_mapW*likelihood_mapH];

        // Initial Likelihood Map
        for(int i = 0 ;i < likelihood_mapW*likelihood_mapH;i++)
        {
            // std::cout << "Initialize likelihood field... Give a small value in each grid" << std::endl; 
            likeliHood_map[i] = 0.001;
        }

        // Markup the probablity of obstacle
        for(int i=0;i<mapH;i++)
        {
            for(int j=0;j<mapW;j++)
            {
                if(binary_img.data[(i*mapW)+j]==0)
                {   
                    map_index = i*mapW+j;
                    map[map_index] = true;

                    for(int m_i=-1*range; m_i<range; m_i++)
                    {
                        for(int m_j=-1*range; m_j<range; m_j++)
                        {
                            double dist = hypot(m_i,m_j);
                            double markNum = Gaussion(dist);
                            int likelihood_field_index = (i+range)*likelihood_mapW+j+range;
                            likelihood_field_index = likelihood_field_index+m_i*likelihood_mapW+m_j;
                            if(likelihood_field_index < 0 || likelihood_field_index > likelihood_mapW*likelihood_mapH)
                            {
                                // std::cout << "Something Wrong Inside building map loop!!!" << std::endl;
                                continue;
                            }

                            if(likeliHood_map[likelihood_field_index]<markNum)
                            {
                                likeliHood_map[likelihood_field_index] = markNum;
                                markNum = markNum*255;
                                cv_likelihood_map->data[likelihood_field_index] = markNum;
                            }else{
                                // std::cout << "err " ; 
                            }
                            
                        }
                    }
                }
            }
        }
        std::cout << "Likelihood map done!" << std::endl;
        cv::flip(*cv_likelihood_map, *cv_likelihood_map, 0);
        imwrite("/home/lui/likelihood_img.jpg",*cv_likelihood_map);
    }else{
        image = imread( "/home/lui/map.jpg", 1 );
        cv::flip(image, image, 0);
        mapW = image.cols;
        mapH = image.rows;
        likelihood_mapW = likelihood_img.cols;
        likelihood_mapH = likelihood_img.rows;

        cv::Mat gray_img;
        cv::Mat binary_img;
        cv::cvtColor(image,gray_img,CV_RGB2GRAY);
        cv::threshold(gray_img,binary_img,10,255,CV_THRESH_BINARY);

        map = new bool[mapW*mapH];
        cv_likelihood_map=new cv::Mat(cv::Size(likelihood_mapW,likelihood_mapH),CV_8UC1,Scalar(0));
        likeliHood_map = new double[likelihood_mapW*likelihood_mapH];
        double markNum=0.0;
        for(int i=0; i<likelihood_mapH; i++)
        {
            for(int j=0; j<likelihood_mapW; j++)
            {
                cv_likelihood_map->data[i*likelihood_mapW+j] = likelihood_img.data[i*likelihood_mapW+j];
                markNum = cv_likelihood_map->data[i*likelihood_mapW+j]/255.0;
                likeliHood_map[i*likelihood_mapW+j] = markNum;
            }
        }
        for(int i=0;i<mapH;i++)
        {
            for(int j=0;j<mapW;j++)
            {
                if(binary_img.data[(i*binary_img.cols)+j]==0)
                {
                    map[(i*mapW)+j] = true;
                }else{
                    map[(i*mapW)+j] = false;
                }
            }
        }
        std::cout << "Loading Likelihood map success!" << std::endl;
    }
}

void ParticleFilter::get_mapSize(int *map_size)
{
    map_size[0] = mapW;
    map_size[1] = mapH;
}

void ParticleFilter::get_likelihoodMapSize(int *map_size)
{
    map_size[0] = likelihood_mapW;
    map_size[1] = likelihood_mapH;
}

double ParticleFilter::randomX() // return the value between 0 ~ N-1
{
    double x = rand() % (int)mapW/6;
    while(x<=0 || x>=mapW)
    {
        x = rand() % (int)mapW/6;
    }
    x = x-mapW/2/6;
    return x;
}

double ParticleFilter::randomY() // return the value between 0 ~ N-1
{
    double y = rand() % (int)mapH/5;
    while(y<=0 || y>=mapH)
    {
        y = rand() % (int)mapH/5;
    }
    y = y-mapH/2/5;
    return y;
}

void ParticleFilter::initParticle_Filter()
{
    pAry.clear();

    //gen pNum particle
    for(int i= 0;i < pNum;i++)
    {
        tmp_p.pos(0) = randomX();
        tmp_p.pos(1) = randomY();
        tmp_yaw = rand()%360;
        // tmp_p.pos(0) = -200;
        // tmp_p.pos(1) = 100;
        // tmp_yaw = 0;
        // std::cout << tmp_p.pos(0) << ", " << tmp_p.pos(1) << std::endl;

        tmp_p.yaw = tmp_yaw*(2*M_PI)/360;
        pAry.push_back(tmp_p);
    }
}

int ParticleFilter::rand_Range()
{
    int randNum = rand()%100;
    return randNum;
}


void ParticleFilter::moveParticle(geometry_msgs::Twist odom)
{
    int tmp0_noise[12];
    int tmp1_noise[12];
    Vector2d move;
    double rot;
    double move_dir;
    double move_dir_tmp = atan2(odom.linear.y,odom.linear.x);                                 // Direction
    double move_dist = hypot(odom.linear.x, odom.linear.y)*100;    // Distance

    // move(0) = tmp.linear.x*100;
    // move(1) = tmp.linear.y*100;
    rot = odom.angular.z;

    for(int i= 0;i < pAry.size();i++){
        //-------------高斯亂數
        Vector2d G_noise;
        G_noise(0) =0;
        G_noise(1) =0;
        for(int m=0;m<12;m++)
        {
            tmp0_noise[m] = rand_Range();
            tmp1_noise[m] = rand_Range();
        }
        for(int k=0;k<12;k++)
        {
            G_noise(0) +=tmp0_noise[k];
            G_noise(1) +=tmp1_noise[k];
        }

        G_noise(0) = (G_noise(0)/2-300)/300*10; //產生-5~5之間的亂數
        G_noise(1) = (G_noise(1)/2-300)/300*0.52; //產生-0.087~0.087的亂數(-5度~5度)

        move_dir = move_dir_tmp + pAry[i].yaw;
        // G_noise(0) = 0;
        // G_noise(1) = 0;
        move(0) = (move_dist+G_noise(0))*cos(move_dir+G_noise(1));
        move(1) = (move_dist+G_noise(0))*sin(move_dir+G_noise(1));

        pAry[i].pos = pAry[i].pos + move ;
        pAry[i].yaw = pAry[i].yaw + rot + G_noise(1);

        //re position of particle
        if(pAry[i].pos(0) <-mapW/2 || pAry[i].pos(0) >=mapW/2 || pAry[i].pos(1) <-mapH/2 || pAry[i].pos(1) >=mapH/2)
        {
            pAry[i].pos(0) = randomX();
            pAry[i].pos(1) = randomY();
            if(trust_imu)
                tmp_yaw = 0;
            else
                tmp_yaw = rand()%360;
            pAry[i].yaw = tmp_yaw*(2*M_PI)/360;
        }
    }
}

void ParticleFilter::Sim_LaserWall(Vector3d robot)
{
    std::cout << "Enter Sim_LaserWall!!!" << std::endl;
    std::cout << "Robot_x: " << robot(0) << ", Robot_y: " << robot(1) << std::endl;
    double nowDeg = 0;
    double eachDeg = (270.0/sensorLineNum);
    double nowRad;
    sensorWall_Dist =  new int[sensorLineNum];
    
    sensorWall_Pos.clear();
    for(int i = 0;i < sensorLineNum;i++){
        nowDeg = i * eachDeg-135;    //minus 90 degree for transform of quadrant
        nowRad = robot(2)+nowDeg*2*M_PI/360;

        for(int j=1;;j++)
        {
            Vector2d distPos(j,0);
            Rotation2Dd rot(nowRad);
            Vector2d addPos = rot * distPos;
            Vector2i addPosi = Vector2i(addPos(0),addPos(1));
            Vector2i newPos;
            newPos(0) =  robot(0) + addPosi(0);
            newPos(1) =  robot(1) + addPosi(1);
            int index = (newPos(1)+ mapH/2)*mapW + (newPos(0)+mapW/2);
            if(index < mapH*mapW){
                if(map[index])
                {
                    sensorWall_Pos.push_back(newPos);
                    sensorWall_Dist[i]=j;
                    break;
                }
            }else{
                std::cout << "index too big!!!" << std::endl; 
            }
        }
    }
    std::cout << "Sensor Wall Size: " << sensorWall_Pos.size() << std::endl;
}

void ParticleFilter::rateGrade(double scan_range, std::vector<double> laser_ranges)
{
    int laser_num = laser_ranges.size();
    tpos_wall.clear();
    double rate2;
    for(int i= 0;i < pAry.size();i++){
        double sumGrade = 0;
        double tmp_grade=1;                                 //----for plus
        // for real 
        Vector2i particle_pos(pAry[i].pos(0),pAry[i].pos(1));
        double P_yaw = pAry[i].yaw;

        // for debug 
        // Vector2i particle_pos(0,0);
        // double P_yaw = 0;

        int downsample_param = 1;
        double eachRad = scan_range/(laser_num/downsample_param);
        int laser_downsample_num = laser_num/downsample_param;

        for(int j=0;j<laser_downsample_num;j++)
        {
            int index = downsample_param*j;
            double nowRad = index*eachRad;
            nowRad = P_yaw + nowRad - scan_range/2;

            if(nowRad >= 2*3.14159)
            {
                nowRad = nowRad - 2*3.14159;
            }
            Vector2d addPos;
            addPos(0) = laser_ranges[index]*cos(nowRad)*100;    // *100 for convert from m to cm
            addPos(1) = laser_ranges[index]*sin(nowRad)*100;    // *100 for convert from m to cm
            Vector2i addPosi = Vector2i(addPos(0),addPos(1));

            tPos = particle_pos + addPosi;

            if(i==0)
            {
                tpos_wall.push_back(tPos);
            }


            // if(laser_ranges[index]>10)
            // {
            //     rate2 = 0.5;
            // }else{
            //     rate2 = 1;
            // }
            
            tPos(0) = tPos(0)+likelihood_mapW/2;
            tPos(1) = tPos(1)+likelihood_mapH/2;
            // std::cout << "x: " << tPos(0) << ", y: " << tPos(1) << std::endl;
            if(tPos(0) >= 0 && tPos(0) < likelihood_mapW && tPos(1) >= 0 && tPos(1) < likelihood_mapH)
            {
                sumGrade = tmp_grade * likeliHood_map[tPos(1)*likelihood_mapW + tPos(0)];
                tmp_grade = sumGrade;
            }else{
                sumGrade = tmp_grade*0.01;
                tmp_grade = sumGrade;
            }
        }
        //pAry[i].sumGrade = sumGrade*rate1;
        pAry[i].sumGrade = sumGrade;
        // std::cout << "sumGrade:" << sumGrade << std::endl;
    }
    // std::cout << "Grade: " << pAry[0].sumGrade << std::endl;
    std::cout << "X: " << pAry[0].pos[0] << ", Y: " << pAry[0].pos[1] << std::endl;
}

bool ParticleFilter::ISConvergence()
{
    Vector2d pre_pos;
    Vector2d pos;
    Vector2d tmp_sub;
    double tmp;
    for(int i=1;i<=pAry.size()*0.4;i++)
    {
        pre_pos(0) = pAry[i-1].pos(0);
        pre_pos(1) = pAry[i-1].pos(1);
        pos(0) = pAry[i].pos(0);
        pos(1) = pAry[i].pos(1);
        tmp_sub = pos-pre_pos;
        tmp = hypot(tmp_sub(0),tmp_sub(1));
        if(tmp==0)
            return true;
        else
            return false;
    }
}

void ParticleFilter::roulette_wheel_selection()
{
    double allPGardeSum = 0;
    double tmp_weight = 0;
    double xSum = 0;
    double ySum = 0;
    double yaw =0;
    std::vector<Particle,  Eigen::aligned_allocator<Particle> > newAry;

    for(int i= 0;i < pAry.size();i++)
    {
        allPGardeSum+= pAry[i].sumGrade;
    }
    //std::cout << allPGardeSum << std::endl;

    for(int i=0;i<pAry.size();i++)
    {
        tmp_weight += pAry[i].sumGrade/allPGardeSum;
        pAry[i].weight = tmp_weight;
    }

    //std::cout << "pAry[i].weight:" << pAry[0].weight <<std::endl;

    for(int i=0;i<pAry.size();i++)
    {
        double tmp_sel = rand()%100;
        double sel = tmp_sel/100;

        for(int j=0;j<pAry.size();j++)
        {
            if(sel < pAry[j].weight)
            {
                newAry.push_back(pAry[j]);
                break;
            }
        }
    }
    pAry = newAry;

    for(int m = 0;m<pAry.size();m++)
    {
        xSum += pAry[m].pos(0);
        ySum += pAry[m].pos(1);
        yaw += pAry[m].yaw;
    }

    predictionPose(0) = xSum/pAry.size();
    predictionPose(1) = ySum/pAry.size();
    predictionPose(2) = (yaw)/pAry.size();
}

bool sort_particle(Particle a, Particle b){
    return a.sumGrade > b.sumGrade;
}

void ParticleFilter::tournament_selection()
{
    double allPGardeSum = 0;
    double tmp_weight = 0;
    double xSum = 0;
    double ySum = 0;
    double yaw =0;
    int sel=0;

    std::vector<Particle,  Eigen::aligned_allocator<Particle> > newAry;

    //-------排序
    sort(pAry.begin(),pAry.end(), sort_particle);
    //-------排序

    //-------競爭取前40%
    for(int i=0;i<pAry.size();i++)
    {
        sel = rand()%25;
        newAry.push_back(pAry[sel]);
    }
    pAry = newAry;

    for(int i= 0;i < pAry.size();i++)
    {
        allPGardeSum+= pAry[i].sumGrade;
    }

    for(int m = 0;m<pAry.size();m++)
    {
        tmp_weight = pAry[m].sumGrade/allPGardeSum;
        xSum += pAry[m].pos(0)*tmp_weight;
        ySum += pAry[m].pos(1)*tmp_weight;
        yaw += pAry[m].yaw*tmp_weight;
    }

    predictionPose(0) = xSum;
    predictionPose(1) = ySum;
    predictionPose(2) = yaw;
}

Vector3d ParticleFilter::get_Robot_pos()
{
    Vector3d tmp_robot;
    return tmp_robot;
}

Vector3d ParticleFilter::get_Estimate_pose()
{
    Vector3d grid_pos(predictionPose(0)/100,predictionPose(1)/100,predictionPose(2));
    std::cout << "YAW: " << predictionPose(2) << std::endl;
    return grid_pos;
}

std::vector<Vector3d> ParticleFilter::get_Particle()
{
    posAry.clear();
    for(int i=0;i < pAry.size();i++){
        Vector3d grid_pos(pAry[i].pos(0),pAry[i].pos(1),pAry[i].yaw);
        posAry.push_back(grid_pos);
    }
    return  posAry;
}
std::vector<Vector2i> ParticleFilter::get_tpwall()
{
    return tpos_wall;
}

std::vector<Vector2i> ParticleFilter::get_simSensorWall()
{
    return sensorWall_Pos;
}
double* ParticleFilter::get_Likelihood_map()
{
    return likeliHood_map;
}