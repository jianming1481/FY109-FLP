#include "ros/ros.h"
#include "monte_carlo_localization/particle_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_mcl");
    ParticleFilter pf;
    if (argc < 2)
    {
        string default_map_path = "/home/lui/map.yaml";
        pf.init(default_map_path);
    }else{
        pf.init(argv[1],500);
    }

    // ros::Rate loop_rate(10);
    // while (ros::ok())
    // {
    //     // pf.rate_particles();
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    return 0;
}
