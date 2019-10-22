#include "monte_carlo_localization/laser_map_publisher.h"
#include "monte_carlo_localization/laser_map_reader.h"
#include "monte_carlo_localization/laser_map.h"
#include "monte_carlo_localization/laser_likelihood_map_builder.h"
#include "monte_carlo_localization/laser_likelihood_map_publisher.h"
#include "monte_carlo_localization/laser_likelihood_map.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_likelihood_map_publisher");
    LaserMapPublisher laser_map_publisher;
    LaserMapReader laser_map_reader;
    LaserMap map;

    laser_map_reader.read_yaml(argv[1]);
    map = laser_map_reader.get_map();
    laser_map_publisher.generate_OccGridMapMsg(map);

    LaserLikelihoodMapBuilder likelihood_map_builder;
    LaserLikelihoodMap likelihood_map;

    likelihood_map_builder.build_likelihood_map(map);
    likelihood_map_builder.save_cv_likelihood_map();
    likelihood_map = likelihood_map_builder.get_likelihood_map();

    LaserLikelihoodMapPublisher likelihood_map_publisher;
    likelihood_map_publisher.generate_OccGridMapMsg(likelihood_map);

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        likelihood_map_publisher.publish_map();
        laser_map_publisher.publish_map();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}