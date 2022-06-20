#include "sensor_node.h"

int main(int argc, char **argv){
    ros::init(argc,argv,"sensor_filtering_node");
    ros::NodeHandle nh;
    sensor_filtering_node sensor_filter(nh);
    ros::spin();
    return 0;
}

