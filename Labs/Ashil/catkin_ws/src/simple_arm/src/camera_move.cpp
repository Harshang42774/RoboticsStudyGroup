#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"camera_move");
    ros::NodeHandle n;
    ros::Publisher rotation1_pub= n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command",10);
    ros::Publisher rotation2_pub= n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command",10);

ros::Rate loop_rate(10);

    return 0;
}