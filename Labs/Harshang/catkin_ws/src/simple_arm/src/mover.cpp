#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "simple_arm/GoToPosition.h"

//publisher variables for the two joints
ros::Publisher joint1_, joint2_;

//callback function for moving the arm robot
bool move_request(simple_arm::GoToPosition::Request& req, simple_arm::GoToPosition::Response& res)
{
    ROS_INFO("GoToPositionRequest Received - j1:%1.2f, j2:%1.2f",(float)req.joint_1,(float)req.joint_2);

    std_msgs::Float64 angle1, angle2;

    angle1.data = (float)req.joint_1;
    angle2.data = (float)req.joint_2;

    //The movement is pusblished for the simple arm robot, it will be executed now
    joint1_.publish(angle1);
    joint2_.publish(angle2);

    ros::Duration(3).sleep();

    res.msg_feedback = "Joint angle changed";
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}


int main(int argc, char *argv[])
{
    //ros node initialised
    ros::init(argc, argv, "mover");
    ros::NodeHandle nh_;

    //Advertisements of publisher variables defined
    joint1_ = nh_.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command",10);
    joint2_ = nh_.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command",10);

    //Service for movement defined here and callback function associated with it
    ros::ServiceServer service = nh_.advertiseService("/mover", move_request);
    ROS_INFO("Ready to send joint commands");

    ros::spin();

    return 0;
}

