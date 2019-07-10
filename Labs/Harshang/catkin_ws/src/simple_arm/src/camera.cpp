#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Image.h"
#include "cstdlib"

//client that can request services defined here
ros::ServiceClient client;

//random values creators
float random_return()
{
    return rand() % 13 - 6.28;
}

//Fuction that sends requests for the joints to move
void move_arm()
{
    ROS_INFO_STREAM("Moving the arm");

    //service variable declared
    simple_arm::GoToPosition srv;

    float joint_1_data = random_return();
    float joint_2_data = random_return();

    //the request send for random movements through service variables
    srv.request.joint_1 = joint_1_data;
    srv.request.joint_2 = joint_2_data;

    //delay call
    ros::Duration(5).sleep();

    if(!client.call(srv))
        ROS_ERROR("Failed to call service move_arm");
}

//The service that is provided based on image data
void look_another_place(const sensor_msgs::Image img)
{
    bool uniform_image = true;

    //check if the image is uniform or not
    for(int i =0; i< img.height*img.step; i++)
    {
        if(img.data[i] - img.data[0] != 0)
        {
            uniform_image = false;
        }            
    }

    if(uniform_image)
    {
        //If the image is similar, this node is going to stop with the current position
        ROS_INFO("The image is Symmetric - Killing this node");
        ros::shutdown();
    }
    else
    {
        move_arm();
    }
}


int main(int argc, char *argv[])
{
    //ros node is initialized here
    ros::init(argc,argv, "camera");
    ros::NodeHandle nh_;

    //client is for the topic mover
    client = nh_.serviceClient<simple_arm::GoToPosition>("/mover");

    //subscribed to image from the camera
    ros::Subscriber sub1 = nh_.subscribe("/rgb_camera/image_raw",1,look_another_place);

    ros::spin();

    return 0;

}
