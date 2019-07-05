#include "ros/ros.h"
#include "std_msgs/Float64.h"
//#include <sstream>


int main(int argc, char **argv)
{
    
ros::init(argc,argv,"arm_mover");
ros::NodeHandle n;

ros::Publisher rotator1_pub= n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command",10);
ros::Publisher rotator2_pub= n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command",10);

ros::Rate loop_rate(10);

char input_key;
int count=0;
float rotp=0.2;
float rotn=-0.2;
while(ros::ok()) 
{
    std_msgs::Float64 msg1;
    std_msgs::Float64 msg2;

    if (rotp>3.14)
    {
       rotp=0.2;
    }
    if (rotn>3.14)
    {
        rotn=0.2;
    }
    //msg1.data=rot;
    //msg2.data=-rot;


    //ROS_INFO("%d",count);

    std::cin>>input_key;
    if ('w'==input_key)
    {
        
        rotp=rotp+0.2;
        std::cout<<rotp<<"\n";
        msg1.data=rotp;
        rotator1_pub.publish(msg1);
    }
    else if ('s'==input_key)
    {
        
        rotp=rotp-0.2;
        std::cout<<rotp<<"\n";
        msg1.data=rotp;
        rotator1_pub.publish(msg1);        
    }
    else if ('a'==input_key)
    {
        
        rotn=0.2+rotn;
        std::cout<<rotn<<"\n";
        msg2.data=rotn;
        rotator2_pub.publish(msg2);
    }
    else if ('d'==input_key)
    {
        
        rotn=rotn-0.2;
        std::cout<<rotn<<"\n";
        msg2.data=-rotn;
        rotator2_pub.publish(msg2);
    }

    //rotator1_pub.publish(msg1);
    //rotator2_pub.publish(msg2); 
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
}

    return 0;
}
