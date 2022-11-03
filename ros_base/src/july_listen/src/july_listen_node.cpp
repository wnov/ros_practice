#include "ros/ros.h"
#include "std_msgs/String.h"
#include "july_msgs/JulyMsg.h"
#include "july_msgs/JulyIntMsg.h"


void julyCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("I heard %s", msg->data.c_str());
}

void julyCallbackNew(const july_msgs::JulyMsg::ConstPtr &msg)
{
    ROS_INFO("I heard %s, %d th.", msg->detail.c_str(), msg->id);
}

void julyCallbackInt(const july_msgs::JulyIntMsg::ConstPtr &msg)
{
    int sum;
    sum = msg->msg_a + msg->msg_b;
    ROS_INFO("I heard %d and %d, sum of them is %d", msg->msg_a, msg->msg_b, sum);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "july_listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/july_topic", 10, julyCallback);
    ros::Subscriber sub_new = n.subscribe("/july_topic_new", 10, julyCallbackNew);
    ros::Subscriber sub_int = n.subscribe("/july_topic_int", 10, julyCallbackInt);

    ros::spin();

    return 0;
}
