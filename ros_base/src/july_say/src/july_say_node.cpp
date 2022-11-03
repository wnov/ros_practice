#include <sstream>

#include "july_msgs/JulyIntMsg.h"
#include "july_msgs/JulyMsg.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "july_talker");
  ros::NodeHandle n;
  ros::Publisher july_pub = n.advertise<std_msgs::String>("/july_topic", 10);
  ros::Publisher july_pub_new =
      n.advertise<july_msgs::JulyMsg>("/july_topic_new", 10);
  ros::Publisher july_pub_int =
      n.advertise<july_msgs::JulyIntMsg>("/july_topic_int", 10);
  ros::Rate loop_rate(10);
  int count = 0;

  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Hello july " << count++;
    msg.data = ss.str();
    july_pub.publish(msg);

    std::string param_str;
    n.param<std::string>("my_param", param_str, "Hi, july");
    july_msgs::JulyMsg julyMsg;
    julyMsg.id = count;
    julyMsg.detail = param_str;  // "Hello july new!";
    july_pub_new.publish(julyMsg);

    int msg_a, msg_b;
    n.param<int>("msg_a", msg_a, 0);
    n.param<int>("msg_b", msg_b, 0);
    july_msgs::JulyIntMsg julyIntMsg;
    julyIntMsg.msg_a = msg_a;
    julyIntMsg.msg_b = msg_b;
    july_pub_int.publish(julyIntMsg);

    loop_rate.sleep();
  }
}
