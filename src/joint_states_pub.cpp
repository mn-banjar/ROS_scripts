#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv)
{
    //Initializen the ros
    ros::init(argc, argv, "joint_states_pub");

    //Declare the node handle
    ros::NodeHandle node;

    //Decleare a joint state publisher
    ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("joint_states",10);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    sensor_msgs::JointState js0;
    js0.header.stamp = ros::Time::now();
    js0.name.resize(2);
    js0.name[0] = "gripper";
    js0.name[1] = "gripper2";
    js0.position.resize(2);
    js0.position[0] = 1.0472;
    js0.position[1] = 1.0472;
    joint_pub.publish(js0);

    ros::spinOnce();
    loop_rate.sleep();
  }

    return 0;
}
