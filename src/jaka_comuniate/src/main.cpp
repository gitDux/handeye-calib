#include <iostream>
#include "ros/ros.h"

#include "jsoncpp/json/json.h"
#include "comuniate/client.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "JAKAZuRobot.h"

using std::endl;
using namespace std;

double radu(double angl)
{
    return angl/180*3.1415926;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"joint_state_publisher");
    ros::NodeHandle nh;
    ros::Publisher joint_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher jaka_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("jaka_pose", 1);
    ros::Rate rate = ros::Rate(10);

    //实例API对象demo  
	JAKAZuRobot m_robot;


    std::string ip;
    ros::param::get("/jaka_comuniate/jaka_host", ip);
    ROS_INFO_STREAM("Load Jaka Ip Param:"<<ip);
    if (ip == ""){
        ROS_ERROR("Can't get ip addr ,Please check param");
    }
    //登陆控制器  
	m_robot.login_in(ip.c_str());
    long frame_id =0;

    while (ros::ok())
    {
        RobotStatus m_status{};
        errno_t m_flag = m_robot.get_robot_status(&m_status);
        

        if(m_flag == ERR_SUCC){
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(6);
            joint_state.position.resize(6);
            joint_state.name={"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
            joint_state.position = {
                radu(m_status.joint_position[0]),
                radu(m_status.joint_position[1]),
                radu(m_status.joint_position[2]),
                radu(m_status.joint_position[3]),
                radu(m_status.joint_position[4]),
                radu(m_status.joint_position[5])
                };
            joint_publisher.publish(joint_state);
        }

        if(m_flag == ERR_SUCC)
        {
            geometry_msgs::PoseStamped pose;
            frame_id++;
            pose.header.frame_id = std::to_string(frame_id);
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = m_status.cartesiantran_position[0]/1000;
            pose.pose.position.y = m_status.cartesiantran_position[1]/1000;
            pose.pose.position.z = m_status.cartesiantran_position[2]/1000;
            geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(m_status.cartesiantran_position[3]*3.1415926/180,m_status.cartesiantran_position[4]/180.0*3.1415926,m_status.cartesiantran_position[5]*3.1415926/180);
            pose.pose.orientation.x = q.x;
            pose.pose.orientation.y = q.y;
            pose.pose.orientation.z = q.z;
            pose.pose.orientation.w = q.w;
            jaka_pose_publisher.publish(pose);
        }
      
        rate.sleep();
    }
    return 0;
}
