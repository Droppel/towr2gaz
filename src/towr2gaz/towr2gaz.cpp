#include <ros/ros.h>
#include <xpp_msgs/RobotStateJoint.h>
#include <sensor_msgs/JointState.h>
#include <stdlib.h>

ros::Publisher pub;

bool first = true;
ros::Time startTime;

void jointstateMessageReceived(const xpp_msgs::RobotStateJoint& msg) {

    if (first) {
        first = false;
        startTime = ros::Time().now();
    }

    sensor_msgs::JointState sensormsg;
    sensormsg = msg.joint_state;
    sensormsg.name = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
                        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
    sensormsg.header.stamp = startTime + msg.time_from_start;

    ROS_INFO_STREAM(std::setprecision(2) << std::fixed << sensormsg);
    pub.publish(sensormsg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "towr2gaz");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("xpp/joint_hyq_des", 1000, &jointstateMessageReceived);
    pub = nh.advertise<sensor_msgs::JointState>("aliengo_gazebo/joint_states", 1000);

    ros::spin();
}