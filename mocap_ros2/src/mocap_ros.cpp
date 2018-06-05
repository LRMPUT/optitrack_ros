#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include <termios.h>
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include "Mocap.hpp"
#include <memory>

using namespace std;

int main(int argc, char *argv[]) {
    
    Mocap mocap(argc, argv);
    Eigen::Vector3d retPos;
    Eigen::Quaterniond retOrient;
    
    ros::init(argc, argv, "mocap_ros2_node");
    ros::NodeHandle n("~");
    
    int nbodies;
    n.param("nbodies", nbodies, 1);
    
    vector<ros::Publisher> rbPubs;
    vector<uint> seqs;
    for(int r = 0; r < nbodies; ++r) {
        rbPubs.push_back(n.advertise<geometry_msgs::PoseStamped>("optitrack" + to_string(r), 1000));
        seqs.push_back(0);
    }
    ros::Rate loop_rate(200);
    
    
    int count = 0;
    while (ros::ok()) {
        vectorPose poses = mocap.getLatestPoses();
        ros::Time curTimestamp = ros::Time::now();
        
        for(const Pose &curPose : poses){
            int r = curPose.id - 1;
            
            geometry_msgs::Point point;
            point.x = curPose.t.x();
            point.y = curPose.t.y();
            point.z = curPose.t.z();

            geometry_msgs::Quaternion quat;
            quat.x = curPose.r.x();
            quat.y = curPose.r.y();
            quat.z = curPose.r.z();
            quat.w = curPose.r.w();

            geometry_msgs::PoseStamped posestamped;
            posestamped.pose.position = point;
            posestamped.pose.orientation = quat;
            posestamped.header.frame_id = "0";
            posestamped.header.stamp = curTimestamp;
            posestamped.header.seq = seqs[r]++;

//            cout << "publishing for " << r << endl;
            rbPubs[r].publish(posestamped);
        }
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
        
    }
    
}


