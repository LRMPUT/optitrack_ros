// STD
#include <sstream>
#include <iostream>
#include <memory>
#include <termios.h>
#include <cstdio>
#include <cstdio>

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

// other
#include "Mocap.hpp"
#include "optitrack/RigidBody.h"
#include "optitrack/Marker.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

int main(int argc, char *argv[]) {
    
    Eigen::Vector3d retPos;
    Eigen::Quaterniond retOrient;
    
    ros::init(argc, argv, "optitrack_node");
    ros::NodeHandle n("~");
    
    int nbodies;
    n.param("nbodies", nbodies, 1);
    
    ROS_INFO("Number of rigid bodies to track: %d", nbodies);
    
    string localAddress, serverAddress;
    if(!n.getParam("local_address", localAddress)){
        ROS_ERROR("Could not read local_address from parameters");
        ros::shutdown();
    }
    if(!n.getParam("server_address", serverAddress)){
        ROS_ERROR("Could not read server_address from parameters");
        ros::shutdown();
    }
    
    Mocap mocap(localAddress, serverAddress);
    
    vector<ros::Publisher> rbPubs;
    vector<ros::Publisher> rbDebugPubs;
    vector<uint> seqs;
    for(int r = 0; r < nbodies; ++r) {
        rbPubs.push_back(n.advertise<geometry_msgs::PoseStamped>("rigid_body_" + to_string(r), 1000));
        rbDebugPubs.push_back(n.advertise<optitrack::RigidBody>("rigid_body_debug_" + to_string(r), 1000));
        seqs.push_back(0);
    }
    ros::Rate loop_rate(1000);


    static tf2_ros::TransformBroadcaster br;

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

            {
                geometry_msgs::PoseStamped poseStamped;
                poseStamped.header.frame_id = "optitrack";
                poseStamped.header.stamp = curTimestamp;
                poseStamped.header.seq = seqs[r];
                poseStamped.pose.position = point;
                poseStamped.pose.orientation = quat;

//            cout << "publishing for " << r << endl;
//            cout << "cameraMidExposure timestamp = " << curPose.cameraMidExposureTimestamp << endl;
                rbPubs[r].publish(poseStamped);
            }
            {

                geometry_msgs::TransformStamped transformStamped;

                transformStamped.header.stamp = curTimestamp;
                transformStamped.header.frame_id = "optitrack";
                transformStamped.child_frame_id = "opti";
                transformStamped.transform.translation.x = point.x;
                transformStamped.transform.translation.y = point.y;
                transformStamped.transform.translation.z = point.z;
                transformStamped.transform.rotation.x = quat.x;
                transformStamped.transform.rotation.y = quat.y;
                transformStamped.transform.rotation.z = quat.z;
                transformStamped.transform.rotation.w = quat.w;

                br.sendTransform(transformStamped);
            }
            {
//                optitrack::RigidBody rigidBody;
//                rigidBody.header.frame_id = "optitrack";
//                rigidBody.header.stamp = curTimestamp;
//                rigidBody.header.seq = seqs[r];
//                rigidBody.pose.position = point;
//                rigidBody.pose.orientation = quat;
//                rigidBody.timestamp = curPose.timestamp;
//                rigidBody.meanError = curPose.meanError;
//                for(const Marker &marker : curPose.markers){
//                    optitrack::Marker markerRos;
//                    markerRos.location.x = marker.location(0);
//                    markerRos.location.y = marker.location(1);
//                    markerRos.location.z = marker.location(2);
//                    markerRos.residual = marker.residual;
//                    markerRos.occluded = marker.occluded;
//                    rigidBody.markers.push_back(markerRos);
//                }
//
//                rbDebugPubs[r].publish(rigidBody);
            }

            ++seqs[r];
        }
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
        
    }
    
}


