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
#include "geometry_msgs/Pose.h"

// other
#include "Mocap.hpp"
#include "optitrack/RigidBody.h"
#include "optitrack/Marker.h"
#include <serial/serial.h>

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

    // SERIAL INIT
    serial::Serial ser;
    try{
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(3);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    //ROS
    ros::Rate loop_rate(240);
    int count = 0;
    //MAP
    static constexpr int mapBufferSize = 300;
    static constexpr int firstGrabbedFrames = 100;
    map<int,ros::Time> frameTimeStamp;
    bool firstFrameFlag=1;
    int firstFrameId=0;
    int localFrameId=0;
    int localCntFrame=1;
    int firstFramesCounter=0;
    static int firstMinCnt;
    // Create a map iterator and point to beginning of map
    std::map<int,ros::Time>::iterator MapIterator= frameTimeStamp.begin();

    //FT232
    std::vector<uint8_t> zero;
    zero.push_back(0);

    while (ros::ok()) {
        if(ser.isOpen() && (count %2==0)){
            localFrameId=localCntFrame+firstFrameId;
            frameTimeStamp[localFrameId]=ros::Time::now();
            ser.write(zero);
            localCntFrame++;
        }

        vectorPose poses = mocap.getLatestPoses();

        if(firstFrameFlag==0){
            if(poses.size() > 0 && poses[0].frameNum>0 && frameTimeStamp.count(poses[0].frameNum)>0 ){
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
                        optitrack::RigidBody rigidBody;
                        rigidBody.header.frame_id = "optitrack";
                        rigidBody.header.stamp = curTimestamp;
                        rigidBody.header.seq = seqs[r];
                        rigidBody.pose.position = point;
                        rigidBody.pose.orientation = quat;
                        rigidBody.timestamp = curPose.timestamp;
                        rigidBody.meanError = curPose.meanError;
                        for(const Marker &marker : curPose.markers){
                            optitrack::Marker markerRos;
                            markerRos.location.x = marker.location(0);
                            markerRos.location.y = marker.location(1);
                            markerRos.location.z = marker.location(2);
                            markerRos.residual = marker.residual;
                            markerRos.occluded = marker.occluded;
                            rigidBody.markers.push_back(markerRos);
                        }
                        rbDebugPubs[r].publish(rigidBody);
                    }
                    ++seqs[r];
                }
                while(MapIterator->first < poses[0].frameNum - mapBufferSize ){
                    MapIterator= frameTimeStamp.erase(MapIterator);
                }

            }else{

            }

        }


        if(firstFrameFlag && poses[0].frameNum!=-1){

            if (firstFramesCounter++ >= firstGrabbedFrames) {
                firstFrameFlag=0;
            }

            if(firstMinCnt==0){
                firstMinCnt++;
                firstFrameId=poses[0].frameNum;
            }

            if(poses[0].frameNum<firstFrameId){
                firstFrameId=poses[0].frameNum;
            }
            MapIterator= frameTimeStamp.begin();
            ROS_INFO("ROS:Frame first id: %d",firstFrameId);
        }


        ros::spinOnce();
        loop_rate.sleep();
        ++count;

    }
    ser.close();
    cout<<"ROS:NODE:STOP"<<endl;
    return 0;

}


