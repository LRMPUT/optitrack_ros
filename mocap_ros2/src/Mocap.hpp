/*
    Copyright (c) 2017 Mobile Robots Laboratory at Poznan University of Technology:
    -Jan Wietrzykowski name.surname [at] put.poznan.pl

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef RECORDING_OPTITRACK_MOCAP_HPP
#define RECORDING_OPTITRACK_MOCAP_HPP

#include <Eigen/Eigen>
//#include <eigen3/Eigen/Dense>
#include <memory>
#include <iostream>

#include "NatNetLinux/CommandListener.h"
#include "NatNetLinux/FrameListener.h"

class Mocap {
public:
    Mocap(int argc, char* argv[]);

    ~Mocap();

    bool getLatestPose(Eigen::Vector3d& retPos,
                        Eigen::Quaterniond& retOrient);
    
    bool getLatestPose(Eigen::Vector3d& retPos,
                       Eigen::Quaterniond& retOrient,
                       int id);
private:
    void readOpts(int argc, char* argv[]);

    uint32_t localAddress;
    uint32_t serverAddress;

    // Version number of the NatNet protocol, as reported by the server.
    unsigned char natNetMajor;
    unsigned char natNetMinor;

    // Sockets
    int sdCommand;
    int sdData;

    std::shared_ptr<CommandListener> commandListener;
    std::shared_ptr<FrameListener> frameListener;

    bool lastMocapFrameValid;

    MocapFrame lastMocapFrame;
    
    // ids of rigid bodies for which data was fetched
    std::set<int> dataFetched;
};


#endif //RECORDING_OPTITRACK_MOCAP_HPP
