/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/BmsCmd.h>
#include <unitree_legged_msgs/BmsState.h>
#include <unitree_legged_msgs/IMU.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "geometry_msgs/Twist.h"

unitree_legged_msgs::Cartesian ToRos(UNITREE_LEGGED_SDK::Cartesian& lcm){
    unitree_legged_msgs::Cartesian ros;
    ros.x = lcm.x;
    ros.y = lcm.y;
    ros.z = lcm.z;
    return ros;
}

UNITREE_LEGGED_SDK::BmsCmd ToLcm(unitree_legged_msgs::BmsCmd& ros){
    UNITREE_LEGGED_SDK::BmsCmd lcm;
    lcm.off = ros.off;
    for(int i(0); i<3; ++i){
        lcm.reserve[i] = ros.reserve[i];
    }
    return lcm;
}

unitree_legged_msgs::BmsState ToRos(UNITREE_LEGGED_SDK::BmsState& lcm){
    unitree_legged_msgs::BmsState ros;
    ros.version_h = lcm.version_h;
    ros.version_l = lcm.version_l;
    ros.bms_status = lcm.bms_status;
    ros.SOC = lcm.SOC;
    ros.current = lcm.current;
    ros.cycle = lcm.cycle;
    for(int i(0); i<2; ++i){
        ros.BQ_NTC[i] = lcm.BQ_NTC[i];
        ros.MCU_NTC[i] = lcm.MCU_NTC[i];
    }
    for(int i(0); i<10; ++i){
        ros.cell_vol[i] = lcm.cell_vol[i];
    }
    return ros;
}

unitree_legged_msgs::IMU ToRos(UNITREE_LEGGED_SDK::IMU& lcm)
{
    unitree_legged_msgs::IMU ros;
    ros.quaternion[0] = lcm.quaternion[0];
    ros.quaternion[1] = lcm.quaternion[1];
    ros.quaternion[2] = lcm.quaternion[2];
    ros.quaternion[3] = lcm.quaternion[3];
    ros.gyroscope[0] = lcm.gyroscope[0];
    ros.gyroscope[1] = lcm.gyroscope[1];
    ros.gyroscope[2] = lcm.gyroscope[2];
    ros.accelerometer[0] = lcm.accelerometer[0];
    ros.accelerometer[1] = lcm.accelerometer[1];
    ros.accelerometer[2] = lcm.accelerometer[2];
    ros.temperature = lcm.temperature;
    return ros;
}

unitree_legged_msgs::MotorState ToRos(UNITREE_LEGGED_SDK::MotorState& lcm)
{
    unitree_legged_msgs::MotorState ros;
    ros.mode = lcm.mode;
    ros.q = lcm.q;
    ros.dq = lcm.dq;
    ros.ddq = lcm.ddq;
    ros.tauEst = lcm.tauEst;
    ros.q_raw = lcm.q_raw;
    ros.dq_raw = lcm.dq_raw;
    ros.ddq_raw = lcm.ddq_raw;
    ros.temperature = lcm.temperature;
    ros.reserve[0] = lcm.reserve[0];
    ros.reserve[1] = lcm.reserve[1];
    return ros;
}

UNITREE_LEGGED_SDK::MotorCmd ToLcm(unitree_legged_msgs::MotorCmd& ros, UNITREE_LEGGED_SDK::MotorCmd lcmType)
{
    UNITREE_LEGGED_SDK::MotorCmd lcm;
    lcm.mode = ros.mode;
    lcm.q = ros.q;
    lcm.dq = ros.dq;
    lcm.tau = ros.tau;
    lcm.Kp = ros.Kp;
    lcm.Kd = ros.Kd;
    lcm.reserve[0] = ros.reserve[0];
    lcm.reserve[1] = ros.reserve[1];
    lcm.reserve[2] = ros.reserve[2];
    return lcm;
}

unitree_legged_msgs::LowState ToRos(UNITREE_LEGGED_SDK::LowState& lcm)
{
    unitree_legged_msgs::LowState ros;
    ros.levelFlag = lcm.levelFlag;
    ros.commVersion = lcm.commVersion;
    ros.robotID = lcm.robotID;
    ros.SN = lcm.SN;
    ros.bandWidth = lcm.bandWidth;
    ros.imu = ToRos(lcm.imu);
    for(int i = 0; i<20; i++){
        ros.motorState[i] = ToRos(lcm.motorState[i]);
    }
    ros.bms = ToRos(lcm.bms);
    for(int i = 0; i<4; i++){
        ros.footForce[i] = lcm.footForce[i];
        ros.footForceEst[i] = lcm.footForceEst[i];
    }
    ros.tick = lcm.tick;
    for(int i = 0; i<40; i++){
        ros.wirelessRemote[i] = lcm.wirelessRemote[i];
    }
    ros.reserve = lcm.reserve;
    ros.crc = lcm.crc;
    return ros;
}

UNITREE_LEGGED_SDK::LowCmd ToLcm(unitree_legged_msgs::LowCmd& ros, UNITREE_LEGGED_SDK::LowCmd lcmType)
{
    UNITREE_LEGGED_SDK::LowCmd lcm;
    lcm.levelFlag = ros.levelFlag;
    lcm.commVersion = ros.commVersion;
    lcm.robotID = ros.robotID;
    lcm.SN = ros.SN;
    lcm.bandWidth = ros.bandWidth;
    for(int i = 0; i<20; i++){
        lcm.motorCmd[i] = ToLcm(ros.motorCmd[i], lcm.motorCmd[i]);
    }
    lcm.bms = ToLcm(ros.bms);
    for(int i = 0; i<40; i++){
        lcm.wirelessRemote[i] = ros.wirelessRemote[i];
    }
    lcm.reserve = ros.reserve;
    lcm.crc = ros.crc;
    return lcm;
}

unitree_legged_msgs::HighState ToRos(UNITREE_LEGGED_SDK::HighState& lcm)
{
    unitree_legged_msgs::HighState ros;
    ros.levelFlag = lcm.levelFlag;
    ros.commVersion = lcm.commVersion;
    ros.robotID = lcm.robotID;
    ros.SN = lcm.SN;
    ros.bandWidth = lcm.bandWidth;
    ros.mode = lcm.mode;
    ros.progress = lcm.progress;
    ros.imu = ToRos(lcm.imu);
    ros.gaitType = lcm.gaitType;
    ros.footRaiseHeight = lcm.footRaiseHeight;
    ros.bodyHeight = lcm.bodyHeight;
    ros.yawSpeed = lcm.yawSpeed;
    ros.bms = ToRos(lcm.bms);
    ros.reserve = lcm.reserve;
    ros.crc = lcm.crc;

    for(int i(0); i<3; ++i){
        ros.position[i] = lcm.position[i];
        ros.velocity[i] = lcm.velocity[i];
    }

    for(int i(0); i<4; ++i){
        ros.footPosition2Body[i] = ToRos(lcm.footPosition2Body[i]);
        ros.footSpeed2Body[i] = ToRos(lcm.footSpeed2Body[i]);
        ros.footForce[i] = lcm.footForce[i];
        ros.footForceEst[i] = lcm.footForceEst[i];
    }

    // for(int i(0); i<20; ++i){
    //     ros.temperature[i] = lcm.temperature[i];
    // }

    for(int i(0); i<40; ++i){
        ros.wirelessRemote[i] = lcm.wirelessRemote[i];
    }

    return ros;
}

UNITREE_LEGGED_SDK::HighCmd ToLcm(unitree_legged_msgs::HighCmd& ros, UNITREE_LEGGED_SDK::HighCmd lcmType)
{
    UNITREE_LEGGED_SDK::HighCmd lcm;
    lcm.levelFlag = ros.levelFlag;
    lcm.commVersion = ros.commVersion;
    lcm.robotID = ros.robotID;
    lcm.SN = ros.SN;
    lcm.bandWidth = ros.bandWidth;
    lcm.mode = ros.mode;
    lcm.gaitType = ros.gaitType;
    lcm.speedLevel = ros.speedLevel;
    lcm.footRaiseHeight = ros.footRaiseHeight;
    lcm.bodyHeight = ros.bodyHeight;
    lcm.yawSpeed = ros.yawSpeed;
    lcm.bms = ToLcm(ros.bms);
    lcm.reserve = ros.reserve;
    lcm.crc = ros.crc;

    for(int i(0); i<2; ++i){
        lcm.postion[i] = ros.postion[i];
        lcm.velocity[i] = ros.velocity[i];
    }

    for(int i(0); i<3; ++i){
        lcm.euler[i] = ros.euler[i];
    }

    for(int i = 0; i<4; i++){
        lcm.led[i].r = ros.led[i].r;
        lcm.led[i].g = ros.led[i].g;
        lcm.led[i].b = ros.led[i].b;
    }

    for(int i = 0; i<40; i++){
        lcm.wirelessRemote[i] = ros.wirelessRemote[i];
    }

    return lcm;
}

unitree_legged_msgs::HighCmd rosMsg2Cmd(const geometry_msgs::Twist::ConstPtr &msg)
{
    unitree_legged_msgs::HighCmd cmd;

    cmd.levelFlag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    cmd.mode = 0;
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0] = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;

    cmd.velocity[0] = msg->linear.x;
    cmd.velocity[1] = msg->linear.y;
    cmd.yawSpeed = msg->angular.z;

    cmd.mode = 2;
    cmd.gaitType = 1;

    return cmd;
}
unitree_legged_msgs::HighCmd rosMsg2Cmd_posture(const geometry_msgs::Twist::ConstPtr &msg)
{
    unitree_legged_msgs::HighCmd cmd;

    cmd.levelFlag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    cmd.mode = 0;
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0] = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;
    
    cmd.euler[1] = msg->linear.x;
    cmd.euler[2] = msg->angular.z;
    if (cmd.euler[1] >1.5)
       cmd.euler[1] = 1.5;
    else if (cmd.euler[1] <-1.5)
       cmd.euler[1] = -1.5;
    if (cmd.euler[2] >0.6)
       cmd.euler[2] = 0.6;
    else if (cmd.euler[2] <-0.6)
        cmd.euler[2] = -0.6;
    
    

    /*
    // 0. idle, default stand  1. force stand (controlled by dBodyHeight + ypr)
											// 2. target velocity walking (controlled by velocity + yawSpeed)
											// 3. target position walking (controlled by position + ypr[0])
											// 4. path mode walking (reserve for future release)
											// 5. position stand down. 
											// 6. position stand up 
											// 7. damping mode 
											// 8. recovery stand
											// 9. backflip
											// 10. jumpYaw
											// 11. straightHand
											// 12. dance1
											// 13. dance2

    std::array<float, 2> position;       // (unit: m), desired position in inertial frame
    std::array<float, 3> euler;         // (unit: rad), roll pitch yaw in stand mode
    std::array<float, 2> velocity;      // (unit: m/s), forwardSpeed, sideSpeed in body frame
    float yawSpeed;                     // (unit: rad/s), rotateSpeed in body frame
        //look_up/down (yaw)
        
    
    SendHighROS.pitch = -posture.linear.x;
    if(SendHighROS.pitch > 1.5)
        SendHighROS.pitch = 1.5;
    else if (SendHighROS.pitch < -1.5)
        SendHighROS.pitch = -1.5;
    //look_right/right (pitch)
    SendHighROS.yaw = -posture.angular.z;

    if(SendHighROS.yaw > 0.6)
        SendHighROS.yaw = 0.6;
    else if (SendHighROS.yaw < -0.6)
        SendHighROS.yaw = -0.6;
    */
    cmd.mode = 1;
    cmd.gaitType = 1;

    return cmd;
}







#endif  // _CONVERT_H_
