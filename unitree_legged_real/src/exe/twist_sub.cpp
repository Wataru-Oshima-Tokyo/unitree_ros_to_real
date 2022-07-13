/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "convert.h"
#include "geometry_msgs/Twist.h"

using namespace UNITREE_LEGGED_SDK;


template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

class Custom
{
public:

    unitree_legged_msgs::HighCmd high_cmd;
    unitree_legged_msgs::HighState high_state;

    unitree_legged_msgs::LowCmd low_cmd;
    unitree_legged_msgs::LowState low_state;





    template<typename TCmd, typename TState, typename TLCM>
    int mainHelper(int argc, char *argv[], TLCM &roslcm)
    {
        std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
                << "Make sure the robot is standing on the ground." << std::endl
                << "Press Enter to continue..." << std::endl;
        std::cin.ignore();

        ros::NodeHandle n;
        ros::Rate loop_rate(500);

        // SetLevel(HIGHLEVEL);
        long motiontime = 0;
        TCmd SendHighLCM = {0};
        TState RecvHighLCM = {0};
        unitree_legged_msgs::HighState RecvHighROS;

        roslcm.SubscribeState();

        pthread_t tid;
        pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

        while (ros::ok()){
            motiontime = motiontime+2;
            roslcm.Get(RecvHighLCM);
            custom.high_state = ToRos(RecvHighLCM);
            SendHighLCM = ToLcm(custpm.high_cmd, SendHighLCM);
            roslcm.Send(SendHighLCM);
            ros::spinOnce();
            loop_rate.sleep(); 
        }
        return 0;
    }


    long cmd_vel_count = 0;

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);

        high_cmd = rosMsg2Cmd(msg);

        printf("cmd_x_vel = %f\n", high_cmd.velocity[0]);
        printf("cmd_y_vel = %f\n", high_cmd.velocity[1]);
        printf("cmd_yaw_vel = %f\n", high_cmd.yawSpeed);


        pub_high.publish(high_states);

        printf("cmdVelCallback ending!\t%ld\n\n", cmd_vel_count++);
    }

};



ros::Subscriber sub_cmd_vel;
ros::Publisher pub_high;

int main(int argc, char *argv[]){
    ros::init(argc, argv, "walk_ros_mode");
    ros::NodeHandle nh;
    Custom custom;
    pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);
    sub_cmd_vel = nh.subscribe("cmd_vel", 1, &Custom::cmdVelCallback, &custom);
    UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
    custom.mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);

}