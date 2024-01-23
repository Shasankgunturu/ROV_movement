#include <calypso_movement/calypso_subs.hpp>
#include <calypso_movement/calypso_joystick.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Mavlink.h>
#include "sensor_msgs/Joy.h"
// mavros_msgs::Mavlink mavFrom;

void go() {
    ros::NodeHandle nh;
    std::string mode_name;
    ros::Rate rate(100);
    ros::Publisher  thrusterPub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
    // ros::Subscriber pressureSub = nh.subscribe("/mavlink/from", 1, getPressure);
    Subscriber sub(nh);
    Controller control(nh);        
    sub.connect();
    while(ros::ok()){ 
        mavros_msgs::OverrideRCIn thrusterCommand;
        thrusterCommand.channels[0]=control.channel_ary[0];
        thrusterCommand.channels[1]=control.channel_ary[1];
        thrusterCommand.channels[2]=control.channel_ary[2];
        thrusterCommand.channels[3]=control.channel_ary[3];
        thrusterCommand.channels[4]=control.channel_ary[4];
        thrusterCommand.channels[5]=control.channel_ary[5];
        thrusterCommand.channels[6]=control.channel_ary[6];
        thrusterCommand.channels[7]=control.channel_ary[7];
        sub.arm();
        if (control.mode_changed == 1) {
            sub.disarm();
            std::cout << "enter the mode: ";
            std::cin  >> mode_name;
            sub.mode_name = mode_name;
            sub.set_mode();
            sub.arm();
            control.mode_changed = 0;
        }
        thrusterPub.publish(thrusterCommand);
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "thrusters");
    go();
}