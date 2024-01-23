#include <calypso_movement/calypso_subs.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Mavlink.h>
#include <sensor_msgs/Joy.h>

uint16_t channel_ary[8] ={1500,1500,1500,1500,1500,1500,65535,65535};
mavros_msgs::Mavlink mavFrom;


void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
            float lateral = msg->axes[0];
            float forward = msg->axes[1];
            float yaw = msg->axes[2];
            float pitch = msg->axes[3];
            float thrust_up = msg->axes[4];
            float thrust_down = msg->axes[5];

            int roll_counter_clk = msg->buttons[6];
            int roll_clk = msg->buttons[7];
            
            channel_ary[0]=1500+(pitch*400);
            channel_ary[1]=1500+(roll_counter_clk*-400)+(roll_clk*400);
            channel_ary[2]=1500+((thrust_up+1)*-200)+((thrust_down+1)*200);
            channel_ary[3]=1500+(yaw*-400);
            channel_ary[4]=1500+(forward*400);
            channel_ary[5]=1500+(lateral*-400);
            channel_ary[6]=UINT16_MAX;
            channel_ary[7]=UINT16_MAX;
            
}



// void getPressure(const mavros_msgs::Mavlink::ConstPtr &msg) {
//     mavFrom = *msg;
//     if (mavFrom.msgid) {
//         std::cout << "Package: %s", msg->header;

//         // Transform the payload in a C++ string
//         std::string payload_str(reinterpret_cast<const char*>(msg->payload64.data()), msg->payload64.size());

//         // Transform the string into valid values
//         uint32_t time_boot_ms;
//         float press_abs, press_diff;
//         int16_t temperature;

//         std::memcpy(&time_boot_ms, payload_str.data(), sizeof(time_boot_ms));
//         std::memcpy(&press_abs, payload_str.data() + sizeof(time_boot_ms), sizeof(press_abs));
//         std::memcpy(&press_diff, payload_str.data() + sizeof(time_boot_ms) + sizeof(press_abs), sizeof(press_diff));
//         std::memcpy(&temperature, payload_str.data() + sizeof(time_boot_ms) + sizeof(press_abs) + sizeof(press_diff), sizeof(temperature));
//     }
// }

void go() {
    ros::NodeHandle nh;
    ros::Rate rate(100);
    ros::Publisher  thrusterPub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
    // ros::Subscriber pressureSub = nh.subscribe("/mavlink/from", 1, getPressure);
    ros::Subscriber joysub = nh.subscribe("/joy", 1, joyCallback);
    Subscriber sub(nh);        
    
    sub.connect();
    
    
    while(ros::ok()){
        //ROS_INFO("publishing to ovverrideRCin");
        mavros_msgs::OverrideRCIn thrusterCommand;
        thrusterCommand.channels[0]=channel_ary[0];
        thrusterCommand.channels[1]=channel_ary[1];
        thrusterCommand.channels[2]=channel_ary[2];
        thrusterCommand.channels[3]=channel_ary[3];
        thrusterCommand.channels[4]=channel_ary[4];
        thrusterCommand.channels[5]=channel_ary[5];
        thrusterCommand.channels[6]=channel_ary[6];
        thrusterCommand.channels[7]=channel_ary[7];
        sub.arm();
        sub.guided_armed();
        thrusterPub.publish(thrusterCommand);
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "thrusters");
    go();
}
