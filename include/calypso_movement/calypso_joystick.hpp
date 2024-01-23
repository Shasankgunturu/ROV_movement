#include <ros/ros.h>
#include "sensor_msgs/Joy.h"

class Controller {
    public:
        std::int16_t mode_changed = 0;
        std::int16_t disarm_changed = 0;
        std::int16_t arm_changed = 1;
        uint16_t channel_ary[8]={65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535};
        Controller(ros::NodeHandle nh) {
            joysub = nh.subscribe("/joy", 1, &Controller::joyCallback, this);
        }
    private:
        ros::Subscriber joysub;
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
            std::cout << "receving out" << std::endl;
            float lateral           = msg->axes[0];
            float forward           = msg->axes[1];
            float yaw               = msg->axes[2];
            float throtle           = msg->axes[3];
            float pitch_up          = msg->axes[4];
            float pitch_down        = msg->axes[5];
            int   roll_counter_clk  = msg->buttons[6];
            int   roll_clk          = msg->buttons[7];
            int   mode_switch       = msg->buttons[0];
            int   arm            = msg->buttons[1];
            channel_ary[2]  = 1500+(throtle*400);                              //throttle
            channel_ary[1]  = 1500+(roll_counter_clk*-400)+(roll_clk*400);     //roll
            channel_ary[0]  = 1500+((pitch_up+1)*-200)+((pitch_down+1)*200);   //pitch
            channel_ary[3]  = 1500+(yaw*-400);                                 //yaw
            channel_ary[4]  = 1500+(forward*400);                              //forward
            channel_ary[5]  = 1500+(lateral*-400);                             //lateral
            channel_ary[6]  = UINT16_MAX;
            channel_ary[7]  = UINT16_MAX;
            // if (disarm==1) {
            //     if ((arm_changed == 1) && (disarm_changed == 0)) {
            //         disarm_changed = 1;
            //         arm_changed = 0;
            //     }
            //     else {
            //         arm_changed = 1;
            //         disarm_changed = 0;  
            //     }
            // }
            if (arm == 1) {
                arm_changed = 1;
            }
            if (mode_switch == 1) {
                mode_changed = 1;
            }
            // for(int i=0;i<8;i++){
            //     if((channel_ary[i]>1450)&&(channel_ary[i]<1550))
            //     channel_ary[i]=UINT16_MAX;
    		// }
        
        }
};
