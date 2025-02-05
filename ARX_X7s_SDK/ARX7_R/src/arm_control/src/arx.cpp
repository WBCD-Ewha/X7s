#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include "utility.h"
#include "Hardware/can.h"
#include "Hardware/motor.h"
#include "Hardware/teleop.h"
#include "App/arm_control.h"
#include "App/keyboard.h"
#include "App/play.h"
#include "App/solve.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <atomic>
#include <signal.h>
#include "arm_control/PosCmd.h"
#include "arm_control/JointControl.h"
#include "arm_control/ChassisCtrl.h"
#include "arm_control/JointInformation.h"
#include "arm_control/CtrlTeach.h"


int CONTROL_MODE=1; //0left  1right
command cmd;

bool app_stopped = false;
void sigint_handler(int sig);
void safe_stop(can CAN_Handlej);
bool ros_cmd = 1;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arx7_r"); 
    ros::NodeHandle node;
    Teleop_Use()->teleop_init(node);

    arx_arm ARX_ARM((int) CONTROL_MODE);

    ros::Subscriber sub_cmd = node.subscribe<arm_control::PosCmd>("/ARX_VR_R", 10, 
                                [&ARX_ARM](const arm_control::PosCmd::ConstPtr& msg) {
                                    ARX_ARM.arx5_cmd.x_t          = msg->x;
                                    ARX_ARM.arx5_cmd.y_t          = msg->y;
                                    ARX_ARM.arx5_cmd.z_t          = msg->z;
                                    ARX_ARM.arx5_cmd.waist_roll = msg->roll;
                                    ARX_ARM.arx5_cmd.waist_pitch  = msg->pitch;
                                    ARX_ARM.arx5_cmd.waist_yaw    = msg->yaw;
                                    ARX_ARM.arx5_cmd.gripper      = msg-> gripper;
                                });

                                    ros::Subscriber sub_teachcmd = node.subscribe<arm_control::CtrlTeach>("/ctrlTeachMode", 10,
                                                                  [&ARX_ARM](const arm_control::CtrlTeach::ConstPtr &msg)
                                                                  {
                                                                      ARX_ARM.teachMode = msg->mode;
                                                                  });


    ros::Publisher pub_current = node.advertise<arm_control::JointInformation>("joint_information2", 10);
    ros::Publisher pub_pos = node.advertise<arm_control::PosCmd>("/follow2_pos_back", 10);
    

    arx5_keyboard ARX_KEYBOARD;

    ros::Rate loop_rate(200);
    can CAN_Handlej;

    std::thread keyThread(&arx5_keyboard::detectKeyPress, &ARX_KEYBOARD);
    sleep(1);

    while(ros::ok())
    { 
        printf(" ARX_ARM.teachMode_R: %d\n",  ARX_ARM.teachMode);

        char key = ARX_KEYBOARD.keyPress.load();
        ARX_ARM.getKey(key);

        ARX_ARM.get_joint();
        if(ros_cmd)
        {ARX_ARM.limit_cmd();}
        else
        {ARX_ARM.get_cmd();}
        
        ARX_ARM.update_real(cmd);
    
                                arm_control::JointInformation msg_joint;   

                                for(int i=0;i<8;i++)
                                {
                                    msg_joint.joint_pos[i] = ARX_ARM.current_pos[i];
                                    msg_joint.joint_vel[i] = ARX_ARM.current_vel[i];
                                    msg_joint.joint_cur[i] = ARX_ARM.current_torque[i];
                                }    

                                pub_current.publish(msg_joint);

                    //发送末端姿态
                                arm_control::PosCmd msg_pos_back;            
                                msg_pos_back.x      =ARX_ARM.solve.End_Effector_Pose[0];
                                msg_pos_back.y      =ARX_ARM.solve.End_Effector_Pose[1];
                                msg_pos_back.z      =ARX_ARM.solve.End_Effector_Pose[2];
                                msg_pos_back.roll   =ARX_ARM.solve.End_Effector_Pose[3];
                                msg_pos_back.pitch  =ARX_ARM.solve.End_Effector_Pose[4];
                                msg_pos_back.yaw    =ARX_ARM.solve.End_Effector_Pose[5];
                                msg_pos_back.gripper=ARX_ARM.current_pos[7];

                                pub_pos.publish(msg_pos_back);      



        ros::spinOnce();
        loop_rate.sleep();
        CAN_Handlej.arx_1();
        
    }
    CAN_Handlej.arx_2();
    return 0;
}