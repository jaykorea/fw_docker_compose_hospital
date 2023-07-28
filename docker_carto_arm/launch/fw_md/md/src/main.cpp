///v1.9d modifying code while PNT_TQ_OFF(PNT_BREKE) communication. it request PNT_MAIN_DATA and for 2s delay
///v1.9e adding code reset command after 2s
///v1.9f adding code reset command when RMID is MDT'
///v1.9g modifying 'Md.sCmdAngularVel calculation' in cmd_main.cpp according to "md_node/angleresolution"
#include "md/global.hpp"
#include "md/main.hpp"
#include "md/com.hpp"
#include "md/robot.hpp"

#include "md/vel_msg.h"
#include "md/monitor_msg.h"

#include <ros/ros.h>
//#include <thread>
#include <boost/bind.hpp>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <math.h>
#include <sstream>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "freeway_msgs/DistanceTimeCalculator.h"

#define SET_STOP_STATUS false

bool e_stop_flag = false;
bool release_flag = false;
bool once_flag = false;
bool pre_release_flag = false;
bool front_obstacle_detected = false;
float ttc_time;
freeway_msgs::DistanceTimeCalculator dt_msg;
geometry_msgs::Twist cmd_vel_msg;
nav_msgs::Odometry odom_msg;
std_msgs::Int8 resume_feedback_msg;

ros::Publisher cmd_emer_super_pub;
ros::Publisher cancel_pub;
ros::Publisher resume_pub;
ros::ServiceClient clearcostmap_pub;

ros::Timer e_stop_timer;
ros::Timer resume_timer;

TableOfRobotControlRegister RB;
Communication Com;
LOG Log;
//
//It is a message callback function.
//It is a function that oprates when a topic message named 'vel_topic' is received.
//The input message is to receive the vel_msg message from 'md' package in msg directory
void velCallBack(const md::vel_msg::ConstPtr& vel)
{
    Com.nCmdSpeed       = vel->nLinear;
    Com.nCmdAngSpeed    = vel->nAngular;
    Com.fgResetOdometry = vel->byResetOdometry;
    Com.fgResetAngle    = vel->byResetAngle;
    Com.fgResetAlarm    = vel->byResetAlarm;
}

void init_set_brake_stop_cb(const std_msgs::Empty& msg)
{
    InitSetBrakeStop();
}

void release_cb(const std_msgs::Bool::ConstPtr& release_msg) {
    release_flag = release_msg->data;
}

void front_obstacle_update_cb(const std_msgs::Bool::ConstPtr& msg) {
   front_obstacle_detected = msg->data;
}

void cmd_vel_update_cb(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_vel_msg = *msg;
}

void odom_msg_update_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_msg = *msg;
}

void dt_update_cb(const freeway_msgs::DistanceTimeCalculator::ConstPtr& msg) {
    dt_msg = *msg;
}

void resume_feedback_update_cb(const std_msgs::Int8::ConstPtr& msg) {
    resume_feedback_msg = *msg;
}
//
//void e_stop_thread_function()
//{
//    ros::Rate r(40); // Set the loop rate to 40 Hz
//    while (ros::ok())
//    {
//        // Your code here
//        if ( e_stop_flag == false && front_obstacle_detected == true) {
//            // Stop the robot if a front obstacle is detected
//            geometry_msgs::Twist zero_velocity;
//            zero_velocity = geometry_msgs::Twist();
//            actionlib_msgs::GoalID empty_goal;
//            cancel_pub.publish(empty_goal);
//            InitSetBrakeStop();
//	    int loop_count = 0;
//            do {
//		cmd_emer_super_pub.publish(zero_velocity);
//		r.sleep();
//           } while (cmd_vel_msg.linear.x > 0.0 || odom_msg.twist.twist.linear.x > 0.0);
//            e_stop_flag = true;
//        }
//        else if (e_stop_flag == true && front_obstacle_detected == false) {
//            // Resume normal operation if no front obstacle is detected
//            e_stop_flag = false;
//        }
//
//        ros::spinOnce();
//        r.sleep();
//    }
//}
//
//
//void e_stop_tim_cb(const ros::TimerEvent& evnet, ros::Timer& timer) {
//    geometry_msgs::Twist zero_velocity;
//    zero_velocity = geometry_msgs::Twist();
//    actionlib_msgs::GoalID empty_goal;
//    if(dt_msg.status_info == 1) cancel_pub.publish(empty_goal);
//
//    if (cmd_vel_msg.linear.x != 0.0 || odom_msg.twist.twist.linear.x != 0.0 ) {
//       cmd_emer_super_pub.publish(zero_velocity);
//     }
//      else {
//        timer.stop();
//     }
//}

void e_stop_timer_callback(const ros::TimerEvent& event, ros::NodeHandle& nh)
{
    if ( e_stop_flag == false && front_obstacle_detected == true) {
        e_stop_flag = true;
        geometry_msgs::Twist zero_velocity;
        zero_velocity = geometry_msgs::Twist();
        actionlib_msgs::GoalID empty_goal;
        if(dt_msg.status_info == 1) cancel_pub.publish(empty_goal);
//        InitSetBrakeStop();
//	e_stop_timer = nh.createTimer(ros::Duration(0.025), boost::bind(&e_stop_tim_cb, _1,  boost::ref(e_stop_timer)));
        // Start a timer to check if cmd_vel_msg.linear.x decreases to zero
        e_stop_timer = nh.createTimer(ros::Duration(0.025), [&](const ros::TimerEvent&){
            if (cmd_vel_msg.linear.x > 0.0 || odom_msg.twist.twist.linear.x > 0.0) {
              cmd_emer_super_pub.publish(zero_velocity);
            }
            else {
        		InitSetBrakeStop();
                e_stop_timer.stop();
            }
        });
    }
    else if (e_stop_flag == true && front_obstacle_detected == false) {
         // Resume normal operation if no front obstacle is detected
          std_srvs::Empty srv;
          clearcostmap_pub.call(srv);
          e_stop_flag = false;
          resume_timer = nh.createTimer(ros::Duration(4.0), [&](const ros::TimerEvent&){
            ROS_INFO("Resume Super timer callback Started!!!");
            std_msgs::Empty resume_msg;
            resume_pub.publish(resume_msg);
            if (resume_feedback_msg.data == -1) {
                ROS_INFO("Resume Super timer callback Stopped!!!, status_info: %d", resume_feedback_msg.data);
                resume_timer.stop();
               }
          });
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "md_node");                                                   //Node name initialization.
    ros::NodeHandle nh;                                                                 //Node handle declaration for communication with ROS system.
    ros::Subscriber vel_sub = nh.subscribe("vel_topic", 100, velCallBack);
    ros::Subscriber release_button_sub = nh.subscribe("/freeway/release", 10, release_cb);
    ros::Subscriber front_obstacle_sub = nh.subscribe("/freeway/front_obstacle_teb", 1, front_obstacle_update_cb);
    ros::Subscriber e_stop__sub = nh.subscribe("/freeway/initsetbrakestop", 10, init_set_brake_stop_cb);
    ros::Subscriber dt_sub = nh.subscribe("/freeway/distancetimecalculator", 10, dt_update_cb);
    ros::Subscriber cmd_sub = nh.subscribe("/cmd_vel", 10, cmd_vel_update_cb);
    ros::Subscriber odom_sub = nh.subscribe("/odom_md", 10, odom_msg_update_cb);
    ros::Subscriber resume_feedback_sub = nh.subscribe("/freeway/resume/feedback", 10, resume_feedback_update_cb);
    cancel_pub = nh.advertise<actionlib_msgs::GoalID>("move_base_flex/move_base/cancel", 10);
    ros::Publisher monitor_pub = nh.advertise<md::monitor_msg>("monitor_topic", 100);   //Publisher declaration.
    ros::Publisher string_pub = nh.advertise<std_msgs::String>("string_com_topic", 100);
    cmd_emer_super_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel/super_emer", 1);
    clearcostmap_pub = nh.serviceClient<std_srvs::Empty>("move_base_flex/clear_costmaps");
    resume_pub = nh.advertise<std_msgs::Empty>("/freeway/resume", 10);

    md::monitor_msg monitor;                                                            //monitor_msg declares message 'message' as message file.

    //std::thread e_stop_thread(e_stop_thread_function);
    // create a timer and pass the node handle to the callback function
    //ros::Timer timer = nh.createTimer(ros::Duration(0.025), e_stop_timer_callback(event, nh);
    ros::Timer timer = nh.createTimer(ros::Duration(0.025), [&](const ros::TimerEvent& event){
        e_stop_timer_callback(event, nh);
    });

    ros::Rate r(10000);                                                                 //Set the loop period -> 100us.

    //variable declaration
    IByte iData;
    static BYTE byCntComStep, byCntCmdVel, fgSendCmdVel, byCntInitStep;
    static BYTE byCnt2500us, byCntCase[10], byFgl, byFglReset, fgInitPosiResetAfter2s, byCntReset;
    static BYTE byCnt, byCntStartDelay, byFglIO;

    int nArray[5];

    Log.fgSet         = ON;
    fgSendCmdVel      = ON;
    byFgl             = OFF;
    Com.fgInitsetting = OFF;

    //Store the value of the parameter in the variable
    nh.getParam("md_node/PC", Com.nIDPC);
    nh.getParam("md_node/MDUI", Com.nIDMDUI);
    nh.getParam("md_node/MDT", Com.nIDMDT);
    nh.getParam("md_node/baudrate", Com.nBaudrate);
    nh.getParam("md_node/diameter", Com.nDiameter);
    nh.getParam("md_node/wheelLength", Com.nWheelLength);
    nh.getParam("md_node/reduction", Com.nGearRatio);
    nh.getParam("md_node/direction", Com.fgDirSign);
    nh.getParam("md_node/halltype", Com.nHallType);
    nh.getParam("md_node/maxrpm", Com.nMaxRPM);
    nh.getParam("md_node/angleresolution", Com.nAngResol);
    nh.getParam("md_node/RMID", Com.nRMID);
    nh.getParam("md_node/slowstart", Com.nSlowstart);
    nh.getParam("md_node/slowdown", Com.nSlowdown);

    InitSerial();     //communication initialization in com.cpp

    while(ros::ok())
    {
        ReceiveDataFromController();

        if(++byCnt2500us == 50)
        {
            byCnt2500us = 0;

            if(Com.fgInitsetting == ON)
            {
                switch(++byCntComStep)
                {
                    case 1:
                        if(++byCntCase[byCntComStep] == TIME_50MS)
                        {
                            byCntCase[byCntComStep] = 0;

                            monitor.lPosiX           = Com.lPosi[_X];
                            monitor.lPosiY           = Com.lPosi[_Y];
                            monitor.sTheta           = Com.sTheta;
                            monitor.sVoltIn          = Com.sVoltIn;
                            monitor.sRealLinearVel   = RB.nRcvLinearVel;
                            monitor.sRealAngularVel  = RB.nRcvAngularVel;
                            monitor.byUS1            = Com.byUS1;
                            monitor.byUS2            = Com.byUS2;
                            monitor.byUS3            = Com.byUS3;
                            monitor.byUS4            = Com.byUS4;
                            monitor.byPlatStatus     = Com.byPlatStatus;
                            monitor.byDocStatus      = Com.byDocStatus;
                            monitor.byLeftMotStatus  = Com.byStatus[MOT_LEFT];
                            monitor.byRightMotStatus = Com.byStatus[MOT_RIGHT];
                            monitor.sLeftMotCur      = Com.sCurrent[MOT_LEFT];
                            monitor.sRightMotCur     = Com.sCurrent[MOT_RIGHT];
                            monitor.lLeftMotPosi     = Com.lMotorPosi[MOT_LEFT];
                            monitor.lRightMotPosi    = Com.lMotorPosi[MOT_RIGHT];
                            monitor.byLeftIOMonitor  = Com.byIOMonitor[MOT_LEFT];
                            monitor.byRightIOMonitor = Com.byIOMonitor[MOT_RIGHT];

                            monitor_pub.publish(monitor);
                        }
                        break;
                    case 2:
                        if(Com.nRMID == Com.nIDMDUI)
                        {
                            if(++byCntCase[byCntComStep] == TIME_50MS)
                            {
                                byFgl ^= 1;
                                byCntCase[byCntComStep] = 0;

                                if(byFgl == 1)
                                {
                                    nArray[0] = PID_ROBOT_MONITOR2;            //PID 224
                                    PutMdData(PID_REQ_PID_DATA, Com.nIDMDUI, nArray);
                                }
                                else
                                {
                                    if(fgInitPosiResetAfter2s == 0)
                                    {
                                        if(byCntReset++ == 20)
                                        {
                                            fgInitPosiResetAfter2s = 1;
                                            ResetPosture();
                                        }
                                    }

                                    if(Com.fgResetOdometry == 1)
                                    {
                                        if(byFglReset==0) ResetPosture();
                                        byFglReset = 1;
                                    }
                                    else byFglReset = 0;
                                }
                            }
                        }
                        else if(Com.nRMID == Com.nIDMDT)
                        {
                            if(++byCntCase[byCntComStep] == TIME_50MS)
                            {
                                byFgl ^= 1;
                                byCntCase[byCntComStep] = 0;

                                if(byFgl == 0)
                                {
                                    if(fgInitPosiResetAfter2s == 0)
                                    {
                                        if(byCntReset++ == 20)
                                        {
                                            fgInitPosiResetAfter2s = 1;
                                            ResetPosture();
                                        }
                                    }

                                    if(Com.fgResetOdometry == 1)
                                    {
                                        if(byFglReset==0) ResetPosture();
                                        byFglReset = 1;
                                    }
                                    else byFglReset = 0;
                                }
                            }
                        }
                        break;
                    case 3:
                        break;
                    case 4:
                        break;
                    case 5:
                        if(++byCntCase[byCntComStep] == TIME_50MS)
                        {
                            byCntCase[byCntComStep] = 0;

                            byFglIO ^= 1;

                            if(byFglIO)
                            {
                                nArray[0] = PID_PNT_IO_MONITOR;                 //PID 241
                                PutMdData(PID_REQ_PID_DATA, Com.nRMID, nArray);
                            }
                            else
                            {
                                if(Com.fgResetAlarm)
                                {
                                    nArray[0] = CMD_ALARM_RESET;
                                    PutMdData(PID_COMMAND, Com.nRMID, nArray);
                                }
                                else
                                {
                                    nArray[0] = PID_PNT_IO_MONITOR;             //PID 241
                                    PutMdData(PID_REQ_PID_DATA, Com.nRMID, nArray);
                                }
                            }
                        }
                        break;
                    case 6:
                        break;
                    case 7:
                        break;
                    case 8:
                        if(++byCntCase[byCntComStep] == TIME_50MS)
                        {
                            byCntCase[byCntComStep] = 0;
                            if(Com.bRccState == OFF)
                            {
                                if(Com.bEmerSW == ON)
                                {
                                    fgSendCmdVel = OFF;
                                    nArray[0] = ENABLE;
                                    nArray[1] = ENABLE;
                                    nArray[2] = REQUEST_PNT_MAIN_DATA;
                                    PutMdData(PID_PNT_TQ_OFF, Com.nRMID, nArray);
                                }
                                else
                                {
                                    if(fgSendCmdVel)
                                    {
                                        RobotCmd2MotCmd(Com.nCmdSpeed, Com.nCmdAngSpeed);

                                        iData = Short2Byte(RB.sRefRPM[0]);
                                        nArray[0] = iData.byLow;
                                        nArray[1] = iData.byHigh;
                                        iData = Short2Byte(RB.sRefRPM[1]);
                                        nArray[2] = iData.byLow;
                                        nArray[3] = iData.byHigh;
                                        nArray[4] = REQUEST_PNT_MAIN_DATA;
                                        PutMdData(PID_PNT_VEL_CMD, Com.nRMID, nArray);
                                    }
                                    else
                                    {
                                        if(byCntCmdVel < 40) byCntCmdVel++;
                                        else if(byCntCmdVel == 40) //after 2s
                                        {
                                            byCntCmdVel  = RESET;
                                            fgSendCmdVel = ON;
                                        }
                                        nArray[0] = PID_PNT_MAIN_DATA;
                                        PutMdData(PID_REQ_PID_DATA, Com.nRMID, nArray);
                                    }
                                }
                            }
                            else if(Com.bRccState == ON)
                            {
                                if(Com.bEmerSW == ON)
                                {
                                    fgSendCmdVel = OFF;
                                    nArray[0] = ENABLE;
                                    nArray[1] = ENABLE;
                                    nArray[2] = REQUEST_PNT_MAIN_DATA;
                                    PutMdData(PID_PNT_TQ_OFF, Com.nRMID, nArray);
                                }
                                else
                                {
                                    if(byCntCmdVel < 40) byCntCmdVel++;
                                    else if(byCntCmdVel == 40) //after 2s
                                    {
                                        byCntCmdVel  = RESET;
                                        fgSendCmdVel = ON;
                                    }
                                    nArray[0] = PID_PNT_MAIN_DATA;
                                    PutMdData(PID_REQ_PID_DATA, Com.nRMID, nArray);
                                }
                            }
                        }
                        break;
                    case 9:
                        break;
                    case 10:
                        byCntComStep = 0;
                        break;
                }
            }
            else
            {
                if(byCntStartDelay <= 200) byCntStartDelay++;
                else
                {
                    switch(++byCntInitStep)
                    {
                        case 1:
                            nArray[0] = 0;
                            nArray[1] = 0;
                            nArray[2] = 0;
                            nArray[3] = 0;
                            nArray[4] = REQUEST_PNT_MAIN_DATA;
                            PutMdData(PID_PNT_VEL_CMD, Com.nRMID, nArray);
                            break;
                        case 4:
                            if(Com.nRMID == Com.nIDMDUI) InitSetParam();   //posture initialization in com.cpp
                            InitRobotParam();
                            break;
                        case 7:
                            ResetPosture();                                //reset about odometry's variable
                            break;
                        case 10:
                            InitSetSlowStart();
                            break;
                        case 13:
                            InitSetSlowDown();
                            break;
                        case 15:
                            if(Com.nRMID == MID_MDUI) InitRobotMotDir();
                            Com.fgInitsetting = ON;
                            break;
                    }
                }
            }
        }
    //    if ( e_stop_flag == false && front_obstacle_detected == true) {
        // Stop the robot if a front obstacle is detected
  //      geometry_msgs::Twist zero_velocity;
//	zero_velocity = geometry_msgs::Twist();
//	actionlib_msgs::GoalID empty_goal;
//	cancel_pub.publish(empty_goal);
  //      InitSetBrakeStop();
//	do { cmd_emer_super_pub.publish(zero_velocity);
       // } while (cmd_vel_msg.linear.x == 0.0);
       //e_stop_flag = true;
       // } 
       // else if (e_stop_flag == true && front_obstacle_detected == false) {
          // Resume normal operation if no front obstacle is detected
       //   e_stop_flag = false;
       // }
        // if(SET_STOP_STATUS) {
        //     if( pre_release_flag != release_flag) once_flag = true;
        //     if (release_flag && once_flag) { //release button pushed
        //         once_flag = false;
        //         InitUnSetStopStatus();
        //     }
        //     else if (!release_flag && once_flag) {
        //         once_flag = false;
        //         InitSetStopStatus();
        //     }
        //     pre_release_flag = release_flag;
        // }

        ros::spinOnce();
        r.sleep();
    } 
}
