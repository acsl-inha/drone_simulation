#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#define Pi 3.14159265359

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "attitude_control");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher rotor_rpm_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10); 
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    double SampleFreq = 100;
    double SampleTime = 0.02;
    ros::Rate rate(SampleFreq);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    int count = 1;

    mavros_msgs::OverrideRCIn act_control_msg;
    // geometry_msgs::PoseStamped cmd_att;
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
        else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // act_control_msg.header.stamp = ros::Time::now();
        // act_control_msg.header.seq = count;
        // act_control_msg.header.frame_id = ' ';
        // act_control_msg.group_mix = 3;
        act_control_msg.channels[0] = 1600;
        act_control_msg.channels[1] = 1600;
        act_control_msg.channels[2] = 1600;
        act_control_msg.channels[3] = 1600;
        act_control_msg.channels[4] = 1500;
        act_control_msg.channels[5] = 1500;
        act_control_msg.channels[6] = 0;
        act_control_msg.channels[7] = 0;
        
        rotor_rpm_pub.publish(act_control_msg);
        count++;

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}