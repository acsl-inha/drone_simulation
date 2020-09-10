#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher send_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    
    geometry_msgs::TwistStamped send_velocity_msg;
    double ros_roll =0.0;
    double ros_pitch =0.0;
    double ros_yaw =0.0;
    double ros_throttle =0.0;
    int count = 1;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
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

        nh.param <double>("ros_roll", ros_roll , 0.0);
        nh.param <double>("ros_pitch", ros_pitch , 0.0);
        nh.param <double>("ros_yaw", ros_yaw , 0.0);
        nh.param <double>("ros_throttle", ros_throttle ,5.0);

        send_velocity_msg.header.stamp = ros::Time::now();
        send_velocity_msg.header.seq = count;
        send_velocity_msg.header.frame_id = 1;

        send_velocity_msg.twist.linear.x = ros_throttle;
        send_velocity_msg.twist.linear.y = 0;
        send_velocity_msg.twist.linear.z = 0.5;
        send_velocity_msg.twist.angular.x = ros_pitch;
        send_velocity_msg.twist.angular.y = ros_roll;
        send_velocity_msg.twist.angular.z = ros_yaw;

        count++;
        send_velocity_pub.publish(send_velocity_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

