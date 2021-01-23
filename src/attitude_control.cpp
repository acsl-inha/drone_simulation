#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Eigen>
#include <cmath>
#define Pi 3.14159265359

double m = 1.535;
double g = 9.81;
double max_thrust = 41.3751272;
double state[15];
double Deg2Rad = Pi /180;
double Rad2Deg = 1/Deg2Rad;

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void Callback_pose(const geometry_msgs::PoseStamped msg_pose)
{
    // ROS_INFO("Got position data: %f, %f, %f", msg_pose.pose.position.x, msg_pose.pose.position.y, msg_pose.pose.position.z);
    tf::Quaternion q( msg_pose.pose.orientation.x, msg_pose.pose.orientation.y, msg_pose.pose.orientation.z, msg_pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    state[6] = msg_pose.pose.position.x;
    state[7] = -msg_pose.pose.position.y;
    state[8] = -msg_pose.pose.position.z;
    state[9] = roll; 
    state[10] = pitch;
    state[11] = yaw;
}

void Callback_velocity(const geometry_msgs::TwistStamped msg_velocity)
{
    // ROS_INFO("Got data velocity: %f, %f, %f", msg_velocity.twist.angular.x,  msg_velocity.twist.angular.y,  msg_velocity.twist.angular.z);
    double x_dot = msg_velocity.twist.linear.x;
    double y_dot = -msg_velocity.twist.linear.y;
    double z_dot = -msg_velocity.twist.linear.z;
    double phi = state[9]; 
    double the = state[10];
    double psi = state[11];
    
    Eigen::Vector3d temp(x_dot,y_dot,z_dot);
    Eigen::Matrix3d Cbn;
    Cbn<<   std::cos(psi)*std::cos(the), std::cos(psi)*std::sin(the)*std::sin(phi) - std::sin(psi)*std::cos(phi), std::cos(psi)*std::sin(the)*std::cos(phi) + std::sin(psi)*std::sin(phi),
            std::sin(psi)*std::cos(the), std::sin(psi)*std::sin(the)*std::sin(phi) + std::cos(psi)*std::cos(phi), std::sin(psi)*std::sin(the)*std::cos(phi) - std::cos(psi)*std::sin(phi),
                    -std::sin(the), std::cos(the)*std::sin(phi), std::cos(the)*std::cos(phi);
    Eigen::Matrix3d Cbn_transpose = Cbn.transpose();
    Eigen::Vector3d v = Cbn_transpose*temp;
    state[0] = v[0]; 
    state[1] = v[1];
    state[2] = v[2];
    state[3] = msg_velocity.twist.angular.x;
    state[4] = msg_velocity.twist.angular.y;
    state[5] = msg_velocity.twist.angular.z;
    state[12] = x_dot;
    state[13] = y_dot;
    state[14] = z_dot;

    // for(int i=0;i<12;i++)
    // {

    //     std::cout << state[i] << ',';
    //     if(i==11)
    //     {
    //         std::cout << std::endl;
    //     }

    // }
}

void compute_guidance(double state[],double (&cmd)[4],double des_position[], double des_velocity[])
{   
    double u = state[0];
    double v = state[1];
    double w = state[2];
    double p = state[3];
    double q = state[4];
    double r = state[5];
    double x = state[6];
    double y = state[7];
    double z = state[8];
    double phi = state[9];
    double the = state[10];
    double psi = state[11];
    double x_dot = state[12];
    double y_dot = state[13];
    double z_dot = state[14];
    double h_dot = -z_dot;
    double h = -z;
    
    Eigen::Matrix3d Cbn;
    Cbn<<   std::cos(psi)*std::cos(the), std::cos(psi)*std::sin(the)*std::sin(phi) - std::sin(psi)*std::cos(phi), std::cos(psi)*std::sin(the)*std::cos(phi) + std::sin(psi)*std::sin(phi),
            std::sin(psi)*std::cos(the), std::sin(psi)*std::sin(the)*std::sin(phi) + std::cos(psi)*std::cos(phi), std::sin(psi)*std::sin(the)*std::cos(phi) - std::cos(psi)*std::sin(phi),
                    -std::sin(the), std::cos(the)*std::sin(phi), std::cos(the)*std::cos(phi);
    Eigen::Matrix3d Cbn_transpose = Cbn.transpose();
    
    double x_cmd = des_position[0];
    double y_cmd = -des_position[1];
    double h_cmd = des_position[2];
    double psi_cmd = 10*Deg2Rad;
    
    double omg_alt = 2.5;
    double zet_alt = 0.8;
    double omg_pos = 2.5; 
    double zet_pos = 0.8;

    // altitude loop
    double up_cmd = -2*zet_alt*omg_alt*h_dot + omg_alt*omg_alt*(h_cmd-h) + g;
    double thrust = m*up_cmd/(std::cos(phi)*std::cos(the));

    // horizontal position loop
    double ax_cmd = -2*zet_pos*omg_pos*x_dot + omg_pos*omg_pos*(x_cmd-x);
    double ay_cmd = -2*zet_pos*omg_pos*y_dot + omg_pos*omg_pos*(y_cmd-y);
    
    Eigen::Vector3d temp(ax_cmd,ay_cmd,0);
    Eigen::Vector3d temp2 = Cbn_transpose*temp;

    double au_cmd = temp2[0];
    double av_cmd = temp2[1];
    double aw_cmd = temp2[2];
    
    double the_cmd = std::atan2(-au_cmd,g);
    double phi_cmd = std::atan2( av_cmd,g);

    cmd[0] = thrust;
    cmd[1] = phi_cmd;
    cmd[2] = the_cmd;
    cmd[3] = psi_cmd;  

    for(int i=0;i<4;i++)
    {

        std::cout << cmd[i] << ',';
        if(i==3)
        {
            std::cout << std::endl;
        }
    }
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "attitude_control");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, Callback_pose);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_body", 10, Callback_velocity);


    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    ros::Publisher local_position_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude",10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    double SampleFreq = 50;
    double SampleTime = 0.01;
    ros::Rate rate(SampleFreq);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 10;
    int count = 1;

    // geometry_msgs::PoseStamped cmd_att;
    std_msgs::Float64 cmd_thr;

    mavros_msgs::AttitudeTarget cmd_att;
    double roll, pitch, yaw;
    double theta = 0.0;
    double lambda = 0.56;
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

        double cmd[4];
        double des_pos[3]; 
        double des_vel[3];

        des_pos[0] = 0; des_pos[1] = 0; des_pos[2] = 1;
        des_vel[0] = 0; des_vel[1] = 0; des_vel[2] = 0; 
        compute_guidance(state, cmd, des_pos, des_vel);
        
        tf2::Quaternion quaternion;
        quaternion.setRPY( cmd[1], cmd[2], cmd[3] );

        cmd_att.header.stamp = ros::Time::now();
        cmd_att.header.seq = count;
        cmd_att.header.frame_id = 1;
        cmd_att.orientation.x = quaternion.x();
        cmd_att.orientation.y = quaternion.y();
        cmd_att.orientation.z = quaternion.z();
        cmd_att.orientation.w = quaternion.w();
        cmd_att.thrust = cmd[0]/max_thrust;
        count++;
        thrust_pub.publish(cmd_att);

        // pose.pose.position.x = 2;
        // pose.pose.position.y = 1;
        // pose.pose.position.z = 1;
        // local_position_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}


// void ADMM_compute_guidance(double state[],double *q,double des_position[], double des_velocity[])
// {
//     // double x_dot,y_dot,z_dot = state[12],state[13],state[14];
//     Eigen::Vector3d r_0(state[6], state[7], state[8]);
//     Eigen::Vector3d v_0(state[12], state[13], state[14]);
// }