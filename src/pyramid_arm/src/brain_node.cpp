#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <string>
#include "pyramid_arm/kinematics.hpp"
#include "pyramid_arm/slots.hpp"

const double Z_grasp = 0.03;
const double Z_trans = 0.24;
const double grip_open = 0.0;
const double grip_close = 1.0;
const Eigen::Vector4d default_pose(0.0, -1.57, 0.0, 0.0);
enum state {IDLE, DETECT, SELECT, APPROACH, GRASP, PLACE, RETREAT, NEXT_SLOT, DONE};

Eigen::Vector4d q_cmd = Eigen::Vector4d::Zero();
double grip = grip_open;
geometry_msgs::Point block;
ros::Time block_time;
ros::Publisher cmd_pub, color_pub;

// BLOCK POS
void block_cb(const geometry_msgs::Point::ConstPtr& msg){
    block = *msg;
    block_time = ros::Time::now();
}

// ARRAY OF COMMANDS
void Cmd(const Eigen::Vector4d& q){
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(q[0]);
    msg.data.push_back(q[1]);
    msg.data.push_back(q[2]);
    msg.data.push_back(q[3]);
    msg.data.push_back(grip);
    cmd_pub.publish(msg);
}

// CMD Q TO MOVE
void Move(const Eigen::Vector4d& goal){
    Eigen::Vector4d start = q_cmd;
    double biggest = (goal - start).cwiseAbs().maxCoeff();
    int steps = (int)(biggest/0.02) + 1;

    ros::Rate rate(50);
    for(int i = 0; i <= steps; i++){
        q_cmd = start + (goal - start) * ((double) i / steps);
        Cmd(q_cmd);
        ros::spinOnce();
        rate.sleep();
    }
    ros::Duration(0.3).sleep();
}

// CMD GRIPPER TO MOVE
void setGrip(double value){
    grip = value;
    for(int i = 0; i < 10; i++){
        Cmd(q_cmd);
        ros::Duration(0.1).sleep();
    }
}

// PUBLISH COLOR TO VISION
bool detect(const std::string& color){
    std_msgs::String msg;
    msg.data = color;
    color_pub.publish(msg);

    ros::Time start = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - start).toSec() < 7.0){
        ros::spinOnce();
        if(block_time > start + ros::Duration(0.5)) return true;
        ros::Duration(0.02).sleep();
    }
    return false;
}

// CALCULATE EVERY STEP
bool Plan(double bx, double by, const Slot& s, Eigen::Vector4d Q[6]){
    Eigen::Vector3d P[6];
    P[0] = Eigen::Vector3d(bx, by, Z_grasp * 2 + 0.01);
    P[1] = Eigen::Vector3d(bx, by, Z_grasp);
    P[2] = Eigen::Vector3d(bx, by, Z_trans);
    P[3] = Eigen::Vector3d(s.x, s.y, Z_trans);
    P[4] = Eigen::Vector3d(s.x, s.y, s.z + 0.003);
    P[5] = Eigen::Vector3d(s.x, s.y, s.z + 0.1);

    Eigen::Vector4d q = seed(P[0]);
    for(int i = 0; i < 6; i++){
        if(!IK(dhMat(), P[i], q)){
            ROS_WARN("IK GAGAL");
            return false;
        }
        Q[i] = q;
    }
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "brain_node");
    ros::NodeHandle nh;

    cmd_pub = nh.advertise<std_msgs::Float64MultiArray>("/joint_cmd", 1);
    color_pub = nh.advertise<std_msgs::String>("/target_color", 1, true);
    ros::Subscriber block_sub = nh.subscribe("/block_pos", 1, block_cb);
    ros::Duration(1.0).sleep();

    state state = IDLE;
    int slot = 0;
    int total = (int)slots().size();
    Eigen::Vector4d Q[6];
    
    // MAIN LOOP
    while(ros::ok() && state != DONE){
        ros::spinOnce();
        switch (state){
        case IDLE:
            grip = grip_close;
            Move(default_pose);
            state = DETECT;
            break;

        case DETECT:
            if(slot >= total){
                state = DONE;
                break;
            }
            ROS_INFO("Slot ke-%d: cari balok %s", slot+1, slots()[slot].color.c_str());
            if(detect(slots()[slot].color)){
                state = SELECT;
            } else{
                ROS_WARN("Balok tidak ditemukan");
                state = NEXT_SLOT;
            }
            break;

        case SELECT:
            ROS_INFO("Balok di  x=%.3f y=%.3f  r=%.3f", block.x, block.y, std::hypot(block.x, block.y));;
            if(Plan(block.x, block.y, slots()[slot], Q)){
                Move(Q[0]);
                state = APPROACH;
            } else{
                ROS_WARN("Tidak terjangkau");
                state = NEXT_SLOT;
            }
            break;
        
        case APPROACH:
            setGrip(grip_open);
            Move(Q[1]);
            state = GRASP;
            break;
        
        case GRASP:
            setGrip(grip_close);
            Move(Q[2]);
            state = PLACE;
            break;

        case PLACE:
            Move(Q[3]);
            Move(Q[4]);
            setGrip(grip_open);
            state = RETREAT;
            break;

        case RETREAT:
            Move(Q[5]);
            Move(default_pose);
            setGrip(grip_close);
            state = NEXT_SLOT;
            break;

        case NEXT_SLOT:
            slot++;
            state = DETECT;
            break;

        default:
            break;
        }
    }

    ROS_INFO("Selesai");
    return 0;
}