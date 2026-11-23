
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

const char*  PORT = "/dev/ttyACM0";
const int    channel[4] = { 0,     1,     2,     3   };  
const double sign_[4]   = {-1.0,  +1.0,  -1.0,  -1.0 };  
const double center[4]  = { 90.0,  180.0,  135.0,  135.0 };  
const bool   ready[4]   = { true,  true, true, true }; 

const int    CLAW_CHANNEL = 4;
const double CLAW_OPEN    = 135.0;
const double CLAW_CLOSED  = 180.0;
const bool CLAW_ENABLED = true;

int  serial_fd = -1;
bool send_enabled = false;
int  last_sent[5] = {-1, -1, -1, -1, -1}; 

void sendServo(int ch, int deg) {
    if (deg == last_sent[ch]) return;
    last_sent[ch] = deg;

    if (send_enabled) {
        unsigned char buf[2] = {(unsigned char)ch, (unsigned char)deg};
        if (write(serial_fd, buf, 2) != 2) ROS_WARN_THROTTLE(1.0, "Gagal menulis ke serial");
    } else {
        ROS_INFO_THROTTLE(0.5, "[DRY-RUN] channel %d -> %d derajat", ch, deg);
    }
}

void onCmd(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 5) return;

    for (int i = 0; i < 4; i++) {
        if (!ready[i]) continue;
        double deg = center[i] + sign_[i] * msg->data[i] * 180.0 / M_PI;
        if (deg < 0.0)   deg = 0.0;       
        if (deg > 180.0) deg = 180.0;
        sendServo(channel[i], (int)std::lround(deg));
    }

    if (CLAW_ENABLED) {
        double g = msg->data[4];
        if (g < 0.0) g = 0.0;
        if (g > 1.0) g = 1.0;
        sendServo(CLAW_CHANNEL, (int)std::lround(CLAW_OPEN + g * (CLAW_CLOSED - CLAW_OPEN)));
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bridge_node");
    ros::NodeHandle nh;
    ros::param::param<bool>("~send", send_enabled, false);

    if (send_enabled) {
        serial_fd = open(PORT, O_RDWR | O_NOCTTY);
        if (serial_fd < 0) {
            ROS_FATAL("Tidak bisa membuka %s (cek kabel USB dan izin grup dialout)", PORT);
            return 1;
        }
        struct termios tty;
        tcgetattr(serial_fd, &tty);
        cfmakeraw(&tty);
        tcsetattr(serial_fd, TCSANOW, &tty);
        ROS_WARN("MODE KIRIM: perintah dikirim ke %s", PORT);
    } else {
        ROS_WARN("MODE DRY-RUN: tidak ada yang dikirim (pakai _send:=true untuk mengirim)");
    }
    for (int i = 0; i < 4; i++)
        if (!ready[i]) ROS_WARN("Joint_%d belum dikalibrasi, tidak dikirim", i + 1);

    ros::Subscriber sub = nh.subscribe("/joint_cmd", 10, onCmd);
    ros::spin();

    if (serial_fd >= 0) close(serial_fd);
    return 0;
}
