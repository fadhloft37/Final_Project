#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <string>
#include <vector>

mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
mjvCamera cam_top;
mjvOption opt_cam;
mjvScene scn_cam;

double g_target[6] = {0,0,0,0,0,0};
double g_j5_open = -0.5, g_j5_close = 0.0;
double g_j6_open = 0.5, g_j6_close = 0.0;

void jointCmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    if(msg->data.size() < 5){
        ROS_WARN("/joint_cmd butuh 5 angka: [q1 q2 q3 q4 grip]");
        return;
    }
    for(int i = 0; i < 4; i++) g_target[i] = msg->data[i];

    double g = std::min(1.0, std::max(0.0, msg->data[4]));
    g_target[4] = g_j5_open + g * (g_j5_close - g_j5_open);
    g_target[5] = g_j6_open + g * (g_j6_close - g_j6_open);
}

bool button_left = false, button_middle = false, button_right = false;
double lastx = 0, lasty = 0;

void mouseButton(GLFWwindow* w, int, int, int){
    button_left = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    button_middle = glfwGetMouseButton(w,GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    button_right = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(w, &lastx, &lasty);
}

void mouseMove(GLFWwindow* w, double xpos, double ypos)
{
    if (!button_left && !button_middle && !button_right) return;
 
    double dx = xpos - lastx, dy = ypos - lasty;
    lastx = xpos; lasty = ypos;
 
    int width, height;
    glfwGetWindowSize(w, &width, &height);
    bool shift = glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
 
    mjtMouse action;
    if (button_right)      action = shift ? mjMOUSE_MOVE_H   : mjMOUSE_MOVE_V;
    else if (button_left)  action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else                   action = mjMOUSE_ZOOM;
 
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void scrollCb(GLFWwindow*, double, double yoffset)
{
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}
 
void keyCb(GLFWwindow* w, int key, int, int act, int)
{
    if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(w, 1);
}

void publishCamera(ros::Publisher& pub)
{
    int W = con.offWidth, H = con.offHeight;
    static std::vector<unsigned char> rgb;
    rgb.resize(3 * W * H);
 
    mjr_setBuffer(mjFB_OFFSCREEN, &con);
    mjrRect vp = {0, 0, W, H};
    mjv_updateScene(m, d, &opt_cam, nullptr, &cam_top, mjCAT_ALL, &scn_cam);
    mjr_render(vp, &scn_cam, &con);
    mjr_readPixels(rgb.data(), nullptr, vp, &con);
    mjr_setBuffer(mjFB_WINDOW, &con);             
    cv::Mat img(H, W, CV_8UC3, rgb.data());      
    cv::Mat bgr;
    cv::cvtColor(img, bgr, cv::COLOR_RGB2BGR);
    cv::flip(bgr, bgr, 0);
 
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "camera_top";
    pub.publish(cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sim_node");
    ros::NodeHandle nh, pnh("~");
 
    std::string model_path;
    double cam_hz = 10.0;
    pnh.param<std::string>("model", model_path, ros::package::getPath("pyramid_arm") + "/mjcf/scene.xml");
    pnh.param("camera_hz", cam_hz, 10.0);
    pnh.param("j5_open",  g_j5_open,  -0.4);
    pnh.param("j5_close", g_j5_close, 0.0);
    pnh.param("j6_open",  g_j6_open,  0.4);
    pnh.param("j6_close", g_j6_close, 0.0);
    g_target[4] = g_j5_open;
    g_target[5] = g_j6_open;
 
    char err[1000] = "";
    m = mj_loadXML(model_path.c_str(), nullptr, err, sizeof(err));
    if (!m) {
        ROS_FATAL("Gagal load %s : %s", model_path.c_str(), err);
        return 1;
    }
    d = mj_makeData(m);
 
    const char* act_names[6] = {"act_J1", "act_J2", "act_J3", "act_J4", "act_J5", "act_J6"};
    const char* sen_names[6] = {"jp1", "jp2", "jp3", "jp4", "jp5", "jp6"};
    int act_id[6], sen_adr[6];
    for (int i = 0; i < 6; i++) {
        act_id[i] = mj_name2id(m, mjOBJ_ACTUATOR, act_names[i]);
        int sid   = mj_name2id(m, mjOBJ_SENSOR, sen_names[i]);
        if (act_id[i] < 0 || sid < 0) {
            ROS_FATAL("Actuator %s atau sensor %s tidak ketemu di model", act_names[i], sen_names[i]);
            return 1;
        }
        sen_adr[i] = m->sensor_adr[sid];
    }
    int cam_id = mj_name2id(m, mjOBJ_CAMERA, "top");
    if (cam_id < 0) {
        ROS_FATAL("Kamera \"top\" tidak ada di scene.xml");
        return 1;
    }
 
    if (!glfwInit()) {
        ROS_FATAL("glfwInit gagal");
        return 1;
    }
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Piramida Sim", nullptr, nullptr);
    if (!window) {
        ROS_FATAL("Gagal membuat jendela GLFW");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
 
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjv_makeScene(m, &scn_cam, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);
 
    cam.azimuth = 135; cam.elevation = -25; cam.distance = 1.2;
    cam.lookat[0] = 0.15; cam.lookat[1] = 0.0; cam.lookat[2] = 0.1;
    opt.geomgroup[3] = 1;               
    
    mjv_defaultCamera(&cam_top);
    cam_top.type = mjCAMERA_FIXED;
    cam_top.fixedcamid = cam_id;
    mjv_defaultOption(&opt_cam);
    for (int i = 0; i < mjNGROUP; i++) opt_cam.sitegroup[i] = 0;
    opt_cam.geomgroup[3] = 0;
 
    glfwSetMouseButtonCallback(window, mouseButton);
    glfwSetCursorPosCallback(window, mouseMove);
    glfwSetScrollCallback(window, scrollCb);
    glfwSetKeyCallback(window, keyCb);
 
    ros::Subscriber sub = nh.subscribe("/joint_cmd", 10, jointCmdCallback);
    ros::Publisher pub_img   = nh.advertise<sensor_msgs::Image>("/camera/image_raw", 1);
 
    ROS_INFO("sim_node jalan. Model: %s", model_path.c_str());
    ROS_INFO("Kirim perintah: rostopic pub -1 /joint_cmd std_msgs/Float64MultiArray \"data: [0, -1.0, 1.0, 0.0, 0.0]\"");
 
    ros::Rate loop(60);
    ros::Time last_img = ros::Time::now();
 
    while (ros::ok() && !glfwWindowShouldClose(window)) {
        ros::spinOnce();                             
        for (int i = 0; i < 6; i++) d->ctrl[act_id[i]] = g_target[i];
 
        mjtNum t0 = d->time;
        while (d->time - t0 < 1.0 / 60.0) mj_step(m, d);
 
        if ((ros::Time::now() - last_img).toSec() >= 1.0 / cam_hz) {
            publishCamera(pub_img);
            last_img = ros::Time::now();
        }
 
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
 
        loop.sleep();
    }
 
    mjv_freeScene(&scn);
    mjv_freeScene(&scn_cam);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}
