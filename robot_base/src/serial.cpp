#include <string>
#include <sstream>
#include <cstdint>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <serial/serial.h>

#include <robot_constants/nodes.h>
#include <robot_constants/topics.h>

float left_wheel_set_ang_vel = 0;
float prev_left_wheel_set_ang_vel = 0;
float right_wheel_set_ang_vel = 0;
float prev_right_wheel_set_ang_vel = 0;
bool left_wheel_set_ang_vel_recv = false;
bool right_wheel_set_ang_vel_recv = false;

void LeftWheelAngVelCallback(const std_msgs::Float32& msg) {
    prev_left_wheel_set_ang_vel = left_wheel_set_ang_vel;
    left_wheel_set_ang_vel = msg.data;
    left_wheel_set_ang_vel_recv = true;
}

void RightWheelAngVelCallback(const std_msgs::Float32& msg) {
    prev_right_wheel_set_ang_vel = right_wheel_set_ang_vel;
    right_wheel_set_ang_vel = msg.data;
    right_wheel_set_ang_vel_recv = true;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, robot_constants::nodes::kSerialNode);
    ros::NodeHandle nh;
    ROS_INFO_STREAM_NAMED(robot_constants::nodes::kSerialNode, "started node ");

    ros::Subscriber left_wheel_set_ang_vel_sub = nh.subscribe(
            robot_constants::topics::kLeftWheelSetAngVel,
            1000,
            LeftWheelAngVelCallback);
    ros::Subscriber right_wheel_set_ang_vel_sub = nh.subscribe(
            robot_constants::topics::kRightWheelSetAngVel,
            1000,
            RightWheelAngVelCallback);

    // TODO consider custom msg for below
    ros::Publisher left_wheel_cur_pos_pub = nh.advertise<std_msgs::Int32>(
            robot_constants::topics::kLeftWheelCurPos,
            1000);
    ros::Publisher right_wheel_cur_pos_pub = nh.advertise<std_msgs::Int32>(
            robot_constants::topics::kRightWheelCurPos,
            1000);
    ros::Publisher left_wheel_cur_ang_vel_pub = nh.advertise<std_msgs::Float32>(
            robot_constants::topics::kLeftWheelCurAngVel,
            1000);
    ros::Publisher right_wheel_cur_ang_vel_pub = nh.advertise<std_msgs::Float32>(
            robot_constants::topics::kRightWheelCurAngVel,
            1000);

    std::string port;
    int baud;
    nh.getParam("/serial_port", port);
    nh.getParam("/serial_baud_rate", baud);
    serial::Serial serial_port(port, baud);
    ROS_INFO_STREAM_NAMED(robot_constants::nodes::kSerialNode,
                          "initialized serial port on " << port << " with baud " << baud);

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        // when both ang_vels recvd, write over serial
        if (left_wheel_set_ang_vel_recv && right_wheel_set_ang_vel_recv &&
            (left_wheel_set_ang_vel != prev_left_wheel_set_ang_vel ||
             right_wheel_set_ang_vel != prev_right_wheel_set_ang_vel)) {
            left_wheel_set_ang_vel_recv = false;
            right_wheel_set_ang_vel_recv = false;

            std::stringstream ss;
            ss << left_wheel_set_ang_vel << ',' << right_wheel_set_ang_vel << ';';
            serial_port.write(ss.str());
        }

        // read from serial, and publish pos and ang_vel
        std::string buf = serial_port.readline();
        size_t buf_idx;
        long left_wheel_cur_pos = std::stoi(buf, &buf_idx);
        long right_wheel_cur_pos = std::stoi(buf.substr(buf_idx + 1), &buf_idx);
        float left_wheel_cur_ang_vel = std::stof(buf.substr(buf_idx + 1), &buf_idx);
        float right_wheel_cur_ang_vel = std::stof(buf.substr(buf_idx + 1), &buf_idx);
        std_msgs::Int32 pos_msg;
        std_msgs::Float32 vel_msg;
        pos_msg.data = left_wheel_cur_pos;
        left_wheel_cur_pos_pub.publish(pos_msg);
        pos_msg.data = right_wheel_cur_pos;
        right_wheel_cur_pos_pub.publish(pos_msg);
        vel_msg.data = left_wheel_cur_ang_vel;
        left_wheel_cur_ang_vel_pub.publish(vel_msg);
        vel_msg.data = right_wheel_cur_ang_vel;
        right_wheel_cur_ang_vel_pub.publish(vel_msg);

        loop_rate.sleep();
    }

    return 0;
}

