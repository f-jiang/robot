#include <cstdint>
#include <exception>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <serial/serial.h>

#include <robot_constants/nodes.h>
#include <robot_constants/topics.h>

const uint32_t kQueueSize = 1000;
const uint32_t kSerialTimeout = 500;

float left_wheel_setpoint_ang_vel = 0;
float prev_left_wheel_setpoint_ang_vel = 0;
float right_wheel_setpoint_ang_vel = 0;
float prev_right_wheel_setpoint_ang_vel = 0;
bool left_wheel_setpoint_ang_vel_recv = false;
bool right_wheel_setpoint_ang_vel_recv = false;

void LeftWheelSetpointAngVelCallback(const std_msgs::Float32& msg) {
    prev_left_wheel_setpoint_ang_vel = left_wheel_setpoint_ang_vel;
    left_wheel_setpoint_ang_vel = msg.data;
    left_wheel_setpoint_ang_vel_recv = true;
    ROS_DEBUG_STREAM_NAMED(robot_constants::nodes::kSerialNode,
                           "received left setpoint of " << msg.data);
}

void RightWheelSetpointAngVelCallback(const std_msgs::Float32& msg) {
    prev_right_wheel_setpoint_ang_vel = right_wheel_setpoint_ang_vel;
    right_wheel_setpoint_ang_vel = msg.data;
    right_wheel_setpoint_ang_vel_recv = true;
    ROS_DEBUG_STREAM_NAMED(robot_constants::nodes::kSerialNode,
                           "received right setpoint of " << msg.data);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, robot_constants::nodes::kSerialNode);
    ros::NodeHandle nh;
    ROS_INFO_STREAM_NAMED(robot_constants::nodes::kSerialNode, "started node");

    ros::Subscriber left_wheel_setpoint_ang_vel_sub = nh.subscribe(
            robot_constants::topics::kLeftWheelSetpointAngVel,
            kQueueSize,
            LeftWheelSetpointAngVelCallback);
    ros::Subscriber right_wheel_setpoint_ang_vel_sub = nh.subscribe(
            robot_constants::topics::kRightWheelSetpointAngVel,
            kQueueSize,
            RightWheelSetpointAngVelCallback);

    // TODO consider custom msg for below
    ros::Publisher left_wheel_cur_ang_pos_pub = nh.advertise<std_msgs::Float32>(
            robot_constants::topics::kLeftWheelCurAngPos,
            kQueueSize);
    ros::Publisher right_wheel_cur_ang_pos_pub = nh.advertise<std_msgs::Float32>(
            robot_constants::topics::kRightWheelCurAngPos,
            kQueueSize);
    ros::Publisher left_wheel_cur_ang_vel_pub = nh.advertise<std_msgs::Float32>(
            robot_constants::topics::kLeftWheelCurAngVel,
            kQueueSize);
    ros::Publisher right_wheel_cur_ang_vel_pub = nh.advertise<std_msgs::Float32>(
            robot_constants::topics::kRightWheelCurAngVel,
            kQueueSize);

    std::string port;
    int baud;
    nh.getParam("/serial_port", port);
    nh.getParam("/serial_baud_rate", baud);
    serial::Serial serial_port(port, baud, serial::Timeout::simpleTimeout(kSerialTimeout));
    ROS_INFO_STREAM_NAMED(robot_constants::nodes::kSerialNode,
                          "initialized serial port on " << port << " with baud " << baud);

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        // when both setpoints recvd, write over serial
        // ready condition assumes that will always receive two setpoints (l and r) each time
        if (left_wheel_setpoint_ang_vel_recv && right_wheel_setpoint_ang_vel_recv &&
            (left_wheel_setpoint_ang_vel != prev_left_wheel_setpoint_ang_vel ||
             right_wheel_setpoint_ang_vel != prev_right_wheel_setpoint_ang_vel)) {
            left_wheel_setpoint_ang_vel_recv = false;
            right_wheel_setpoint_ang_vel_recv = false;

            ROS_DEBUG_STREAM_NAMED(robot_constants::nodes::kSerialNode,
                                   "writing setpoints to " << port);
            std::stringstream ss;
            ss << left_wheel_setpoint_ang_vel << ',' << right_wheel_setpoint_ang_vel << ';';
            size_t bytes_written = serial_port.write(ss.str());
            ROS_DEBUG_STREAM_NAMED(robot_constants::nodes::kSerialNode,
                                   "serial write (" << bytes_written << " bytes): " << ss.str());
        }

        // read from serial, and publish ang. pos and ang. vel
        try {
            std::string buf;
            size_t bytes_read = serial_port.readline(buf);

            if (!buf.empty()) {
                ROS_DEBUG_STREAM_NAMED(robot_constants::nodes::kSerialNode,
                                       "serial read (" << bytes_read << " bytes): " << buf);
                size_t buf_idx;
                float left_wheel_cur_ang_vel = std::stof(buf, &buf_idx);
                buf.erase(0, buf_idx + 1);
                float right_wheel_cur_ang_vel = std::stof(buf, &buf_idx);
                buf.erase(0, buf_idx + 1);
                float left_wheel_cur_ang_pos = std::stof(buf, &buf_idx);
                buf.erase(0, buf_idx + 1);
                float right_wheel_cur_ang_pos = std::stof(buf);
                std_msgs::Float32 vel_msg;
                std_msgs::Float32 pos_msg;

                vel_msg.data = left_wheel_cur_ang_vel;
                left_wheel_cur_ang_vel_pub.publish(vel_msg);

                vel_msg.data = right_wheel_cur_ang_vel;
                right_wheel_cur_ang_vel_pub.publish(vel_msg);

                pos_msg.data = left_wheel_cur_ang_pos;
                left_wheel_cur_ang_pos_pub.publish(pos_msg);

                pos_msg.data = right_wheel_cur_ang_pos;
                right_wheel_cur_ang_pos_pub.publish(pos_msg);
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM_NAMED(robot_constants::nodes::kSerialNode,
                                   "serial read failed: " << e.what());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

