#include <algorithm>
#include <functional>
#include <chrono>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <std_msgs/Float64.h>

#include <controller_manager/controller_manager.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <robot_constants/topics.h>
#include <robot_constants/nodes.h>

class Robot : public hardware_interface::RobotHW {
public:
    Robot() : private_nh("~") {
//        private_nh.param<double>("max_speed", max_speed_, 1.0);

        std::fill_n(cmd, NUM_JOINTS, 0.0f);
        std::fill_n(pos, NUM_JOINTS, 0.0f);
        std::fill_n(vel, NUM_JOINTS, 0.0f);
        std::fill_n(eff, NUM_JOINTS, 0.0f);
        std::fill_n(wheel_angle_, NUM_JOINTS, 0.0f);

        hardware_interface::JointStateHandle left_joint_state_handle(
                LEFT_JOINT_NAME,
                &pos[LEFT_JOINT_IDX],
                &vel[LEFT_JOINT_IDX],
                &eff[LEFT_JOINT_IDX]);
        jnt_state_interface_.registerHandle(left_joint_state_handle);
        hardware_interface::JointStateHandle right_joint_state_handle(
                RIGHT_JOINT_NAME,
                &pos[RIGHT_JOINT_IDX],
                &vel[RIGHT_JOINT_IDX],
                &eff[RIGHT_JOINT_IDX]);
        jnt_state_interface_.registerHandle(right_joint_state_handle);
        registerInterface(&jnt_state_interface_);

        hardware_interface::JointHandle left_joint_vel_handle(
                jnt_state_interface_.getHandle(LEFT_JOINT_NAME),
                &cmd[LEFT_JOINT_IDX]);
        vel_jnt_interface_.registerHandle(left_joint_vel_handle);
        hardware_interface::JointHandle right_joint_vel_handle(
                jnt_state_interface_.getHandle(RIGHT_JOINT_NAME),
                &cmd[RIGHT_JOINT_IDX]);
        vel_jnt_interface_.registerHandle(right_joint_vel_handle);
        registerInterface(&vel_jnt_interface_);

        // Initialize publishers and subscribers
        left_wheel_setpoint_pub_ = nh.advertise<std_msgs::Float64>(
                robot_constants::topics::kLeftWheelSetpoint, 1);
        right_wheel_setpoint_pub_ = nh.advertise<std_msgs::Float64>(
                robot_constants::topics::kRightWheelSetpoint, 1);
    }

//    void LimitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right) {
//        double speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));
//        if (speed > max_speed_) {
//            diff_speed_left *= max_speed_ / speed;
//            diff_speed_right *= max_speed_ / speed;
//        }
//    }

    void write() {
        // angular velocity
        double cmd_left = cmd[LEFT_JOINT_IDX];
        double cmd_right = cmd[RIGHT_JOINT_IDX];

//      LimitDifferentialSpeed(cmd_left, cmd_right);

        std_msgs::Float64 left_wheel_setpoint_msg;
        left_wheel_setpoint_msg.data = cmd_left;
        left_wheel_setpoint_pub_.publish(left_wheel_setpoint_msg);

        std_msgs::Float64 right_wheel_setpoint_msg;
        right_wheel_setpoint_msg.data = cmd_right;
        right_wheel_setpoint_pub_.publish(right_wheel_setpoint_msg);
    }

    // TODO synchronize with fw_node
    void read(const ros::Duration& /*period*/) {
        pos[LEFT_JOINT_IDX] = ros::topic::waitForMessage<std_msgs::Float64>(
                robot_constants::topics::kLeftWheelPosition)->data;
        // $(find robot_base)/launch/hw_base.launch
        vel[LEFT_JOINT_IDX] = ros::topic::waitForMessage<std_msgs::Float64>(
                robot_constants::topics::kLeftWheelVelocity)->data;
        eff[LEFT_JOINT_IDX] = ros::topic::waitForMessage<std_msgs::Float64>(
                robot_constants::topics::kLeftWheelControlEffort)->data;

        pos[RIGHT_JOINT_IDX] = ros::topic::waitForMessage<std_msgs::Float64>(
                robot_constants::topics::kRightWheelPosition)->data;
        // $(find robot_base)/launch/hw_base.launch
        vel[RIGHT_JOINT_IDX] = ros::topic::waitForMessage<std_msgs::Float64>(
                robot_constants::topics::kRightWheelVelocity)->data;
        eff[RIGHT_JOINT_IDX] = ros::topic::waitForMessage<std_msgs::Float64>(
                robot_constants::topics::kRightWheelControlEffort)->data;
    }

    ros::Time get_time() {
        prev_update_time_ = cur_update_time_;
        cur_update_time_ = ros::Time::now();
        return cur_update_time_;
    }

    ros::Duration get_period() {
        return cur_update_time_ - prev_update_time_;
    }

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

private:
    static const size_t NUM_JOINTS = 2;

    // $(find robot_base)/param/robot_base.yaml
    const std::string LEFT_JOINT_NAME = "left_wheel_joint";
    const std::string RIGHT_JOINT_NAME = "right_wheel_joint";

    const size_t LEFT_JOINT_IDX = 0;
    const size_t RIGHT_JOINT_IDX = 1;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::VelocityJointInterface vel_jnt_interface_;
    double cmd[NUM_JOINTS];
    double pos[NUM_JOINTS];
    double vel[NUM_JOINTS];
    double eff[NUM_JOINTS];

//    double max_speed_;
    double wheel_angle_[NUM_JOINTS];

    ros::Time cur_update_time_;
    ros::Time prev_update_time_;

    ros::Publisher left_wheel_setpoint_pub_;
    ros::Publisher right_wheel_setpoint_pub_;

};  // class

void controlLoop(Robot& hw,
                 controller_manager::ControllerManager& cm,
                 std::chrono::system_clock::time_point& last_time) {
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - last_time;
    ros::Duration elapsed(elapsed_time.count());
    last_time = current_time;

    hw.read(elapsed);
    cm.update(ros::Time::now(), elapsed);
    hw.write();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, robot_constants::nodes::kBaseNode);

    Robot hw;
    controller_manager::ControllerManager cm(&hw, hw.nh);

    double control_frequency;
    hw.private_nh.param<double>("control_frequency", control_frequency, 100);

    ros::CallbackQueue my_robot_queue;
    ros::AsyncSpinner my_robot_spinner(1, &my_robot_queue);
    std::chrono::system_clock::time_point last_time = std::chrono::system_clock::now();
    ros::TimerOptions control_timer(
            ros::Duration(1 / control_frequency), 
            std::bind(controlLoop, std::ref(hw), std::ref(cm), std::ref(last_time)), 
            &my_robot_queue);
    ros::Timer control_loop = hw.nh.createTimer(control_timer);
    my_robot_spinner.start();
    ros::spin();

    return 0;
}
