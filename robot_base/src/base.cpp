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

#include <robot_constants.h>

class Robot : public hardware_interface::RobotHW {
public:
    static const size_t kNumJoints = 2;

    static const char* kMaxSpeedParam;
    static const char* kTimeoutParam;

    // $(find robot_base)/param/robot_base.yaml
    const std::string kLeftJointName = "left_wheel_joint";
    const std::string kRightJointName = "right_wheel_joint";

    Robot() : private_nh("~") {
        ros::param::get(robot_constants::params::kControlFrequency, control_frequency_);

//        private_nh.param<double>(MAX_SPEED_PARAM, max_speed_, 1.0);
        private_nh.param<double>(kTimeoutParam,  timeout_,
                                 1 / (static_cast<double>(control_frequency_) /** kNumMsgsToRead*/));

        std::fill_n(cmd, kNumJoints, 0.0f);
        std::fill_n(pos, kNumJoints, 0.0f);
        std::fill_n(vel, kNumJoints, 0.0f);
        std::fill_n(eff, kNumJoints, 0.0f);

        hardware_interface::JointStateHandle left_joint_state_handle(
                kLeftJointName,
                &pos[kLeftJointIdx],
                &vel[kLeftJointIdx],
                &eff[kLeftJointIdx]);
        jnt_state_interface_.registerHandle(left_joint_state_handle);
        hardware_interface::JointStateHandle right_joint_state_handle(
                kRightJointName,
                &pos[kRightJointIdx],
                &vel[kRightJointIdx],
                &eff[kRightJointIdx]);
        jnt_state_interface_.registerHandle(right_joint_state_handle);
        registerInterface(&jnt_state_interface_);

        hardware_interface::JointHandle left_joint_vel_handle(
                jnt_state_interface_.getHandle(kLeftJointName),
                &cmd[kLeftJointIdx]);
        vel_jnt_interface_.registerHandle(left_joint_vel_handle);
        hardware_interface::JointHandle right_joint_vel_handle(
                jnt_state_interface_.getHandle(kRightJointName),
                &cmd[kRightJointIdx]);
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
        double cmd_left = cmd[kLeftJointIdx];
        double cmd_right = cmd[kRightJointIdx];

//      LimitDifferentialSpeed(cmd_left, cmd_right);

        std_msgs::Float64 left_wheel_setpoint_msg;
        left_wheel_setpoint_msg.data = cmd_left;
        left_wheel_setpoint_pub_.publish(left_wheel_setpoint_msg);

        std_msgs::Float64 right_wheel_setpoint_msg;
        right_wheel_setpoint_msg.data = cmd_right;
        right_wheel_setpoint_pub_.publish(right_wheel_setpoint_msg);
    }

    void read_msg(const std::string& topic,
                  const ros::Duration& timeout_duration,
                  double default_val,
                  double& dest) {
        auto msg_ptr = ros::topic::waitForMessage<std_msgs::Float64>(
                topic, timeout_duration);
        dest = msg_ptr ? msg_ptr->data : 0;
    }

    // TODO synchronize with fw_node (same period, but may have phase lag)
    // FIXME currently this function is taking much longer than it should given the timeout_ value
    // current timeout behaviour: if no msg received, assume 0 and carry on with remaining read() and write()
    void read(const ros::Duration& /*period*/) {
        ros::Duration timeout_duration(timeout_);

        read_msg(robot_constants::topics::kLeftWheelPosition, timeout_duration, 0, pos[kLeftJointIdx]);
        // $(find robot_base)/launch/hw_base.launch
        read_msg(robot_constants::topics::kLeftWheelVelocity, timeout_duration, 0, vel[kLeftJointIdx]);
        read_msg(robot_constants::topics::kLeftWheelControlEffort, timeout_duration, 0, eff[kLeftJointIdx]);

        read_msg(robot_constants::topics::kRightWheelPosition, timeout_duration, 0, pos[kRightJointIdx]);
        // $(find robot_base)/launch/hw_base.launch
        read_msg(robot_constants::topics::kRightWheelVelocity, timeout_duration, 0, vel[kRightJointIdx]);
        read_msg(robot_constants::topics::kRightWheelControlEffort, timeout_duration, 0, eff[kRightJointIdx]);
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
    const size_t kLeftJointIdx = 0;
    const size_t kRightJointIdx = 1;

//    const size_t kNumMsgsToRead = 6;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::VelocityJointInterface vel_jnt_interface_;
    double cmd[kNumJoints]; // rad/s
    double pos[kNumJoints]; // rad
    double vel[kNumJoints]; // rad/s
    double eff[kNumJoints]; // [-255, 255]

//    double max_speed_;
    int control_frequency_;
    double timeout_;

    ros::Time cur_update_time_;
    ros::Time prev_update_time_;

    ros::Publisher left_wheel_setpoint_pub_;
    ros::Publisher right_wheel_setpoint_pub_;

};  // class

const char* Robot::kMaxSpeedParam = "max_speed_param";
const char* Robot::kTimeoutParam = "timeout";

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

    if (!ros::param::has(robot_constants::params::kControlFrequency)) {
        ros::param::set(robot_constants::params::kControlFrequency, 100);
    }

    Robot hw;
    controller_manager::ControllerManager cm(&hw, hw.nh);

    ros::CallbackQueue my_robot_queue;
    ros::AsyncSpinner my_robot_spinner(1, &my_robot_queue);
    std::chrono::system_clock::time_point last_time = std::chrono::system_clock::now();
    int control_frequency;
    ros::param::get(robot_constants::params::kControlFrequency, control_frequency);
    ros::TimerOptions control_timer(
            ros::Duration(1 / static_cast<double>(control_frequency)),
            std::bind(controlLoop, std::ref(hw), std::ref(cm), std::ref(last_time)), 
            &my_robot_queue);
    ros::Timer control_loop = hw.nh.createTimer(control_timer);
    my_robot_spinner.start();
    ros::spin();

    return 0;
}
