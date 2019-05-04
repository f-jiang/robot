#include <algorithm>
#include <functional>
#include <chrono>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <std_msgs/Float32.h>

#include <controller_manager/controller_manager.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <robot_constants/topics.h>
#include <robot_constants/nodes.h>

class Robot : public hardware_interface::RobotHW {
public:
    static const size_t LEFT_JOINT = 0;
    static const size_t RIGHT_JOINT = 1;
    static const size_t NUM_JOINTS = 2;

    Robot()
        : private_nh("~") {
        private_nh.param<double>("max_speed", max_speed_, 1.0);

        std::fill_n(cmd, NUM_JOINTS, 0.0f);
        std::fill_n(pos, NUM_JOINTS, 0.0f);
        std::fill_n(vel, NUM_JOINTS, 0.0f);
        std::fill_n(eff, NUM_JOINTS, 0.0f);
        std::fill_n(wheel_angle_, NUM_JOINTS, 0.0f);

        // TODO update this code
        // connect and register the joint state and velocity interfaces
        for (unsigned int i = 0; i < NUM_JOINTS; ++i)
        {
            std::ostringstream os;
            os << "wheel_" << i << "_joint";

            hardware_interface::JointStateHandle state_handle(os.str(), &pos[i], &vel[i], &eff[i]);
            jnt_state_interface_.registerHandle(state_handle);

            hardware_interface::JointHandle vel_handle(jnt_state_interface_.getHandle(os.str()), &cmd[i]);
            vel_jnt_interface_.registerHandle(vel_handle);
        }
        registerInterface(&jnt_state_interface_);
        registerInterface(&vel_jnt_interface_);

        // Initialize publishers and subscribers
        left_wheel_setpoint_ang_vel_pub_ = nh.advertise<std_msgs::Float32>(
                robot_constants::topics::kLeftWheelSetpointAngVel, 1);
        right_wheel_setpoint_ang_vel_pub_ = nh.advertise<std_msgs::Float32>(
                robot_constants::topics::kRightWheelSetpointAngVel, 1);
    }

    void LimitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right) {
        double speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));
        if (speed > max_speed_) {
            diff_speed_left *= max_speed_ / speed;
            diff_speed_right *= max_speed_ / speed;
        }
    }

    void Write() {
        double diff_ang_speed_left = cmd[LEFT_JOINT];
        double diff_ang_speed_right = cmd[RIGHT_JOINT];
//      LimitDifferentialSpeed(diff_ang_speed_left, diff_ang_speed_right);

        std_msgs::Float32 left_wheel_setpoint_ang_vel_msg;
        std_msgs::Float32 right_wheel_setpoint_ang_vel_msg;
        left_wheel_setpoint_ang_vel_msg.data = diff_ang_speed_left;
        right_wheel_setpoint_ang_vel_msg.data = diff_ang_speed_right;
        left_wheel_setpoint_ang_vel_pub_.publish(left_wheel_setpoint_ang_vel_msg);
        right_wheel_setpoint_ang_vel_pub_.publish(right_wheel_setpoint_ang_vel_msg);
    }

    void Read(const ros::Duration& /*period*/) {
        pos[LEFT_JOINT] = ros::topic::waitForMessage<std_msgs::Float32>(
                robot_constants::topics::kLeftWheelCurAngPos)->data;
        pos[RIGHT_JOINT] = ros::topic::waitForMessage<std_msgs::Float32>(
                robot_constants::topics::kRightWheelCurAngPos)->data;
        vel[LEFT_JOINT] = ros::topic::waitForMessage<std_msgs::Float32>(
                robot_constants::topics::kLeftWheelCurAngVel)->data;
        vel[RIGHT_JOINT] = ros::topic::waitForMessage<std_msgs::Float32>(
                robot_constants::topics::kRightWheelCurAngVel)->data;
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
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::VelocityJointInterface vel_jnt_interface_;
    double cmd[NUM_JOINTS];
    double pos[NUM_JOINTS];
    double vel[NUM_JOINTS];
    double eff[NUM_JOINTS];

    double max_speed_;
    double wheel_angle_[NUM_JOINTS];

    ros::Time cur_update_time_;
    ros::Time prev_update_time_;

    ros::Subscriber left_wheel_angle_sub_;
    ros::Subscriber right_wheel_angle_sub_;
    ros::Publisher left_wheel_setpoint_ang_vel_pub_;
    ros::Publisher right_wheel_setpoint_ang_vel_pub_;

};  // class

void controlLoop(Robot& hw,
                 controller_manager::ControllerManager& cm,
                 std::chrono::system_clock::time_point& last_time) {
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - last_time;
    ros::Duration elapsed(elapsed_time.count());
    last_time = current_time;

    hw.Read(elapsed);
    cm.update(ros::Time::now(), elapsed);
    hw.Write();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, robot_constants::nodes::kBaseNode);

    Robot hw;
    controller_manager::ControllerManager cm(&hw, hw.nh);

    double control_frequency;
    hw.private_nh.param<double>("control_frequency", control_frequency, 10.0);

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
