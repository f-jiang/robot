#include <ros/ros.h>

class MyRobotHWInterface : public hardware_interface::RobotHW
{
    public:
        MyRobotHWInterface();

        /*
         *
         */
        void write() {
            double diff_ang_speed_left = cmd[0];
            double diff_ang_speed_right = cmd[1];
            limitDifferentialSpeed(diff_ang_speed_left, diff_ang_speed_right);
            // Publish results                                                                                                                                                                                                                                  
            std_msgs::Float32 left_wheel_vel_msg;
            std_msgs::Float32 right_wheel_vel_msg;
            left_wheel_vel_msg.data = diff_ang_speed_left;
            right_wheel_vel_msg.data = diff_ang_speed_right;
            left_wheel_vel_pub_.publish(left_wheel_vel_msg);
            right_wheel_vel_pub_.publish(right_wheel_vel_msg);
        }

        /**
         * Reading encoder values and setting position and velocity of enconders 
         */
        void read(const ros::Duration &period) {
            double ang_distance_left = _wheel_angle[0];
            double ang_distance_right = _wheel_angle[1];
            pos[0] += ang_distance_left;
            vel[0] += ang_distance_left / period.toSec();
            pos[1] += ang_distance_right;
            vel[1] += ang_distance_right / period.toSec();
            /*
               std::ostringstream os;
               for (unsigned int i = 0; i < NUM_JOINTS - 1; ++i)
               {
               os << cmd[i] << ", ";
               pos[i] = cmd[i];
               }
               os << cmd[NUM_JOINTS - 1];

               ROS_INFO_STREAM("Commands for joints: " << os.str());
               */
        }

        ros::Time get_time() {
            prev_update_time = curr_update_time;
            curr_update_time = ros::Time::now();
            return curr_update_time;
        }

        ros::Duration get_period() {
            return curr_update_time - prev_update_time;
        }

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        double cmd[NUM_JOINTS];
        double pos[NUM_JOINTS];
        double vel[NUM_JOINTS];
        double eff[NUM_JOINTS];

        bool running_;
        double _wheel_diameter;
        double _max_speed;
        double _wheel_angle[NUM_JOINTS];

        ros::Time curr_update_time, prev_update_time;

        ros::Subscriber left_wheel_angle_sub_;
        ros::Subscriber right_wheel_angle_sub_;
        ros::Publisher left_wheel_vel_pub_;
        ros::Publisher right_wheel_vel_pub_;

        ros::ServiceServer start_srv_;
        ros::ServiceServer stop_srv_;

        bool start_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
        { 
            running_ = true;
            return true;
        }

        bool stop_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
        {
            running_ = false;
            return true;
        }

        void leftWheelAngleCallback(const std_msgs::Float32& msg) {
            _wheel_angle[0] = msg.data;
        }

        void rightWheelAngleCallback(const std_msgs::Float32& msg) {
            _wheel_angle[1] = msg.data;
        }

        void limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
        {
            double speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));
            if (speed > _max_speed) {
                diff_speed_left *= _max_speed / speed;
                diff_speed_right *= _max_speed / speed;
            }
        }

};  // class

MyRobotHWInterface::MyRobotHWInterface()
    : running_(true)
      , private_nh("~")
      , start_srv_(nh.advertiseService("start", &MyRobotHWInterface::start_callback, this))
      , stop_srv_(nh.advertiseService("stop", &MyRobotHWInterface::stop_callback, this)) 
{
    private_nh.param<double>("wheel_diameter", _wheel_diameter, 0.064);
    private_nh.param<double>("max_speed", _max_speed, 1.0);

    // Intialize raw data
    std::fill_n(pos, NUM_JOINTS, 0.0);
    std::fill_n(vel, NUM_JOINTS, 0.0);
    std::fill_n(eff, NUM_JOINTS, 0.0);
    std::fill_n(cmd, NUM_JOINTS, 0.0);

    // connect and register the joint state and velocity interfaces
    for (unsigned int i = 0; i < NUM_JOINTS; ++i)
    {
        std::ostringstream os;
        os << "wheel_" << i << "_joint";

        hardware_interface::JointStateHandle state_handle(os.str(), &pos[i], &vel[i], &eff[i]);
        jnt_state_interface.registerHandle(state_handle);

        hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(os.str()), &cmd[i]);
        jnt_vel_interface.registerHandle(vel_handle);
    }
    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_vel_interface);

    // Initialize publishers and subscribers
    left_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("my_robot/left_wheel_vel", 1);
    right_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("my_robot/right_wheel_vel", 1);

    left_wheel_angle_sub_ = nh.subscribe("my_robot/left_wheel_angle", 1, &MyRobotHWInterface::leftWheelAngleCallback, this);
    right_wheel_angle_sub_ = nh.subscribe("my_robot/right_wheel_angle", 1, &MyRobotHWInterface::rightWheelAngleCallback, this);
}

void controlLoop(MyRobotHWInterface &hw,
        controller_manager::ControllerManager &cm,
        std::chrono::system_clock::time_point &last_time)
{
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - last_time;
    ros::Duration elapsed(elapsed_time.count());
    last_time = current_time;

    hw.read(elapsed);
    cm.update(ros::Time::now(), elapsed);
    hw.write();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_node");

    MyRobotHWInterface hw;
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
