#ifndef GEOMETRIC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_H

#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <chrono>
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_angular_velocity.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

#include "controller_msgs/msg/flat_target.hpp"

#include "geometric_control/common.h"
#include "geometric_control/control.h"

//误差描述方式: 1. 四元数 2. SE(3)
#define ERROR_QUATERNION 1
#define ERROR_GEOMETRIC 2

//
#define POSITION_MODE 11
#define VELOCITY_MODE 12
#define ACCELERATION_MODE 13
#define ATTITUDE_MODE 14
#define ANGULAR_RATE_MODE 15

using namespace Eigen;
using namespace std::chrono_literals;

enum class MAV_STATE {
    MAV_STATE_UNINIT,
    MAV_STATE_BOOT,
    MAV_STATE_CALIBRATIN,
    MAV_STATE_STANDBY,
    MAV_STATE_ACTIVE,
    MAV_STATE_CRITICAL,
    MAV_STATE_EMERGENCY,
    MAV_STATE_POWEROFF,
    MAV_STATE_FLIGHT_TERMINATION,
};

class geometricCtrl : public rclcpp::Node {
public:
    geometricCtrl();

    ~geometricCtrl();

    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameters_event_sub_;

    void on_parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

    void getStates(Eigen::Vector3d &pos, Eigen::Vector4d &att, Eigen::Vector3d &vel, Eigen::Vector3d &angvel) {
        pos = mavPos_;
        att = mavAtt_;
        vel = mavVel_;
        angvel = mavRate_;
    };

    void getErrors(Eigen::Vector3d &pos, Eigen::Vector3d &vel) {
        pos = mavPos_ - targetPos_;
        vel = mavVel_ - targetVel_;
    };

    void setBodyRateCommand(Eigen::Vector4d bodyrate_command) { cmdBodyRate_ = bodyrate_command; };

    void setDesiredAcceleration(Eigen::Vector3d &acceleration) { targetAcc_ = acceleration; };

    static Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);

    static double getVelocityYaw(const Eigen::Vector3d velocity) { return atan2(velocity(1), velocity(0)); };

private:
    // Subscription:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr referenceSub_;

    rclcpp::Subscription<controller_msgs::msg::FlatTarget>::SharedPtr flatReferenceSub_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yawReferenceSub_;

    rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr multiDOFJointSub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr mavStateSub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr mavPoseSub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr mavAttitudeSub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr mavRatesSub_;

    // Publisher:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr referencePosePub_;

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr targetPosePub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboardControlModePub_;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicleCommandPub_;

    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitudeSetpointPub_;

    rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr angularVelPub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr poseHistoryPub_;


    // Timer:
    rclcpp::TimerBase::SharedPtr cmdLoop_timer_;
    rclcpp::TimerBase::SharedPtr input_timer_;

//    rclcpp::TimerBase::SharedPtr statusLoop_timer_;

    // Service:

    // Callback Function:
    void targetCallback(const geometry_msgs::msg::TwistStamped::UniquePtr &msg);

    void flattargetCallback(const controller_msgs::msg::FlatTarget::UniquePtr &msg);

    void yawtargetCallback(const std_msgs::msg::Float32::UniquePtr &msg);

    void multiDOFJointCallback(const trajectory_msgs::msg::MultiDOFJointTrajectory::UniquePtr &msg);

    void mavstateCallback(const px4_msgs::msg::VehicleStatus::UniquePtr &msg);

    void mavposeCallback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr &msg);

    void mavattitudeCallback(const px4_msgs::msg::VehicleAttitude::UniquePtr &msg);

    void mavtratesCallback(const px4_msgs::msg::VehicleAngularVelocity::UniquePtr &msg);

    void cmdloopCallback();

    void inputCallback();

//    void statusloopCallback();

    // Parameter Function:
    void DeclareParams();

    void GetParams();

    void PrintParams();

    // Other Function:
    void arm();

    void disarm();

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    void publish_offboard_control_mode(int MODE);

    void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &target_acc);

    Eigen::Vector3d controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                    const Eigen::Vector3d &target_acc);

    Eigen::Vector3d poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error);

    Eigen::Vector4d attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
                                  Eigen::Vector4d &curr_att);

    void pubMotorCommands();

    void pubRateCommands(const Eigen::Vector4d &cmd);

    void pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude);

    void publish_trajectory_setpoint(float x = 0.0, float y = 0.0, float z = -1.0, float yaw = 0.0);

    void pubPoseHistory();

    void pubSystemStatus();

    void appendPoseHistory();

    void odomCallback(const nav_msgs::msg::Odometry::UniquePtr &odomMsg);

    void keyboardCallback(const geometry_msgs::msg::Twist &msg);

    geometry_msgs::msg::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d &position, Eigen::Vector4d &orientation);

    std::size_t offboard_setpoint_counter_;
    // Parameter:
    std::string mav_name_{};
    int ctrl_mode_{};
    bool sim_enable_{};
    float geofence_x, geofence_y, geofence_z;
    bool velocity_yaw_{};
    double max_fb_acc_{};
    double mavYaw_{};
    double attctrl_tau{};
    Eigen::Vector3d Kpos_, Kvel_, D_;
    double norm_thrust_const_{}, norm_thrust_offset_{};
    double Kpos_x_{}, Kpos_y_{}, Kpos_z_{}, Kvel_x_{}, Kvel_y_{}, Kvel_z_{};
    double initTargetPos_x_{}, initTargetPos_y_{}, initTargetPos_z_{};
    std::vector<double> initTargetPos_;
    int posehistory_window_{};

    bool fail_detec_{false};
    bool feedthrough_enable_{false};
    bool ctrl_enable_{true};
    bool landing_commanded_{false};
    double kp_rot_{}, kd_rot_{};
    double reference_request_dt_{};
    px4_msgs::msg::VehicleStatus current_state_;
    std::vector<geometry_msgs::msg::PoseStamped> posehistory_vector_;
//    MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;

    rclcpp::Time last_request_{}, reference_request_now_{}, reference_request_last_{};

    Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetJerk_, targetSnap_, targetPos_prev_, targetVel_prev_;
    Eigen::Vector3d mavPos_, mavVel_, mavRate_;
    Eigen::Vector3d gravity_{Eigen::Vector3d(0.0, 0.0, -9.8)};
    Eigen::Vector4d mavAtt_, q_des;
    Eigen::Vector4d cmdBodyRate_;  //{wx, wy, wz, Thrust}

    enum FlightState {
        WAITING_FOR_HOME_POSE, MISSION_EXECUTION, LANDING, LANDED
    } node_state;
    geometry_msgs::msg::Pose home_pose_;
    bool received_home_pose{};
    std::shared_ptr<Control> controller_;
};


#endif
