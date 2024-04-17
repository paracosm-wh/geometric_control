#include "geometric_control/geometric_controller.h"
#include "geometric_control/jerk_tracking_control.h"
#include "geometric_control/nonlinear_attitude_control.h"
#include "geometric_control/nonlinear_geometric_control.h"
#include <sys/select.h>

using namespace Eigen;

rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

geometricCtrl::geometricCtrl() : Node("geometric_controller") {

    DeclareParams();
    GetParams();
    PrintParams();
    offboard_setpoint_counter_ = 0;
    referenceSub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "reference/setpoint", 10, std::bind(&geometricCtrl::targetCallback, this, std::placeholders::_1));

    flatReferenceSub_ = this->create_subscription<controller_msgs::msg::FlatTarget>(
            "reference/flatsetpoint", qos, std::bind(&geometricCtrl::flattargetCallback, this, std::placeholders::_1));

    yawReferenceSub_ = this->create_subscription<std_msgs::msg::Float32>(
            "reference/yaw", qos, std::bind(&geometricCtrl::yawtargetCallback, this, std::placeholders::_1));

    multiDOFJointSub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
            "command/trajectory", qos, std::bind(&geometricCtrl::multiDOFJointCallback, this, std::placeholders::_1));

    mavStateSub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos, std::bind(&geometricCtrl::mavstateCallback, this, std::placeholders::_1));

    mavPoseSub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            std::bind(&geometricCtrl::mavposeCallback, this, std::placeholders::_1));

    mavAttitudeSub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            std::bind(&geometricCtrl::mavattitudeCallback, this, std::placeholders::_1));

    mavRatesSub_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
            "/fmu/out/vehicle_angular_velocity", qos,
            std::bind(&geometricCtrl::mavtratesCallback, this, std::placeholders::_1));

    cmdLoop_timer_ = this->create_wall_timer(std::chrono::duration<double>(0.01),
                                             std::bind(&geometricCtrl::cmdloopCallback, this));

//    statusLoop_timer_ = this->create_wall_timer(std::chrono::duration<double>(1),
//                                                std::bind(&geometricCtrl::statusloopCallback, this));

//    ctrltriggerServ_ = nh_.advertiseService("trigger_rlcontroller", &geometricCtrl::ctrltriggerCallback, this);

    referencePosePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("reference/pose", 1);

    targetPosePub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "fmu/in/trajectory_setpoint", 10);

    offboardControlModePub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 1);

    vehicleCommandPub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "fmu/in/vehicle_command", 1);

    attitudeSetpointPub_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "/fmu/in/vehicle_attitude_setpoint", 1);

    angularVelPub_ = this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
            "/fmu/in/vehicle_rates_setpoint", 1);

    poseHistoryPub_ = this->create_publisher<nav_msgs::msg::Path>("geometric_controller/path", 10);

//    systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);

//    land_service_ = nh_.advertiseService("land", &geometricCtrl::landCallback, this);

    /*
     * 动态参数配置（GUI）
     * 参考：https://www.jianshu.com/p/d31b514ee5d2
     * 1. rqt->Plugins->Configuration->Dynamic Reconfigure
     * 2. 设置参数改变的回调函数
     */
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
            this->get_node_base_interface(),
            this->get_node_topics_interface(),
            this->get_node_graph_interface(),
            this->get_node_services_interface());

    parameters_event_sub_ = parameters_client_->on_parameter_event(
            std::bind(&geometricCtrl::on_parameter_event_callback, this, std::placeholders::_1));


    bool jerk_enabled = false;
    if (!jerk_enabled) {
        if (ctrl_mode_ == ERROR_GEOMETRIC) {
            controller_ = std::make_shared<NonlinearGeometricControl>(attctrl_tau);
        } else {
            controller_ = std::make_shared<NonlinearAttitudeControl>(attctrl_tau);
        }
    } else {
//        controller_ = std::make_shared<JerkTrackingControl>();
    }


}

geometricCtrl::~geometricCtrl() {
    // Destructor
}

void geometricCtrl::targetCallback(const geometry_msgs::msg::TwistStamped::UniquePtr &msg) {
    reference_request_last_ = reference_request_now_;

    targetPos_prev_ = targetPos_;
    targetVel_prev_ = targetVel_;

    reference_request_now_ = this->get_clock()->now();
    reference_request_dt_ = (reference_request_now_ - reference_request_last_).seconds();

    targetPos_ = toEigen(msg->twist.angular);
    targetVel_ = toEigen(msg->twist.linear);

    if (reference_request_dt_ > 0)
        targetAcc_ = (targetVel_ - targetVel_prev_) / reference_request_dt_;
    else
        targetAcc_ = Eigen::Vector3d::Zero();
}

void geometricCtrl::flattargetCallback(const controller_msgs::msg::FlatTarget::UniquePtr &msg) {
    reference_request_last_ = reference_request_now_;

    targetPos_prev_ = targetPos_;
    targetVel_prev_ = targetVel_;

    reference_request_now_ = this->get_clock()->now();
    reference_request_dt_ = (reference_request_now_ - reference_request_last_).seconds();

    targetPos_ = toEigen(msg->position);
    targetVel_ = toEigen(msg->velocity);

    if (msg->type_mask == 1) {
        targetAcc_ = toEigen(msg->acceleration);
        targetJerk_ = toEigen(msg->jerk);
        targetSnap_ = Eigen::Vector3d::Zero();

    } else if (msg->type_mask == 2) {
        feedthrough_enable_ = true;
        targetAcc_ = toEigen(msg->acceleration);
        targetJerk_ = Eigen::Vector3d::Zero();
        targetSnap_ = Eigen::Vector3d::Zero();

    } else if (msg->type_mask == 4) {
        targetAcc_ = Eigen::Vector3d::Zero();
        targetJerk_ = Eigen::Vector3d::Zero();
        targetSnap_ = Eigen::Vector3d::Zero();

    } else {
        targetAcc_ = toEigen(msg->acceleration);
        targetJerk_ = toEigen(msg->jerk);
        targetSnap_ = toEigen(msg->snap);
    }
}

void geometricCtrl::yawtargetCallback(const std_msgs::msg::Float32::UniquePtr &msg) {
    if (!velocity_yaw_) mavYaw_ = double(msg->data);
}

void geometricCtrl::multiDOFJointCallback(const trajectory_msgs::msg::MultiDOFJointTrajectory::UniquePtr &msg) {
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint pt = msg->points[0];
    reference_request_last_ = reference_request_now_;

    targetPos_prev_ = targetPos_;
    targetVel_prev_ = targetVel_;

    reference_request_now_ = this->get_clock()->now();
    reference_request_dt_ = (reference_request_now_ - reference_request_last_).seconds();

    targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
    targetVel_ << pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z;

    targetAcc_ << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z;
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();

    if (!velocity_yaw_) {
        Eigen::Quaterniond q(pt.transforms[0].rotation.w, pt.transforms[0].rotation.x, pt.transforms[0].rotation.y,
                             pt.transforms[0].rotation.z);
        Eigen::Vector3d rpy = Eigen::Matrix3d(q).eulerAngles(0, 1, 2);  // RPY
        mavYaw_ = rpy(2);
    }
}

void geometricCtrl::mavstateCallback(const px4_msgs::msg::VehicleStatus::UniquePtr &msg) { current_state_ = *msg; }

void geometricCtrl::mavposeCallback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr &msg) {
    if (!received_home_pose) {
        received_home_pose = true;
        home_pose_.position.x = msg->x;
        home_pose_.position.y = msg->y;
        home_pose_.position.z = msg->z;
        RCLCPP_INFO_STREAM(this->get_logger(), "Home pose initialized to: " << home_pose_.position.x << ", "
                                                                            << home_pose_.position.y << ", "
                                                                            << home_pose_.position.z);
    }
    mavPos_ << msg->y, msg->x, -msg->z;
    mavVel_ << msg->vy, msg->vx, -msg->vz;
//    RCLCPP_INFO_STREAM(this->get_logger(), "mavPos: " << mavPos_(0) << ", " << mavPos_(1) << ", " << mavPos_(2));
}

void geometricCtrl::mavattitudeCallback(const px4_msgs::msg::VehicleAttitude::UniquePtr &msg) {

    Eigen::Quaterniond q_ned2enu(0, 0.707, 0.707, 0); // NED to ENU (w,x,y,z)
    Eigen::Quaterniond q_enu, q_ned;
    q_ned.w() = msg->q[0];
    q_ned.x() = msg->q[1];
    q_ned.y() = msg->q[2];
    q_ned.z() = msg->q[3];
//    q_enu = q_ned2enu * q_ned * q_ned2enu.conjugate();
    q_enu = q_ned2enu * q_ned * q_ned2enu.inverse();
    mavAtt_ << q_enu.w(), q_enu.x(), q_enu.y(), q_enu.z();
}

void geometricCtrl::mavtratesCallback(const px4_msgs::msg::VehicleAngularVelocity::UniquePtr &msg) {
    mavRate_ << msg->xyz[1], msg->xyz[0], -msg->xyz[2];
}

void geometricCtrl::cmdloopCallback() {
    // TODO: 修改实机程序
    if (current_state_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        if (current_state_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
            this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            this->arm();
            RCLCPP_INFO_STREAM(this->get_logger(), "Offboard enabled");
        } else {
//            RCLCPP_INFO_STREAM(this->get_logger(), "Vehicle armed");
            publish_offboard_control_mode(ANGULAR_RATE_MODE);
            Eigen::Vector3d desired_acc;
            if (feedthrough_enable_) {
                desired_acc = targetAcc_;
            } else {
                desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_);
            }
            computeBodyRateCmd(cmdBodyRate_, desired_acc);
            pubReferencePose(targetPos_, q_des);
            pubRateCommands(cmdBodyRate_);
//            std::cout << "desired_position:" << targetPos_.z() << std::endl;
            appendPoseHistory();
            pubPoseHistory();
        }
    } else {
        this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        publish_offboard_control_mode(POSITION_MODE);
        publish_trajectory_setpoint();
        RCLCPP_INFO_STREAM(this->get_logger(), "Offboard disabled");
    }
}


void geometricCtrl::DeclareParams() {
    // Declare parameters that can be modified via launch file or command line.
    RCLCPP_INFO_STREAM(this->get_logger(), "Declaring controller parameters");
    this->declare_parameter<std::string>("mavname", "iris");
    this->declare_parameter<int>("ctrl_mode", ERROR_GEOMETRIC);
    this->declare_parameter<bool>("enable_sim", true);
    this->declare_parameter<bool>("feedthrough_enable", false);
    this->declare_parameter<bool>("velocity_yaw", false);
    this->declare_parameter<double>("max_acc", 10.0);
    this->declare_parameter<double>("yaw_heading", 0.0);
    this->declare_parameter<double>("drag_dx", 0.0);
    this->declare_parameter<double>("drag_dy", 0.0);
    this->declare_parameter<double>("drag_dz", 0.0);
    this->declare_parameter<double>("attctrl_constant", 0.3);
    this->declare_parameter<double>("normalizedthrust_constant", 0.06);
    this->declare_parameter<double>("normalizedthrust_offset", 0.1);
    this->declare_parameter<double>("Kp_x", 2.0);
    this->declare_parameter<double>("Kp_y", 2.0);
    this->declare_parameter<double>("Kp_z", 4.0);
    this->declare_parameter<double>("Kv_x", 4.0);
    this->declare_parameter<double>("Kv_y", 4.0);
    this->declare_parameter<double>("Kv_z", 4.0);
    this->declare_parameter<int>("posehistory_window", 200);
    this->declare_parameter<std::vector<double>>("init_pos", {0.0, 0.0, 1.0});
    this->declare_parameter<double>("init_pos_x", 0.0);
    this->declare_parameter<double>("init_pos_y", 0.0);
    this->declare_parameter<double>("init_pos_z", 1.0);
}

void geometricCtrl::GetParams() {
    RCLCPP_INFO_STREAM(this->get_logger(), "Declaring controller parameters");
    this->get_parameter("mavname", mav_name_);
    this->get_parameter("ctrl_mode", ctrl_mode_);
    this->get_parameter("enable_sim", sim_enable_);
    this->get_parameter("feedthrough_enable", feedthrough_enable_);
    this->get_parameter("velocity_yaw", velocity_yaw_);
    this->get_parameter("max_acc", max_fb_acc_);
    this->get_parameter("yaw_heading", mavYaw_);
    double dx, dy, dz;
    this->get_parameter("drag_dx", dx);
    this->get_parameter("drag_dy", dy);
    this->get_parameter("drag_dz", dz);
    D_ << dx, dy, dz;
    this->get_parameter("attctrl_constant", attctrl_tau);
    this->get_parameter("normalizedthrust_constant", norm_thrust_const_);
    this->get_parameter("normalizedthrust_offset", norm_thrust_offset_);
    this->get_parameter("Kp_x", Kpos_x_);
    this->get_parameter("Kp_y", Kpos_y_);
    this->get_parameter("Kp_z", Kpos_z_);
    this->get_parameter("Kv_x", Kvel_x_);
    this->get_parameter("Kv_y", Kvel_y_);
    this->get_parameter("Kv_z", Kvel_z_);
    this->get_parameter("posehistory_window", posehistory_window_);
    this->get_parameter("init_pos", initTargetPos_);
    this->get_parameter("init_pos_x", initTargetPos_x_);
    this->get_parameter("init_pos_y", initTargetPos_y_);
    this->get_parameter("init_pos_z", initTargetPos_z_);

    targetPos_ << initTargetPos_[0], initTargetPos_[1], initTargetPos_[2];
    targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;  // Initial Position
    targetVel_ << 0.0, 0.0, 0.0;
    mavPos_ << 0.0, 0.0, 0.0;
    mavVel_ << 0.0, 0.0, 0.0;
    Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
    Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;

    bool jerk_enabled = false;
    if (!jerk_enabled) {
        if (ctrl_mode_ == ERROR_GEOMETRIC) {
            controller_ = std::make_shared<NonlinearGeometricControl>(attctrl_tau);
        } else {
            controller_ = std::make_shared<NonlinearAttitudeControl>(attctrl_tau);
        }
    } else {
//        controller_ = std::make_shared<JerkTrackingControl>();
    }
}

void geometricCtrl::PrintParams() {
    RCLCPP_INFO_STREAM(this->get_logger(), "Printing controller parameters");

    RCLCPP_INFO_STREAM(this->get_logger(), "mavname = " << mav_name_);
    RCLCPP_INFO_STREAM(this->get_logger(), "ctrl_mode = " << ctrl_mode_);
    RCLCPP_INFO_STREAM(this->get_logger(), "enable_sim = " << sim_enable_);
    RCLCPP_INFO_STREAM(this->get_logger(), "feedthrough_enable = " << feedthrough_enable_);
    RCLCPP_INFO_STREAM(this->get_logger(), "velocity_yaw = " << velocity_yaw_);
    RCLCPP_INFO_STREAM(this->get_logger(), "max_acc = " << max_fb_acc_);
    RCLCPP_INFO_STREAM(this->get_logger(), "yaw_heading = " << mavYaw_);
    RCLCPP_INFO_STREAM(this->get_logger(), "drag_dx = " << D_(0));
    RCLCPP_INFO_STREAM(this->get_logger(), "drag_dy = " << D_(1));
    RCLCPP_INFO_STREAM(this->get_logger(), "drag_dz = " << D_(2));
    RCLCPP_INFO_STREAM(this->get_logger(), "attctrl_constant = " << attctrl_tau);
    RCLCPP_INFO_STREAM(this->get_logger(), "normalizedthrust_constant = " << norm_thrust_const_);
    RCLCPP_INFO_STREAM(this->get_logger(), "normalizedthrust_offset = " << norm_thrust_offset_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Kp_x = " << Kpos_x_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Kp_y = " << Kpos_y_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Kp_z = " << Kpos_z_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Kv_x = " << Kvel_x_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Kv_y = " << Kvel_y_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Kv_z = " << Kvel_z_);
    RCLCPP_INFO_STREAM(this->get_logger(), "posehistory_window = " << posehistory_window_);
    RCLCPP_INFO_STREAM(this->get_logger(), "init_pos_x = " << initTargetPos_x_);
    RCLCPP_INFO_STREAM(this->get_logger(), "init_pos_y = " << initTargetPos_y_);
    RCLCPP_INFO_STREAM(this->get_logger(), "init_pos_z = " << initTargetPos_z_);
}

void geometricCtrl::on_parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {

    for (auto &changed_parameter: event->changed_parameters) {
        const auto &type = changed_parameter.value.type;
        const auto &name = changed_parameter.name;
        const auto &value = changed_parameter.value;

        if (name == "max_acc") {
            max_fb_acc_ = value.double_value;
            RCLCPP_INFO(this->get_logger(), "Reconfigure request : max_acc = %.2f ", value.double_value);
        } else if (name == "Kp_x") {
            Kpos_x_ = value.double_value;
            RCLCPP_INFO(this->get_logger(), "Reconfigure request : Kp_x  = %.2f  ", value.double_value);
        } else if (name == "Kp_y") {
            Kpos_y_ = value.double_value;
            RCLCPP_INFO(this->get_logger(), "Reconfigure request : Kp_y  = %.2f  ", value.double_value);
        } else if (name == "Kp_z") {
            Kpos_z_ = value.double_value;
            RCLCPP_INFO(this->get_logger(), "Reconfigure request : Kp_z  = %.2f  ", value.double_value);
        } else if (name == "Kv_x") {
            Kvel_x_ = value.double_value;
            RCLCPP_INFO(this->get_logger(), "Reconfigure request : Kv_x  = %.2f  ", value.double_value);
        } else if (name == "Kv_y") {
            Kvel_y_ = value.double_value;
            RCLCPP_INFO(this->get_logger(), "Reconfigure request : Kv_y =%.2f  ", value.double_value);
        } else if (name == "Kv_z") {
            Kvel_z_ = value.double_value;
            RCLCPP_INFO(this->get_logger(), "Reconfigure request : Kv_z  = %.2f  ", value.double_value);
        } else if (name == "init_pos_x") {
            initTargetPos_x_ = value.double_value;
            RCLCPP_INFO(this->get_logger(), "Reconfigure request : init_pos_x  = %.2f  ", value.double_value);
        } else if (name == "init_pos_y") {
            initTargetPos_y_ = value.double_value;
            RCLCPP_INFO(this->get_logger(), "Reconfigure request : init_pos_y  = %.2f  ", value.double_value);
        } else if (name == "init_pos_z") {
            initTargetPos_z_ = value.double_value;
            RCLCPP_INFO(this->get_logger(), "Reconfigure request : init_pos_z  = %.2f  ", value.double_value);
        } else if (name == "init_pos") {
            initTargetPos_ = value.double_array_value;
            RCLCPP_INFO(this->get_logger(), "Reconfigure request : init_pos  = %.2f  ", value.double_value);
        }

        Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
        Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
        targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;
        targetPos_ << initTargetPos_[0], initTargetPos_[1], initTargetPos_[2];
        mavYaw_ = initTargetPos_[3];
    }
}

/**
 * Send a command to Arm the vehicle
 */
void geometricCtrl::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * Send a command to Disarm the vehicle
 */
void geometricCtrl::disarm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void geometricCtrl::publish_vehicle_command(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicleCommandPub_->publish(msg);
}

/**
 * Publish the offboard control mode.
 */
void geometricCtrl::publish_offboard_control_mode(int MODE) {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    switch (MODE) {
        case 11:
            msg.position = true;
            break;
        case 12:
            msg.velocity = true;
            break;
        case 13:
            msg.acceleration = true;
            break;
        case 14:
            msg.attitude = true;
            break;
        case 15:
            msg.body_rate = true;
            break;
        default:
            std::cout << "Invalid Mode!!!";
    }
    offboardControlModePub_->publish(msg);
}

/**
 * @itr_wh Update position_setpoint in Position mode
 */
void geometricCtrl::publish_trajectory_setpoint(float x, float y, float z, float yaw) {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = {x, y, z};
    msg.yaw = yaw; // [-PI:PI]
    targetPosePub_->publish(msg);
}


/*
 * @itr_wh Publish desired 
 */
void
geometricCtrl::pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude) {
    geometry_msgs::msg::PoseStamped msg;

    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";
    msg.pose.position.x = target_position(0);
    msg.pose.position.y = target_position(1);
    msg.pose.position.z = target_position(2);
    msg.pose.orientation.w = target_attitude(0);
    msg.pose.orientation.x = target_attitude(1);
    msg.pose.orientation.y = target_attitude(2);
    msg.pose.orientation.z = target_attitude(3);
    referencePosePub_->publish(msg);
}

void geometricCtrl::pubRateCommands(const Eigen::Vector4d &cmd) {
    px4_msgs::msg::VehicleRatesSetpoint msg;

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.roll = (float) cmd(1);
    msg.pitch = (float) cmd(0);
    msg.yaw = -(float) cmd(2);
    msg.thrust_body[0] = 0.0;
    msg.thrust_body[1] = 0.0;
    msg.thrust_body[2] = -(float) cmd(3);

    angularVelPub_->publish(msg);
}

void geometricCtrl::pubPoseHistory() {
    nav_msgs::msg::Path msg;

    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";
    msg.poses = posehistory_vector_;

    poseHistoryPub_->publish(msg);
}


void geometricCtrl::appendPoseHistory() {
    posehistory_vector_.insert(posehistory_vector_.begin(), vector3d2PoseStampedMsg(mavPos_, mavAtt_));
    if (posehistory_vector_.size() > posehistory_window_) {
        posehistory_vector_.pop_back();
    }
}

geometry_msgs::msg::PoseStamped geometricCtrl::vector3d2PoseStampedMsg(Eigen::Vector3d &position,
                                                                       Eigen::Vector4d &orientation) {
    geometry_msgs::msg::PoseStamped encode_msg;
    encode_msg.header.stamp = this->get_clock()->now();
    encode_msg.header.frame_id = "map";
    encode_msg.pose.orientation.w = orientation(0);
    encode_msg.pose.orientation.x = orientation(1);
    encode_msg.pose.orientation.y = orientation(2);
    encode_msg.pose.orientation.z = orientation(3);
    encode_msg.pose.position.x = position(0);
    encode_msg.pose.position.y = position(1);
    encode_msg.pose.position.z = position(2);
    return encode_msg;
}


/*
 * 位置控制
 * /// Compute BodyRate commands using differential flatness
    /// Controller based on Faessler 2017
 */
Eigen::Vector3d geometricCtrl::controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                               const Eigen::Vector3d &target_acc) {

    const Eigen::Vector3d a_ref = target_acc;
    if (velocity_yaw_) {
        mavYaw_ = getVelocityYaw(mavVel_);
    }

    const Eigen::Vector4d q_ref = acc2quaternion(a_ref - gravity_, mavYaw_);
    const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

    const Eigen::Vector3d pos_error = mavPos_ - target_pos;
    const Eigen::Vector3d vel_error = mavVel_ - target_vel;

    // Position Controller
    const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);

    // Rotor Drag compensation
    const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel;  // Rotor drag

    // Reference acceleration
    const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - gravity_;

//    std::cout<<"a_des: "<<a_des<<std::endl;
    return a_des;
}

void geometricCtrl::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des) {
    // Reference attitude
    q_des = acc2quaternion(a_des, mavYaw_);

    controller_->Update(mavAtt_, q_des, a_des, targetJerk_);  // Calculate BodyRate
    bodyrate_cmd.head(3) = controller_->getDesiredRate();
    double thrust_command = controller_->getDesiredThrust().z();
    bodyrate_cmd(3) =
            std::max(0.0, std::min(1.0, norm_thrust_const_ * thrust_command +
                                        norm_thrust_offset_));  // Calculate thrustcontroller_->getDesiredThrust()(3);
//    std::cout << "thrust:" << bodyrate_cmd(3) << std::endl;
}

Eigen::Vector3d geometricCtrl::poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error) {
    Eigen::Vector3d a_fb =
            Kpos_.asDiagonal() * pos_error +
            Kvel_.asDiagonal() * vel_error;  // feedforward term for trajectory error

    if (a_fb.norm() > max_fb_acc_)
        a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large

    return a_fb;
}

Eigen::Vector4d geometricCtrl::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
    Eigen::Vector4d quat;
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
    Eigen::Matrix3d rotmat;

    proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

    zb_des = vector_acc / vector_acc.norm();
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
    xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

    rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
    quat = rot2Quaternion(rotmat);
    return quat;
}

