#include "finalize_control/finalize_control.hpp"

FinalizeControlNode::FinalizeControlNode() :
    rclcpp::Node("finalize_control")
{
    Global::setup_for_simulation(this);

    //: Callback group
    timer_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    msg_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto msg_group_option = rclcpp::SubscriptionOptions();
    msg_group_option.callback_group = msg_group;
    
    //: Create Subscriber
    input_control_SUB = this->create_subscription<ControlInterface>(
        Topic::CONTROL_REACTIVE,
        rclcpp::SensorDataQoS(),
        std::bind(&FinalizeControlNode::InputControlCallback, this, _1),
        msg_group_option
    );

    vehicle_status_SUB = this->create_subscription<VehicleStatus>(
        "/fmu/out/vehicle_status_v1",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&FinalizeControlNode::VehicleStatusCallback, this, _1),
        msg_group_option
    );

    vehicle_land_SUB = this->create_subscription<VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&FinalizeControlNode::VehicleLandedCallback, this, _1),
        msg_group_option
    );

    odometry_SUB = this->create_subscription<VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        rclcpp::SensorDataQoS(),
        std::bind(&FinalizeControlNode::OdometryCallback, this, _1),
        msg_group_option
    );

    //: Create Publisher
    vehicle_cmd_PUB = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    
    offboard_ctrl_PUB = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    
    attitude_set_PUB = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);

    trajectory_set_PUB = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

    //: Create timer
    node_loop_TIME = this->create_timer(
        std::chrono::nanoseconds(Clock::LOOP_CYCLE_FAST_NANOSEC),
        std::bind(&FinalizeControlNode::NodeLoopCallback, this),
        timer_group
    );

    //: Set variables
    latest_input_control = nullptr;
    input_control = latest_input_control;
    loss_final_control_count = 0;
    offboard_stream_counter = 0;
    offboard_attitude_counter = 0;
    arming_state = false;
    offboard_state = false;
    in_failure = false;
    landed = true;
    last_nav_state = VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    control_state = false;
    in_hover = false;
    latest_position_NED = {0.0f, 0.0f, 0.0f};
    position_NED = latest_position_NED;
    latest_quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
    quaternion = latest_quaternion;
    yaw_W = 0.0f;
    current_loop_state = NodeLoopState::INIT;
    just_change_loop_state = true;

    // Test
#if TIME_ANALYSE
    analyzer = std::make_unique<time_utils::TimeAnalyzer>(this->get_logger(), 800);
#endif
}

FinalizeControlNode::~FinalizeControlNode() {
    PublishVehicleCmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 5);
    SendDisarmCmd();
    PublishVehicleCmd(VehicleCommand::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1, 1);

#if TIME_ANALYSE
    analyzer->printSummary();
#endif
}

#pragma region Callbacks

void FinalizeControlNode::InputControlCallback(const ControlInterface::SharedPtr msg) {
    lock_guard lock(mx_control);
    latest_input_control = msg;
}

void FinalizeControlNode::VehicleStatusCallback(const VehicleStatus::SharedPtr msg) {
    arming_state = msg->arming_state == VehicleStatus::ARMING_STATE_ARMED;
    offboard_state = msg->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    last_nav_state = msg->nav_state;
    in_failure = msg->failure_detector_status != VehicleStatus::FAILURE_NONE;
    // RCLCPP_INFO(this->get_logger(), PINK "current nav_state = %d" RESET, msg->nav_state);
}

void FinalizeControlNode::VehicleLandedCallback(const VehicleLandDetected::SharedPtr msg) {
    landed = msg->landed;
}

void FinalizeControlNode::OdometryCallback(const VehicleOdometry::SharedPtr msg) {
    lock_guard lock(mx_spatial);
    if(!in_hover) latest_position_NED = msg->position;
    latest_quaternion = frame_utils::quaternionNEDtoENU(frame_utils::arrayToQuaternion(msg->q));
}

#pragma endregion

void FinalizeControlNode::NodeLoopCallback() {
#if TIME_ANALYSE
    analyzer->start_segment("Node loop");
#endif

    // Cache data
    {
        lock_guard lock(mx_control);
        input_control = latest_input_control;
    }
    {
        lock_guard lock(mx_spatial);
        position_NED = latest_position_NED;
        quaternion = latest_quaternion;
    }

    // Update drone data
    if(input_control != nullptr) {
        loss_final_control_count = 0;
        control_state = input_control->control_state;

        if(ALLOW_ATTITUDE && (abs(input_control->roll) > ATTITUDE_THRESHOLD || abs(input_control->pitch) > ATTITUDE_THRESHOLD)) {
            if(offboard_attitude_counter < OFFBOARD_MODE_CHANGE_THRESHOLD) offboard_attitude_counter++;
        }
        else {
            if(offboard_attitude_counter > 0) offboard_attitude_counter--;
        }

        if(offboard_attitude_counter >= OFFBOARD_MODE_CHANGE_THRESHOLD) {
            if(offboard_mode != OffboardMode::ATTITUDE) {
                RCLCPP_WARN(this->get_logger(), TEAL "Changed mode to ATTITUDE" RESET);
            }
            offboard_mode = OffboardMode::ATTITUDE;
        }
        else {
            if(offboard_mode != OffboardMode::VELOCITY && !in_hover) {
                RCLCPP_WARN(this->get_logger(), GREEN "Changed mode to VECLOCITY" RESET);
            }
            offboard_mode = OffboardMode::VELOCITY;
        }
    }
    else {
        if(loss_final_control_count <= LOSS_FINAL_CONTROL_THRESHOLD) loss_final_control_count++;
        offboard_mode = OffboardMode::VELOCITY;
        offboard_attitude_counter = 0;
    }

    yaw_W = frame_utils::quaternionToYaw(quaternion);

    // Main FSM loop
    switch(current_loop_state) {
        case NodeLoopState::INIT:
            if(just_change_loop_state) {
                offboard_stream_counter = 0;
                just_change_loop_state = false;
            }

            if(!control_state) {
                offboard_stream_counter = 0;
                current_loop_state = NodeLoopState::DISARM;
                just_change_loop_state = true;
            }
            else {
                SendOffboardCmd();
                offboard_stream_counter++;
            }

            if(offboard_stream_counter >= OFFBOARD_STREAM_THRESHOLD){
                current_loop_state = NodeLoopState::ARM;
                just_change_loop_state = true;
            }
        break;
        case NodeLoopState::ARM:
            if(just_change_loop_state) {
                just_change_loop_state = false;
            }
            
            if(!control_state) {
                offboard_stream_counter = 0;
                current_loop_state = NodeLoopState::DISARM;
                just_change_loop_state = true;
            }
            else if(arming_state) {
                current_loop_state = NodeLoopState::OFFBOARD;
                just_change_loop_state = true;
            }
            else {
                SendOffboardCmd();
                SendArmCmd();
            }
        break;
        case NodeLoopState::OFFBOARD:
            if(just_change_loop_state) {
                PublishVehicleCmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                RCLCPP_INFO(this->get_logger(), YELLOW "System try to enter offboard" RESET);
                just_change_loop_state = false;
                offboard_stream_counter = 0; // Repurposed to lost offboard counter
            }

            if(control_state) {
                SendOffboardCmd();

                if(!arming_state) { // Lost arm
                    current_loop_state = NodeLoopState::INIT;
                    just_change_loop_state = true;
                    RCLCPP_WARN(this->get_logger(), RED "System have been disarmed, rearm" RESET);
                }
                else if(last_nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                    if(offboard_stream_counter < Threshold::ALLOW_MISSED_FAST_TOPIC) offboard_stream_counter++;

                    if(offboard_stream_counter >= Threshold::ALLOW_MISSED_FAST_TOPIC) { // In Loiter, Return-to-launch
                        current_loop_state = NodeLoopState::INIT;
                        offboard_stream_counter = 0;
                        just_change_loop_state = true;
                        RCLCPP_INFO(this->get_logger(), RED "System have leave offboard, restart offboard" RESET);
                    }
                }
            }
            else {
                current_loop_state = NodeLoopState::DISARM;
                just_change_loop_state = true;
            }
        break;
        case NodeLoopState::LOST: // Not deal with yet
            current_loop_state = NodeLoopState::INIT;
            just_change_loop_state = true;
        break;
        case NodeLoopState::DISARM:
            if(just_change_loop_state) {
                just_change_loop_state = false;
            }

            PublishVehicleCmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 3);
            if(arming_state && last_nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
                SendDisarmCmd();
            }

            if(control_state){ // Signal arm
                current_loop_state = NodeLoopState::INIT;
                just_change_loop_state = true;
            }
        break;
        default:
            current_loop_state = NodeLoopState::INIT;
            just_change_loop_state = true;
        break;
    }

#if TIME_ANALYSE
    analyzer->stop_segment("Node loop");
#endif
}

#pragma region Methods

void FinalizeControlNode::SendOffboardCmd() {
    if(offboard_attitude_counter > 0) PublishAttitudeSetPoint();
    else if(offboard_attitude_counter < OFFBOARD_MODE_CHANGE_THRESHOLD) PublishTrajectorySetpoint();
    else PublishTrajectorySetpoint();
    
    PublishOffboardControlMode();
    
    if(loss_final_control_count >= LOSS_FINAL_CONTROL_THRESHOLD) input_control = nullptr;
}

void FinalizeControlNode::PublishOffboardControlMode() {
    auto msg = px4_msgs::msg::OffboardControlMode();
    if(in_hover) offboard_mode = OffboardMode::POSITION;
    switch(offboard_mode) {
        case OffboardMode::POSITION:
            msg.position = true;
            break;
        case OffboardMode::VELOCITY:
            msg.velocity = true;
            break;
        case OffboardMode::ACCELERATION:
            msg.acceleration =true;
            break;
        case OffboardMode::ATTITUDE:
            msg.attitude = true;
            break;
        case OffboardMode::RATE:
            msg.body_rate = true;
            break;
        case OffboardMode::THRUST:
            msg.thrust_and_torque = true;
            break;
        case OffboardMode::ACTUATOR:
            msg.direct_actuator = true;
            break;
    }

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_ctrl_PUB->publish(msg);
}

void FinalizeControlNode::PublishTrajectorySetpoint() {
    auto msg = TrajectorySetpoint();

    if(input_control != nullptr) {
        float vx = 0;
        if(input_control->forward > 0) vx = input_control->forward * Drone::SPEED_MAX_FORWARD;
        else vx = input_control->forward * Drone::SPEED_MAX_BACKWARD;
        float vy = input_control->left * Drone::SPEED_MAX_STRAFE;
        float vz = 0;
        if(input_control->up > 0) vz = input_control->up * Drone::SPEED_MAX_UP;
        else vz = input_control->up * Drone::SPEED_MAX_DOWN;

        // Hover, keep last positon to avoid drip
        if(vx == 0 && vy == 0 && vz == 0) {
            in_hover = true;
            msg.position = position_NED;
            msg.velocity = {0.0f, 0.0f, 0.0f};
        }
        else {
            in_hover = false;
            msg.position.fill(NO_DATA_f);
            msg.velocity = frame_utils::frameENUtoNED(frame_utils::frameFLUtoENU(vx, vy, vz, yaw_W));
        }
        msg.yaw = NO_DATA_f;
        msg.yawspeed = -input_control->yaw * Drone::SPEED_MAX_ANGLE; // frame FLU to FRD
    }
    else { // Hover, keep last position to avoid drip
        in_hover = true;
        msg.position = position_NED;
        msg.yaw = NO_DATA_f;
        msg.yawspeed = 0.0f;
    }

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_set_PUB->publish(msg);
    // RCLCPP_INFO(this->get_logger(), GREEN "Published Trajectory setpoint." RESET);
}

void FinalizeControlNode::PublishAttitudeSetPoint() {
    auto msg = VehicleAttitudeSetpoint();

    if(input_control != nullptr) {
        // Body velocity rate to world quaternion
        Eigen::Vector3f omega( // in velocity
            input_control->roll * Drone::SPEED_MAX_ANGLE * 8, // max 9 degree per fast cycle
            input_control->pitch * Drone::SPEED_MAX_ANGLE * 8,
            input_control->yaw * Drone::SPEED_MAX_ANGLE * 8
        );
        float angle = omega.norm() * Clock::LOOP_CYCLE_FAST;
        Eigen::Quaternionf dq;
        if(angle > 1e-6f) dq = Eigen::AngleAxisf(angle, omega.normalized());
        else dq = Eigen::Quaternionf::Identity();
        Eigen::Quaternionf q_new = quaternion * dq;
        q_new.normalize();
        
        // Compute thrust maintaining world hover
        //! This could be wrong
        float z_projection = 1.0f - 2.0f * (q_new.x() * q_new.x() + q_new.y() * q_new.y());
        z_projection = std::max(z_projection, 0.01f);
        float hover_thrust = Drone::HOVER_THRUST / z_projection;
        Eigen::Vector3f hover_body(0.0f, 0.0f, hover_thrust);
        
        Eigen::Vector3f move_body(
            0.0f, // Do not matter in acrobatic control for multicopter
            0.0f,  // Do not matter in acrobatic control for multicopter
            input_control->up // See as throttle in acrobatic control for multicopter
        );
        
        Eigen::Vector3f total_thurst = hover_body + move_body;
        total_thurst = total_thurst.cwiseMax(-Drone::THRUST_SAFE_LIMIT).cwiseMin(Drone::THRUST_SAFE_LIMIT);
        
        msg.q_d = frame_utils::quaternionToArray(frame_utils::quaternionENUtoNED(q_new));
        msg.thrust_body = frame_utils::frameFLUtoFRD(total_thurst); // Thrust x and y do nothing
        msg.yaw_sp_move_rate = -input_control->yaw * Drone::SPEED_MAX_ANGLE; // Frame FLU to FRD
    }
    else { // Hover still
        // Set body rate to flat, respect current yaw
        Eigen::Quaternionf flat_q = frame_utils::eulerToQuaternion(0.0f, 0.0f, yaw_W);

        // Hover thrust, respect current body rate
        Eigen::Vector3f hover_body(0.0f, 0.0f, Drone::HOVER_THRUST);

        msg.q_d = frame_utils::quaternionToArray(frame_utils::quaternionENUtoNED(flat_q));
        msg.thrust_body = frame_utils::frameFLUtoFRD(hover_body);
        msg.yaw_sp_move_rate = 0.0f;
    }

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    attitude_set_PUB->publish(msg);
}

void FinalizeControlNode::PublishVehicleCmd(uint16_t command, float param1, float param2, float param3) {
    auto msg = VehicleCommand();
	msg.param1 = param1;
	msg.param2 = param2;
    msg.param3 = param3;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_cmd_PUB->publish(msg);
}

bool FinalizeControlNode::SendArmCmd() {
    if(in_failure) return false;
    PublishVehicleCmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), BLUE "Arm command sended" RESET);
    return true;
}

bool FinalizeControlNode::SendDisarmCmd() {
    if(!landed) return false;
    PublishVehicleCmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), RED "Disarm command sended" RESET);
    return true;
}

bool FinalizeControlNode::SendForceDisarmCmd() {
    // Command: 400 (VEHICLE_CMD_COMPONENT_ARM_DISARM)
    // Param1: 0.0 (Disarm)
    // Param2: 21196.0 (Force Code)
    PublishVehicleCmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0, 0.0);
    RCLCPP_INFO(this->get_logger(), RED "Force Disarm command sended" RESET);
    return true;
}

#pragma endregion