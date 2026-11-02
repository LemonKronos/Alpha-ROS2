#include "finalize_control/finalize_control.hpp"

using std::placeholders::_1;
using std::mutex;
using std::lock_guard;
using std::atomic;

using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleOdometry;
// using px4_msgs::msg::VehicleCommandAck;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::VehicleAttitudeSetpoint;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::ActuatorMotors;
using px4_msgs::msg::ActuatorServos;
using alpha_msgs::msg::ControlInterface;
using alpha_msgs::msg::NormalizedMotorsServos;

FinalizeControlNode::FinalizeControlNode() :
    rclcpp::Node("finalize_control")
{
    Global::setup_for_simulation(this);

    //: Callback group
    control_signal_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    msg_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto msg_group_option = rclcpp::SubscriptionOptions();
    msg_group_option.callback_group = msg_callback_group;
    
    //: Create Subscriber
    SUB_raw_input_control = this->create_subscription<ControlInterface>(
        Topic::CONTROL_INPUT,
        rclcpp::SensorDataQoS(),
        std::bind(&FinalizeControlNode::RawInputControlCallback, this, _1),
        msg_group_option
    );

    SUB_reactive_control = this->create_subscription<ControlInterface>(
        Topic::CONTROL_REACTIVE,
        rclcpp::SensorDataQoS(),
        std::bind(&FinalizeControlNode::ReactiveControlCallback, this, _1),
        msg_group_option
    );

    SUB_acrobatic_control = this->create_subscription<NormalizedMotorsServos>(
        Topic::CONTROL_ACROBATIC,
        rclcpp::SensorDataQoS(),
        std::bind(&FinalizeControlNode::AcrobaticControlCallback, this, _1),
        msg_group_option
    );

    SUB_vehicle_status = this->create_subscription<VehicleStatus>(
        Topic::Px4::VEHICLE_STATUS,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&FinalizeControlNode::VehicleStatusCallback, this, _1),
        msg_group_option
    );

    SUB_vehicle_land = this->create_subscription<VehicleLandDetected>(
        Topic::Px4::VEHICLE_LAND_DETECTED,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&FinalizeControlNode::VehicleLandedCallback, this, _1),
        msg_group_option
    );

    SUB_odometry = this->create_subscription<VehicleOdometry>(
        Topic::Px4::VEHICLE_ODOMETRY,
        rclcpp::SensorDataQoS(),
        std::bind(&FinalizeControlNode::OdometryCallback, this, _1),
        msg_group_option
    );

    //: Create Publisher
    PUB_vehicle_cmd = this->create_publisher<VehicleCommand>(Topic::Px4::VEHICLE_COMMAND, rclcpp::SystemDefaultsQoS());
    PUB_offboard_ctrl = this->create_publisher<OffboardControlMode>(Topic::Px4::OFFBOARD_CONTROL_MODE, rclcpp::SystemDefaultsQoS());
    PUB_attitude_set = this->create_publisher<VehicleAttitudeSetpoint>(Topic::Px4::VEHICLE_ATTITUDE_SETPOINT, rclcpp::SystemDefaultsQoS());
    PUB_trajectory_set = this->create_publisher<TrajectorySetpoint>(Topic::Px4::TRAJECTORY_SETPOINT, rclcpp::SystemDefaultsQoS());
    PUB_motors = this->create_publisher<ActuatorMotors>(Topic::Px4::ACTUATOR_MOTORS, rclcpp::SystemDefaultsQoS());
    PUB_servos = this->create_publisher<ActuatorServos>(Topic::Px4::ACTUATOR_SERVOS, rclcpp::SystemDefaultsQoS());

    //: Create timer
    TIME_node_loop = this->create_timer(
        std::chrono::nanoseconds(Clock::LOOP_CYCLE_FAST_NANOSEC),
        std::bind(&FinalizeControlNode::NodeLoopCallback, this),
        control_signal_group
    );

    //: Set variables
    last_raw_input_control = nullptr;
    raw_input_control = nullptr;

    last_reactive_control = nullptr;
    reactive_control = nullptr;

    last_acrobatic_control = nullptr;
    acrobatic_control = nullptr;

    offboard_stream_counter = 0;
    offboard_attitude_counter = 0;

    final_control = nullptr;

    arming_state = false;
    offboard_state = false;
    in_failure = false;
    landed = true;
    last_nav_state = VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    control_state = false;
    in_hover = false;

    latest_position_NED = {0.0f, 0.0f, 0.0f};
    position_NED = latest_position_NED;
    latest_quaternion_NED = {1.0f, 0.0f, 0.0f, 0.0f};
    quaternion_ENU = {1.0f, 0.0f, 0.0f, 0.0f};
    heading_ENU = 0.0f;

    current_loop_state = NodeLoopState::INIT;
    just_change_loop_state = true;

    // Test
    #if TIME_ANALYSE
        analyzer = std::make_unique<time_utils::TimeAnalyzer>(this->get_logger(), 800);
    #endif
}

FinalizeControlNode::~FinalizeControlNode() {
    PublishVehicleCmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 5); // Return to Launch RTL
    SendDisarmCmd();
    PublishVehicleCmd(VehicleCommand::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1, 1);

#if TIME_ANALYSE
    analyzer->printSummary();
#endif
}

void FinalizeControlNode::RawInputControlCallback(const ControlInterface::SharedPtr msg) {
    lock_guard lock(mutex_raw_input);
    last_raw_input_control = msg;
}

void FinalizeControlNode::ReactiveControlCallback(const ControlInterface::SharedPtr msg) {
    lock_guard lock(mutex_reactive);
    last_reactive_control = msg;
}

void FinalizeControlNode::AcrobaticControlCallback(const NormalizedMotorsServos::SharedPtr msg) {
    lock_guard lock(mutex_acrobatic);
    last_acrobatic_control = msg;
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
    lock_guard lock(mutex_pose);
    latest_position_NED = msg->position;
    latest_quaternion_NED = frame_utils::arrayToQuaternion(msg->q);
}

//_ Node loop 
void FinalizeControlNode::NodeLoopCallback() {
    #if TIME_ANALYSE
        analyzer->start_segment("Node loop");
    #endif

    //: Cache data 
    {
        lock_guard lock(mutex_raw_input);
        raw_input_control = last_raw_input_control;
    }
    //TODO handle lost signal here 

    {
        lock_guard lock(mutex_reactive);
        reactive_control = last_reactive_control;
    }
    //TODO handle lost signal here 

    {
        lock_guard lock(mutex_acrobatic);
        acrobatic_control = last_acrobatic_control;
    }
    //TODO handle lost signal here 

    {
        lock_guard lock(mutex_pose);
        position_NED = latest_position_NED;
        heading_NED = frame_utils::quaternionToHeading(latest_quaternion_NED);
        quaternion_ENU = frame_utils::quaternionNEDtoENU(latest_quaternion_NED);
    }
    heading_ENU = frame_utils::quaternionToHeading(quaternion_ENU);

    //: Set Control Support mode 
    if(raw_input_control) {
        this->control_support = raw_input_control->control_support;
    } else if (reactive_control) {
        this->control_support = reactive_control->control_support;
    } // else keep last mode

    //: Set final control signal (meaningful for non-actuator control only) 
    if(this->control_support == ControlInterface::NONE || this->control_support == ControlInterface::AUTOPILOT) {
        this->final_control = raw_input_control;
    } else if(this->control_support == ControlInterface::REACTIVE_OA) {
        this->final_control = reactive_control;
    } else {
        this->final_control = nullptr;
    }

    //: Set Offboard mode 
    if(acrobatic_control && this->control_support == ControlInterface::ACROBATIC_OA) {
        if(offboard_mode != OffboardMode::ACTUATOR) {
            RCLCPP_WARN(this->get_logger(), PINK "Changed mode to ACTUATOR" RESET);
            
            offboard_mode = OffboardMode::ACTUATOR;
            control_state = true; // Always in ARM
        }
    }
    else if(final_control != nullptr) {
        control_state = final_control->control_state;

        if((abs(final_control->roll) > Threshold::ENTER_ATTIUDE_MODE || abs(final_control->pitch) > Threshold::ENTER_ATTIUDE_MODE)) {
            if(offboard_attitude_counter < Threshold::OFFBOARD_STREAM) offboard_attitude_counter++;
        }
        else {
            if(offboard_attitude_counter > 0) offboard_attitude_counter--;
        }

        if(offboard_attitude_counter >= Threshold::OFFBOARD_STREAM) {
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
        // Lost control put to hover
        offboard_mode = OffboardMode::VELOCITY;
        offboard_attitude_counter = 0;
    }

    //: Main FSM loop 
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

            if(offboard_stream_counter >= Threshold::OFFBOARD_STREAM){
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
                PublishVehicleCmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // Get into Offboard mode

                just_change_loop_state = false;
                offboard_stream_counter = 0; // Repurposed to lost offboard counter

                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), 
                    *this->get_clock(), 1000, 
                    YELLOW "System try to enter offboard" RESET
                );
            }

            if(control_state) {
                SendOffboardCmd();

                if(!arming_state) { // Lost arm
                    current_loop_state = NodeLoopState::INIT;
                    just_change_loop_state = true;

                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), 
                        *this->get_clock(), 1000, 
                        YELLOW "System have been disarmed, rearm" RESET
                    );
                }
                else if(last_nav_state != VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                    if(offboard_stream_counter < Threshold::ALLOW_MISSED_FAST_TOPIC) offboard_stream_counter++;

                    if(offboard_stream_counter >= Threshold::ALLOW_MISSED_FAST_TOPIC) { // In Loiter, Return-to-launch
                        current_loop_state = NodeLoopState::INIT;
                        offboard_stream_counter = 0;
                        just_change_loop_state = true;

                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(), 
                            *this->get_clock(), 1000, 
                            YELLOW "System have leave offboard, restart offboard" RESET
                        );
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

            PublishVehicleCmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 3); // Get into Hold/Loiter mdoe
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

void FinalizeControlNode::SendOffboardCmd() {    
    auto msg = OffboardControlMode();
    switch(offboard_mode) {
        case OffboardMode::POSITION:
            break;

        case OffboardMode::VELOCITY:
            msg.velocity = true;
            PublishTrajectorySetpoint();
            break;

        case OffboardMode::ATTITUDE:
            msg.attitude = true;
            PublishAttitudeSetPoint();
            break;

        case OffboardMode::RATE:
            break;

        case OffboardMode::THRUST:
            break;

        case OffboardMode::ACTUATOR:
            msg.direct_actuator = true;
            PublishActuatorMotors();
            PublishActuatorServos();
            break;
    }

    if(in_hover) { // Automatically change to position to avoid drifting
        msg.position = true;
        msg.velocity = false;
    }

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000ull;
    PUB_offboard_ctrl->publish(msg);
}

void FinalizeControlNode::PublishTrajectorySetpoint() {
    auto msg = TrajectorySetpoint();

    if(final_control != nullptr) {
        float vx = 0;
        if(final_control->forward > 0) vx = final_control->forward * Drone::SPEED_MAX_FORWARD;
        else vx = final_control->forward * Drone::SPEED_MAX_BACKWARD;
        
        float vy = final_control->left * Drone::SPEED_MAX_STRAFE;

        float vz = 0;
        if(final_control->up > 0) vz = final_control->up * Drone::SPEED_MAX_UP;
        else vz = final_control->up * Drone::SPEED_MAX_DOWN;

        if(vx == 0 && vy == 0 && vz == 0) { 
            // Hover, keep last positon to avoid drip
            in_hover = true;
            msg.position = position_NED;
            msg.velocity = {0.0f, 0.0f, 0.0f};
        }
        else {
            // Set velocity
            in_hover = false;
            msg.position.fill(NO_DATA_f);
            msg.velocity = frame_utils::frameENUtoNED(frame_utils::frameFLUtoENU(vx, vy, vz, heading_ENU));
        }

        if(final_control->yaw == 0) {
            // Keep heading, avoid drip
            msg.yaw = this->heading_NED;
            msg.yawspeed = NO_DATA_f;
        } else { 
            // Change heading according to yaw speed
            msg.yaw = NO_DATA_f;
            msg.yawspeed = -final_control->yaw * Drone::SPEED_MAX_ANGLE; // frame FLU to FRD
        }

        msg.timestamp =
            final_control->header.stamp.sec * 1'000'000ull + 
            final_control->header.stamp.nanosec / 1'000ull
        ;
    }
    else { // Hover, keep last position to avoid drip
        in_hover = true;
        msg.position = position_NED;
        msg.yaw = NO_DATA_f;
        msg.yawspeed = 0.0f;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1'000ull;
    }

    
    PUB_trajectory_set->publish(msg);
    
    #if FLOW && 0
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), 
        *this->get_clock(), 1000, 
        GREEN "Published Trajectory setpoint." RESET
    );
    #endif

}

void FinalizeControlNode::PublishAttitudeSetPoint() {
    auto msg = VehicleAttitudeSetpoint();

    if(final_control != nullptr) {
        // Body velocity rate to world quaternion
        Eigen::Vector3f omega( // in velocity
            final_control->roll * Drone::SPEED_MAX_ANGLE * 8, // max 9 degree per fast cycle
            final_control->pitch * Drone::SPEED_MAX_ANGLE * 8,
            final_control->yaw * Drone::SPEED_MAX_ANGLE * 8
        );
        float angle = omega.norm() * Clock::LOOP_CYCLE_FAST;
        Eigen::Quaternionf dq;
        if(angle > 1e-6f) dq = Eigen::AngleAxisf(angle, omega.normalized());
        else dq = Eigen::Quaternionf::Identity();
        Eigen::Quaternionf q_new = quaternion_ENU * dq;
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
            final_control->up // See as throttle in acrobatic control for multicopter
        );
        
        Eigen::Vector3f total_thurst = hover_body + move_body;
        total_thurst = total_thurst.cwiseMax(-Drone::THRUST_SAFE_LIMIT).cwiseMin(Drone::THRUST_SAFE_LIMIT);
        
        msg.q_d = frame_utils::quaternionToArray(frame_utils::quaternionENUtoNED(q_new));
        msg.thrust_body = frame_utils::frameFLUtoFRD(total_thurst); // Thrust x and y do nothing
        msg.yaw_sp_move_rate = -final_control->yaw * Drone::SPEED_MAX_ANGLE; // Frame FLU to FRD
        msg.timestamp =
            final_control->header.stamp.sec * 1'000'000ull + 
            final_control->header.stamp.nanosec / 1'000ull
        ;
    }
    else { // Hover still, but actually will not happen, will be cachec by hover
        // Set body rate to flat, respect current yaw
        Eigen::Quaternionf flat_q = frame_utils::eulerToQuaternion(0.0f, 0.0f, heading_ENU);

        // Hover thrust, respect current body rate
        Eigen::Vector3f hover_body(0.0f, 0.0f, Drone::HOVER_THRUST);

        msg.q_d = frame_utils::quaternionToArray(frame_utils::quaternionENUtoNED(flat_q));
        msg.thrust_body = frame_utils::frameFLUtoFRD(hover_body);
        msg.yaw_sp_move_rate = 0.0f;

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1'000ull;
    }

    
    PUB_attitude_set->publish(msg);

    #if FLOW && 0
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), 
        *this->get_clock(), 1000, 
        GREEN "Published Attitude setpoint." RESET
    );
    #endif
}

void FinalizeControlNode::PublishActuatorMotors() {
    auto msg = ActuatorMotors();
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1'000ull;
    msg.timestamp_sample = 
        acrobatic_control->header.stamp.sec * 1'000'000ull + 
        acrobatic_control->header.stamp.nanosec / 1'000ull
    ;
    msg.control = acrobatic_control->motors_control;

    PUB_motors->publish(msg);

    #if FLOW && 0
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), 
        *this->get_clock(), 1000, 
        GREEN "Published Actuator Motors." RESET
    );
    #endif
}

void FinalizeControlNode::PublishActuatorServos() {
    auto msg = ActuatorServos();
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1'000ull;
    msg.timestamp_sample = 
        acrobatic_control->header.stamp.sec * 1'000'000ull + 
        acrobatic_control->header.stamp.nanosec / 1'000ull
    ;
    msg.control = acrobatic_control->servos_control;

    PUB_servos->publish(msg);

    #if FLOW && 0
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), 
        *this->get_clock(), 1000, 
        GREEN "Published Actuator Servos." RESET
    );
    #endif
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
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000ull;
	PUB_vehicle_cmd->publish(msg);
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

