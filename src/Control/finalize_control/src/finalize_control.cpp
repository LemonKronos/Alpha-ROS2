#include "finalize_control/finalize_control.hpp"

#define ALLOW_ATTITUDE true

FinalizeControlNode::FinalizeControlNode() : rclcpp::Node("finalize_control") {
    using namespace std::chrono_literals;

    setup_for_simulation(this);
    
    // Create Subscriber
    final_ctrl_SUB = this->create_subscription<ros2_msgs::msg::ControlInterface>(
        CONTROL_REACTIVE_TOPIC,
        10,
        std::bind(&FinalizeControlNode::FinalCtrlCallback, this, _1)
    );

    vehicle_status_SUB = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&FinalizeControlNode::VehicleStatusCallback, this, _1)
    );

    fuse_perception_SUB = this->create_subscription<ros2_msgs::msg::FusePerception>(
        FUSE_PERCEPTION_TOPIC,
        rclcpp::SensorDataQoS(),
        std::bind(&FinalizeControlNode::FusePerceptionCallback, this, _1)
    );

    // Create Publisher
    vehicle_cmd_PUB = this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/in/vehicle_command", 10);
    
    offboard_ctrl_PUB = this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/in/offboard_control_mode", 10);
    
    attitude_set_PUB = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);

    trajectory_set_PUB = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

    // Create wall timer
    node_loop_TIME = this->create_timer(
        std::chrono::nanoseconds(SYSTEM_LOOP_CYCLE_FAST_NANOSEC),
        std::bind(&FinalizeControlNode::NodeLoopCallback, this)
    );

    // Set variables
    loss_final_control_count = 0;
    arming_state = false;
    control_state = false;
    yaw_W = 0;
    offboard_stream_counter = 0;
    current_loop_state = NodeLoopState::INIT;
    just_change_loop_state = true;
}

FinalizeControlNode::~FinalizeControlNode() {
    PublishVehicleCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 5);
    SendDisarmCmd();
    PublishVehicleCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1, 1);
}

/*########################################## Callbacks */

void FinalizeControlNode::FinalCtrlCallback(const ros2_msgs::msg::ControlInterface::SharedPtr msg) {
    last_final_ctrl = msg;
}

void FinalizeControlNode::VehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    arming_state = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    offboard_state = msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    last_nav_state = msg->nav_state;
    // RCLCPP_INFO(this->get_logger(), PINK "current nav_state = %d" RESET, msg->nav_state);
}

void FinalizeControlNode::FusePerceptionCallback(const ros2_msgs::msg::FusePerception::SharedPtr msg) {
    last_q = frame_utils::arrayToQuaternion(msg->q);
    odo_z_velocity = msg->velocity[2];
}

/*########################################## FSM */

void FinalizeControlNode::NodeLoopCallback() {
    // Update drone data
    if(last_final_ctrl != nullptr) {
        loss_final_control_count = 0;
        control_state = last_final_ctrl->control_state;
        if(!ALLOW_ATTITUDE || (abs(last_final_ctrl->roll) <= ATTITUDE_THRESHOLD && abs(last_final_ctrl->pitch) <= ATTITUDE_THRESHOLD)) {
            if(offboard_mode != OffboardMode::VELOCITY) {
                RCLCPP_WARN(this->get_logger(), GREEN "Change mode to VECLOCITY" RESET);
            }
            offboard_mode = OffboardMode::VELOCITY;
        }
        else {
            if(offboard_mode != OffboardMode::ATTITUDE) {
                RCLCPP_WARN(this->get_logger(), TEAL "Change mode to ATTITUDE" RESET);
            }
            offboard_mode = OffboardMode::ATTITUDE;
        }
    }
    else if(loss_final_control_count <= LOSS_FINAL_CONTROL_THRESHOLD) loss_final_control_count++;
    yaw_W = frame_utils::quaternionToYaw(last_q);

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

            if(offboard_stream_counter >= 20){
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
                PublishVehicleCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                RCLCPP_INFO(this->get_logger(), YELLOW "System have entered offboard" RESET);
                just_change_loop_state = false;
            }

            if(control_state) {
                if(!arming_state) { // Lost arm
                    current_loop_state = NodeLoopState::INIT;
                    just_change_loop_state = true;
                    RCLCPP_INFO(this->get_logger(), RED "System have been disarm, rearm" RESET);
                }
                else if(
                    last_nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL ||
                    last_nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL
                ) { // In RTL
                    current_loop_state = NodeLoopState::INIT;
                    just_change_loop_state = true;
                    RCLCPP_INFO(this->get_logger(), RED "System have leave offboard, restart offboard" RESET);
                }
                else SendOffboardCmd();
            }
            else {
                current_loop_state = NodeLoopState::DISARM;
                just_change_loop_state = true;
                RCLCPP_INFO(this->get_logger(), RED "End of control, disarm" RESET);
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

            PublishVehicleCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 3);
            if(arming_state && last_nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
                SendDisarmCmd();
            }

            if(control_state || !arming_state){
                current_loop_state = NodeLoopState::INIT;
                just_change_loop_state = true;
            }
        break;
        default:
            current_loop_state = NodeLoopState::INIT;
            just_change_loop_state = true;
        break;
    }
}

/*########################################## Methods */

void FinalizeControlNode::SendOffboardCmd() {
    PublishOffboardControlMode();
    switch(offboard_mode) {
        case OffboardMode::VELOCITY:
            PublishTrajectorySetpoint();
        break;
        case OffboardMode::ATTITUDE:
            PublishAttitudeSetPoint();
        break;
        default:
            PublishTrajectorySetpoint();
        break;
    }
    if(loss_final_control_count >= LOSS_FINAL_CONTROL_THRESHOLD) last_final_ctrl = nullptr;
}

void FinalizeControlNode::PublishOffboardControlMode() {
    auto msg = px4_msgs::msg::OffboardControlMode();
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
    auto msg = px4_msgs::msg::TrajectorySetpoint();

    if(last_final_ctrl != nullptr) {
        float vx = 0;
        if(last_final_ctrl->forward > 0) vx = last_final_ctrl->forward * SPEED_MAX_FORWARD;
        else vx = last_final_ctrl->forward * SPEED_MAX_BACKWARD;
        float vy = last_final_ctrl->left * SPEED_MAX_STRAFE;
        float vz = 0;
        if(last_final_ctrl->up > 0) vz = last_final_ctrl->up * SPEED_MAX_UP;
        else vz = last_final_ctrl->up * SPEED_MAX_DOWN;

        msg.position.fill(NO_DATA_f);
        msg.velocity = frame_utils::frameENUtoNED(frame_utils::frameFLUtoENU(vx, vy, vz, yaw_W));
        msg.yaw = NO_DATA_f;
        msg.yawspeed = -last_final_ctrl->yaw * M_PI_2f; // frame FLU to FRD
    }
    else {
        msg.position.fill(NO_DATA_f);
        msg.velocity = {0.0f, 0.0f, 0.0f};
        msg.yaw = NO_DATA_f;
        msg.yawspeed = 0.0f;
    }

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_set_PUB->publish(msg);
    // RCLCPP_INFO(this->get_logger(), GREEN "Published Trajectory setpoint." RESET);
}

void FinalizeControlNode::PublishAttitudeSetPoint() {
    auto msg = px4_msgs::msg::VehicleAttitudeSetpoint();

    if(last_final_ctrl != nullptr) { // Update new rate and thrust
        // Body rate to world quaternion
        Eigen::Vector3f omega(
            last_final_ctrl->roll * 2*M_PIf,
            -last_final_ctrl->pitch * 2*M_PIf, // No idea why!
            last_final_ctrl->yaw * 2*M_PIf
        );
        float angle = omega.norm() * SYSTEM_LOOP_CYCLE;
        Eigen::Quaternionf dq;
        if(angle > 1e-6f) dq = Eigen::AngleAxisf(angle, omega.normalized());
        else dq = Eigen::Quaternionf::Identity();
        Eigen::Quaternionf q_new = last_q * dq;
        q_new.normalize();
        
        // Compute thrust maintaining world hover
        Eigen::Vector3f euler = q_new.toRotationMatrix().eulerAngles(0, 1, 2);
        float roll  = euler.x();
        float pitch = euler.y();
        float hover_thrust = HOVER_THRUST / (cosf(roll) * cosf(pitch));
        Eigen::Vector3f hover_body(0.0f, 0.0f, hover_thrust);
        
        Eigen::Vector3f move_body(
            last_final_ctrl->forward, // Do not matter in acrobatic control for multicopter
            last_final_ctrl->left,  // Do not matter in acrobatic control for multicopter
            last_final_ctrl->up // See as throttle in acrobatic control for multicopter
        );
        
        Eigen::Vector3f total_thurst = hover_body + move_body;
        total_thurst = total_thurst.cwiseMax(-THRUST_SAFE_LIMIT).cwiseMin(THRUST_SAFE_LIMIT);
        
        msg.q_d = frame_utils::quaternionToArray(frame_utils::quaternionENUtoNED(q_new));
        msg.thrust_body = frame_utils::frameFLUtoFRD(total_thurst); // Thrust x and y do nothing
        msg.yaw_sp_move_rate = -last_final_ctrl->yaw; // FLU to FRD
    }
    else { // Hover still
        // Set body rate to flat, respect current yaw
        Eigen::Quaternionf flat_q = frame_utils::eulerToQuaternion(0.0f, 0.0f, yaw_W);

        // Hover thrust, respect current body rate
        Eigen::Vector3f hover_body(0.0f, 0.0f, HOVER_THRUST);

        msg.q_d = frame_utils::quaternionToArray(frame_utils::quaternionENUtoNED(flat_q));
        msg.thrust_body = frame_utils::frameFLUtoFRD(hover_body);
        msg.yaw_sp_move_rate = 0.0f;
    }

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    attitude_set_PUB->publish(msg);
}

void FinalizeControlNode::PublishVehicleCmd(uint16_t command, float param1, float param2, float param3) {
    auto msg = px4_msgs::msg::VehicleCommand();
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
    PublishVehicleCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), BLUE "Arm command send" RESET);
    return true;
}

bool FinalizeControlNode::SendDisarmCmd() {
    // Command: 400 (VEHICLE_CMD_COMPONENT_ARM_DISARM)
    // Param1: 0.0 (Disarm)
    // Param2: 21196.0 (The Magic Force Number)
    // PublishVehicleCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0, 0.0);
    // RCLCPP_INFO(this->get_logger(), RED "FORCE DISARM SENT (Magic Number)" RESET);
    PublishVehicleCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), RED "Disarm command send" RESET);
    return true;
}
