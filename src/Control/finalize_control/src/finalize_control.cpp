#include "finalize_control/finalize_control.hpp"
#include <algorithm>

FinalizeControlNode::FinalizeControlNode() : rclcpp::Node("finalize_control") {
    using namespace std::chrono_literals;

    // Create Subscriber
    final_ctrl_SUB = this->create_subscription<ros2_msgs::msg::ControlInterface>(
        "control/final",
        5,
        std::bind(&FinalizeControlNode::FinalCtrlCallback, this, _1)
    );

    vehicle_status_SUB = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&FinalizeControlNode::VehicleStatusCallback, this, _1)
    );

    odo_SUB = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        rclcpp::SensorDataQoS(),
        std::bind(&FinalizeControlNode::OdometryCallback, this, _1)
    );

    // Create Publisher
    vehicle_cmd_PUB = this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/in/vehicle_command", 5);
    
    offboard_ctrl_PUB = this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/in/offboard_control_mode", 5);
    
    attitude_set_PUB = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 5);

    trajectory_set_PUB = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 5);

    // Create wall timer
    node_loop_TIME = this->create_wall_timer(
        std::chrono::nanoseconds(SYSTEM_LOOP_CYCLE_NANOSEC),
        std::bind(&FinalizeControlNode::NodeLoopCallback, this)
    );

    // Set variables
    arming_state = false;
    control_state = false;
    yaw_W = 0;
    offboard_stream_counter = 0;
    current_loop_state = NodeLoopState::INIT;
    just_change_loop_state = true;
}

FinalizeControlNode::~FinalizeControlNode() {}

/*########################################## Callbacks */

void FinalizeControlNode::FinalCtrlCallback(const ros2_msgs::msg::ControlInterface::SharedPtr msg) {
    last_final_ctrl = msg;
    control_state = msg->control_state;
}

void FinalizeControlNode::VehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    arming_state = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
}

void FinalizeControlNode::OdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    last_q = msg->q;
    last_yaw_W = frame_utils::quaternion_to_yaw(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
}

/*########################################## FSM */

void FinalizeControlNode::NodeLoopCallback() {
    switch(current_loop_state) {
        case NodeLoopState::INIT:
            if(just_change_loop_state) {
                offboard_stream_counter = 0;
                just_change_loop_state = false;
            }

            if(!control_state) {
                offboard_stream_counter = 0;
            }
            else {
                SendOffboardCmd();
                offboard_stream_counter++;
            }

            if(offboard_stream_counter >= 10){
                current_loop_state = NodeLoopState::ARM;
                just_change_loop_state = true;
            }
        break;
        case NodeLoopState::ARM:
            if(just_change_loop_state) {
                just_change_loop_state = false;
            }

            SendArmCmd();
            PublishVehicleCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            SendOffboardCmd();

            current_loop_state = NodeLoopState::OFFBOARD;
            just_change_loop_state = true;
        break;
        case NodeLoopState::OFFBOARD:
            if(just_change_loop_state) {
                just_change_loop_state = false;
            }

            if(!arming_state && control_state) {
                current_loop_state = NodeLoopState::ARM;
                just_change_loop_state = true;
            }
            if(!control_state) {
                current_loop_state = NodeLoopState::DISARM;
                just_change_loop_state = true;
            }
            
            SendOffboardCmd();
        break;
        case NodeLoopState::LOST: // Not deal with yet
            current_loop_state = NodeLoopState::INIT;
            just_change_loop_state = true;
        break;
        case NodeLoopState::DISARM:
            if(just_change_loop_state) {
                just_change_loop_state = false;
            }

            PublishVehicleCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4);
            SendDisarmCmd();

            current_loop_state = NodeLoopState::INIT;
            just_change_loop_state = true;
        break;
        default:
            current_loop_state = NodeLoopState::INIT;
            just_change_loop_state = true;
        break;
    }
}

/*########################################## Methods */

void FinalizeControlNode::SendOffboardCmd() {
    // Update drone data
    yaw_W = last_yaw_W;

    // Publish offboard command
    PublishOffboardControlMode();
    switch(offboard_mode) {
        case OffboardMode::POSITION:

        break;
        case OffboardMode::VELOCITY:
            PublishTrajectorySetpoint();
        break;
        case OffboardMode::ATTITUDE:
            PublishAttitudeSetPoint();
        break;
        default:

        break;
    }

}

void FinalizeControlNode::PublishTrajectorySetpoint() {
    auto msg = px4_msgs::msg::TrajectorySetpoint();

    if(last_final_ctrl != nullptr) {
        float vx = 0;
        if(last_final_ctrl->forward > 0) vx = last_final_ctrl->forward * SPEED_MAX_FORWARD_FW;
        else vx = last_final_ctrl->forward * SPEED_MAX_BACKWARD_FW;
        float vy = last_final_ctrl->right * SPEED_MAX_STRAFE;
        float vz = 0;
        if(last_final_ctrl->down > 0) vz = last_final_ctrl->down * SPEED_MAX_DOWN_FW;
        else vz = last_final_ctrl->down * SPEED_MAX_UP_FW;

        msg.position.fill(NO_DATA_f);
        msg.velocity = frame_utils::frame_FRD_to_NED(vx, vy, vz, yaw_W);
        msg.acceleration.fill(NO_DATA_f);
        msg.yaw = NO_DATA_f;
        msg.yawspeed = last_final_ctrl->yaw * M_PI_2f;
    }
    else {
        // No need to do anything cause trajectory setpoint auto stay still
    }

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1e3;
    trajectory_set_PUB->publish(msg);
    RCLCPP_INFO(this->get_logger(), GREEN "Published Trajectory setpoint." RESET);
}

void FinalizeControlNode::PublishAttitudeSetPoint() {
    auto msg = px4_msgs::msg::VehicleAttitudeSetpoint();

    if(last_final_ctrl != nullptr) {
        // Body rate velocity to Quaternion
        msg.q_d = frame_utils::euler_to_quaternion(last_final_ctrl->roll, last_final_ctrl->pitch, last_final_ctrl->yaw);

        // Body velocity to Thrust
        msg.thrust_body[0] = last_final_ctrl->forward;
        msg.thrust_body[1] = last_final_ctrl->right;
        msg.thrust_body[2] = last_final_ctrl->down;
    }
    else {
        msg.q_d = last_q;
        msg.thrust_body[2] = -0.5f;
    }

    msg.yaw_sp_move_rate = NO_DATA_f;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1e3;
    attitude_set_PUB->publish(msg);
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

void FinalizeControlNode::PublishVehicleCmd(uint16_t command, float param1, float param2) {
    auto msg = px4_msgs::msg::VehicleCommand();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1e3;
	vehicle_cmd_PUB->publish(msg);
}

bool FinalizeControlNode::SendArmCmd() {
    PublishVehicleCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), BLUE "Arm command send" RESET);
    return true;
}

bool FinalizeControlNode::SendDisarmCmd() {
    PublishVehicleCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), PINK "Disarm command send" RESET);
    return true;
}
