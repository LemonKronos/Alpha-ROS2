#include "px4_emulator/px4_emulator.hpp"

using std::placeholders::_1;

using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::ActuatorMotors;
using px4_msgs::msg::ActuatorServos;
using actuator_msgs::msg::Actuators;
using std_msgs::msg::Float64;

Px4EmulatorNode::Px4EmulatorNode() : rclcpp::Node("px4_emulator") {
    Global::setup_for_simulation(this);

    //: Create Subscriber 
    this->SUB_actuator_motors = this->create_subscription<ActuatorMotors>(
        Topic::Px4::ACTUATOR_MOTORS,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&Px4EmulatorNode::ActuatorMotorsCallback, this, _1)
    );

    this->SUB_actuator_servos = this->create_subscription<ActuatorServos>(
        Topic::Px4::ACTUATOR_SERVOS,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&Px4EmulatorNode::ActuatorServosCallback, this, _1)
    );

    //: Create Publisher 
    this->PUB_vehicle_status = this->create_publisher<VehicleStatus>(
        Topic::Px4::VEHICLE_STATUS,
        rclcpp::SystemDefaultsQoS()
    );
    
    this->PUB_actuator_motors = this->create_publisher<Actuators>(
        Topic::DRONE_MOTORS,
        rclcpp::SystemDefaultsQoS()
    );
    
    this->PUB_actuator_servo_0 = this->create_publisher<Float64>(
        Topic::DRONE_ELEVON_LEFT,
        rclcpp::SystemDefaultsQoS()
    );

    this->PUB_actuator_servo_1 = this->create_publisher<Float64>(
        Topic::DRONE_ELEVON_RIGHT,
        rclcpp::SystemDefaultsQoS()
    );

    this->PUB_actuator_servo_2 = this->create_publisher<Float64>(
        Topic::DRONE_TILT_RIGHT,
        rclcpp::SystemDefaultsQoS()
    );

    this->PUB_actuator_servo_3 = this->create_publisher<Float64>(
        Topic::DRONE_TILT_LEFT,
        rclcpp::SystemDefaultsQoS()
    );

    //: Create timer
    this->TIME_response_loop = this->create_timer(
        std::chrono::nanoseconds(Clock::LOOP_CYCLE_SLOW_NANOSEC),
        std::bind(&Px4EmulatorNode::VehicleCommandCallback, this)
    );
}

Px4EmulatorNode::~Px4EmulatorNode() {

}

void Px4EmulatorNode::VehicleCommandCallback() {
    auto msg = VehicleStatus();
    msg.arming_state = VehicleStatus::ARMING_STATE_ARMED;
    msg.nav_state = VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    msg.failure_detector_status = VehicleStatus::FAILURE_NONE;

    PUB_vehicle_status->publish(msg);
}

void Px4EmulatorNode::ActuatorMotorsCallback(const ActuatorMotors::SharedPtr msg) {
    auto motors_msg = Actuators();
    motors_msg.header.stamp.sec = msg->timestamp / 1'000'000ll;
    motors_msg.header.stamp.nanosec = (msg->timestamp % 1'000'000ll) * 1'000ll;

    auto denorm = [](float c) -> double {
        if(!std::isfinite(c) || c < 0.0f) return 0.0;

        const double u = std::min(static_cast<double>(c), 1.0);
        return Drone::ROTOR_MIN + u * Drone::ROTOR_RANGE;
    };

    motors_msg.velocity.resize(4);
    for(size_t i = 0; i < 4; i++) {
        motors_msg.velocity[i] = denorm(msg->control[i]);
    }

    PUB_actuator_motors->publish(motors_msg);
    
    #if FLOW & 1
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 1000,
        GREEN "Motor control send [%.2f, %.2f, %.2f, %.2f]" RESET,
        motors_msg.velocity[0], motors_msg.velocity[1], motors_msg.velocity[2], motors_msg.velocity[3]
    );
    #endif
}

void Px4EmulatorNode::ActuatorServosCallback(const ActuatorServos::SharedPtr msg) {
    auto denorm_elevon = [](float e) -> double {
        if(!std::isfinite(e)) return 0.0;
        if(e > 1.0f) return Drone::ELEVON_MAX;
        if(e < -1.0f) return Drone::ELEVON_MIN;

        const double u = std::min((static_cast<double>(e) + 1.0) / 2.0, 1.0);
        return Drone::ELEVON_MIN + u * Drone::ELEVON_RANGE;
    };

    auto servo_0_msg = Float64();
    servo_0_msg.data = denorm_elevon(msg->control[0]);
    PUB_actuator_servo_0->publish(servo_0_msg);

    auto servo_1_msg = Float64();
    servo_1_msg.data = denorm_elevon(msg->control[1]);
    PUB_actuator_servo_1->publish(servo_1_msg);

    auto denorm_tilt = [](float t) -> double {
        if(!std::isfinite(t) || t < 0.0f) return Drone::TILT_MIN;

        const double u = std::min(static_cast<double>(t), 1.0);
        return Drone::TILT_MIN + u * Drone::TILT_RANGE;
    };

    auto servo_2_msg = Float64();
    servo_2_msg.data = denorm_tilt(msg->control[2]);
    PUB_actuator_servo_2->publish(servo_2_msg);

    auto servo_3_msg = Float64();
    servo_3_msg.data = denorm_tilt(msg->control[3]);
    PUB_actuator_servo_3->publish(servo_3_msg);

    #if FLOW & 1
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 1000,
        GREEN "Servos control send [%.2f, %.2f, %.2f, %.2f]" RESET,
        servo_0_msg.data,
        servo_1_msg.data,
        servo_2_msg.data,
        servo_3_msg.data
    );
    #endif
}