#include "simulation_control/simulation_control.hpp"

using namespace std::chrono_literals;

SimulationControlNode::SimulationControlNode() 
    : Node("simulation_control"), 
      is_paused_(false),
      new_frame_available_(false)
{
    // 1. Setup Window
    cv::namedWindow(WINDOW_OVERVIEW_FPV, cv::WINDOW_AUTOSIZE);

    // 2. Image Subscription (Best Effort)
    sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(
        Topic::OVERVIEW_CAM, 
        rclcpp::SensorDataQoS(),
        std::bind(&SimulationControlNode::img_callback, this, std::placeholders::_1));

    // 3. Record Control Subscription (Reliable) --- NEW ---
    sub_record_control_ = this->create_subscription<ros2_msgs::msg::RecordControl>(
        Topic::LOGGER_RECORD, 
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), 
        std::bind(&SimulationControlNode::record_control_callback, this, std::placeholders::_1));

    // 4. World Control Client
    client_gz_ = this->create_client<ros_gz_interfaces::srv::ControlWorld>(Service::CONTROL_WORLD_NAME);

    // 5. Wall Timer Real
    timer_ = this->create_wall_timer(
        16ms, 
        std::bind(&SimulationControlNode::node_loop, this));

    RCLCPP_INFO(this->get_logger(), GREEN "Simulation Control Node Started. Listening to %s for Pause." RESET, Topic::LOGGER_RECORD);
}

SimulationControlNode::~SimulationControlNode() {
    cv::destroyWindow(WINDOW_OVERVIEW_FPV);
}

void SimulationControlNode::img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        std::lock_guard<std::mutex> lock(mtx_);
        current_frame_ = cv_ptr->image;
        new_frame_available_ = true;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

// --- NEW CALLBACK ---
void SimulationControlNode::record_control_callback(const ros2_msgs::msg::RecordControl::SharedPtr msg) {
    // Only act if the state actually changes
    if (msg->pause && !is_paused_) {
        // Command says PAUSE, we are running -> PAUSE
        is_paused_ = true;
        send_pause_command(true);
    } 
    else if (!msg->pause && is_paused_) {
        // Command says RESUME, we are paused -> RESUME
        is_paused_ = false;
        send_pause_command(false);
    }
}

void SimulationControlNode::node_loop() {
    // 1. Keep Window Alive (No Keyboard Logic)
    cv::waitKey(1); 

    // 2. Get Frame (Thread-safe)
    cv::Mat frame_to_show;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (new_frame_available_ && !current_frame_.empty()) {
            frame_to_show = current_frame_;
        }
    }

    // 3. Render
    if (!frame_to_show.empty()) {
        // Overlay logic: Shows "PAUSED" if is_paused_ is true
        if (is_paused_) {
            cv::putText(frame_to_show, "PAUSED", cv::Point(50, 50), 
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
        }
        
        cv::imshow(WINDOW_OVERVIEW_FPV, frame_to_show);
    }
}

void SimulationControlNode::send_pause_command(bool pause) {
    if (!client_gz_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "World control service not ready!");
        return;
    }

    auto req = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
    req->world_control.pause = pause;

    client_gz_->async_send_request(req);

    if (pause) {
        RCLCPP_INFO(this->get_logger(), "Command: PAUSE Simulation");
    } else {
        RCLCPP_INFO(this->get_logger(), "Command: RESUME Simulation");
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimulationControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}