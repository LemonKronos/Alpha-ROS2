#include "record_acrobatic/add_nood_control.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>

AddNoobControlNode::AddNoobControlNode(int ep_num) 
    : Node("add_noob_control_node"), 
      frame_idx_(0), 
      state_("WAITING") 
{
    // 1. Locate Paths
    // Dynamic in system config
    
    char dir_name[32];
    snprintf(dir_name, sizeof(dir_name), "episode_%03d", ep_num);
    episode_path_ = fs::path(RECORD_ACROBATIC_DIR) / dir_name;

    if (!fs::exists(episode_path_)) {
        RCLCPP_ERROR(this->get_logger(), RED "Directory not found: %s" RESET, episode_path_.c_str());
        exit(1);
    }

    // 2. Load Metadata (to sync frame count)
    fs::path meta_path = episode_path_ / "meta.json";
    if (fs::exists(meta_path)) {
        std::ifstream f(meta_path);
        try {
            json j = json::parse(f);
            total_frames_ = j.value("frames", 0);
            fps_ = j.value("fps", 30);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), RED "JSON Parse Error: %s" RESET, e.what());
            exit(1);
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), RED "meta.json not found!" RESET);
        exit(1);
    }

    // 3. Open Video
    fs::path vid_path = episode_path_ / "overview.mp4";
    cap_.open(vid_path.string());
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), RED "Could not open overview.mp4" RESET);
        exit(1);
    }

    // 4. Init Input Buffer
    current_input_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // 5. Subscribers
    sub_input_ = this->create_subscription<ros2_msgs::msg::ControlInterface>(
        CONTROL_INPUT_TOPIC, 
        10,
        std::bind(&AddNoobControlNode::input_callback, this, std::placeholders::_1));

    sub_record_ = this->create_subscription<ros2_msgs::msg::RecordControl>(
        LOGGER_RECORD_TOPIC,
        10,
        std::bind(&AddNoobControlNode::record_control_callback, this, std::placeholders::_1));

    // 6. Timer (Syncs to Video FPS)
    double interval = 1.0 / static_cast<double>(fps_);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(interval),
        std::bind(&AddNoobControlNode::game_loop, this));

    RCLCPP_INFO(this->get_logger(), GREEN " Loaded Episode %d | Frames: %lu | FPS: %d" RESET, ep_num, total_frames_, fps_);
    RCLCPP_INFO(this->get_logger(), GREEN "Publish control topic to record" RESET);
}

AddNoobControlNode::~AddNoobControlNode() {
    cv::destroyAllWindows();
}

void AddNoobControlNode::input_callback(const ros2_msgs::msg::ControlInterface::SharedPtr msg) {
    current_input_ = {
        msg->roll, msg->pitch, msg->yaw,
        msg->forward, msg->left, msg->up,
        static_cast<float>(msg->wings_mode)
    };
}

void AddNoobControlNode::record_control_callback(const ros2_msgs::msg::RecordControl::SharedPtr msg) {
    // Start Trigger
    if (msg->record && state_ == "WAITING") {
        start_recording();
    }
    else if (!msg->record && state_ == "RECORDING") {
        discard_recording();
    }
    
    // Pause Trigger
    if (msg->pause && state_ == "RECORDING") {
        state_ = "PAUSED";
        RCLCPP_INFO(this->get_logger(), "PAUSED.");
    } 
    else if (!msg->pause && state_ == "PAUSED") {
        state_ = "RECORDING";
        RCLCPP_INFO(this->get_logger(), "RESUMED.");
    }
}

void AddNoobControlNode::start_recording() {
    state_ = "RECORDING";
    frame_idx_ = 0;
    noob_buffer_.clear();
    // Rewind video to start
    cap_.set(cv::CAP_PROP_POS_FRAMES, 0); 
    RCLCPP_INFO(this->get_logger(), YELLOW "START Recording..." RESET);
}

void AddNoobControlNode::game_loop() {
    // 1. Handle Pause State (Freeze Frame)
    if (state_ == "PAUSED") {
        if (!last_frame_.empty()) {
            cv::Mat disp = last_frame_.clone();
            cv::putText(disp, "PAUSED", cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 3);
            cv::imshow(WINDOW_OVERVIEW_FPV, disp);
            cv::waitKey(1);
        }
        return;
    }

    // 2. Read Next Frame
    cv::Mat frame;
    cap_ >> frame;

    // 3. Handle End of File
    if (frame.empty()) {
        if (state_ == "RECORDING") {
            // DETECT DESYNC: Video ended, but we still expect more data frames
            if (frame_idx_ < total_frames_) {
                size_t missing = total_frames_ - frame_idx_;
                RCLCPP_WARN(this->get_logger(), PINK "Video ended early! Padding last input for %lu frames..." RESET, missing);
                
                // Fill the rest with the CURRENT input (hold position)
                while (frame_idx_ < total_frames_) {
                    noob_buffer_.insert(noob_buffer_.end(), current_input_.begin(), current_input_.end());
                    frame_idx_++;
                }
            }
            finish_recording();
            return;
        } else {
            // Waiting mode loop
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            return;
        }
    }

    // Cache for pause logic
    last_frame_ = frame.clone();

    // 4. UI Overlay
    std::string txt = "State: " + state_ + " | Frame: " + std::to_string(frame_idx_) + "/" + std::to_string(total_frames_);
    cv::Scalar color = (state_ == "RECORDING") ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
    cv::putText(frame, txt, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);

    cv::imshow(WINDOW_OVERVIEW_FPV, frame);
    cv::waitKey(1); // 1ms wait allows OpenCV to draw

    // 5. Logic
    if (state_ == "RECORDING") {
        // Capture Input
        noob_buffer_.insert(noob_buffer_.end(), current_input_.begin(), current_input_.end());
        frame_idx_++;

        // Stop Condition
        if (frame_idx_ >= total_frames_) {
            finish_recording();
        }
    }
}

void AddNoobControlNode::finish_recording() {
    state_ = "SAVING";

    // Validate Data
    size_t recorded_rows = noob_buffer_.size() / 7;
    if (recorded_rows != total_frames_) {
        RCLCPP_WARN(this->get_logger(), PINK "Mismatch! Expected %lu frames, got %lu." RESET, total_frames_, recorded_rows);
    }

    // Find next filename: noob_01.npy, noob_02.npy ...
    int file_idx = 1;
    while (true) {
        char fname[32];
        snprintf(fname, sizeof(fname), "noob_%02d.npy", file_idx);
        fs::path p = episode_path_ / fname;
        
        if (!fs::exists(p)) {
            // Save using cnpy
            // Shape is {rows, 7 columns}
            cnpy::npy_save(p.string(), &noob_buffer_[0], {recorded_rows, 7}, "w");
            RCLCPP_INFO(this->get_logger(), GREEN "SAVED NOOB CONTROL: %s" RESET, fname);
            break;
        }
        file_idx++;
    }

    // Back to waiting
    noob_buffer_.clear();
    frame_idx_ = 0;
    state_ = "WAITING";
}

void AddNoobControlNode::discard_recording() {
    state_ = "DISCARDING";
    RCLCPP_WARN(this->get_logger(), PINK "DISCARD Current record, waiting..." RESET);
    noob_buffer_.clear();
    state_ = "WAITING";
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    int ep = 0;
    // Simple Argument Parsing: ros2 run ... <number>
    if (argc > 1) {
        try {
            ep = std::stoi(argv[1]);
        } catch (...) {
            std::cerr << RED << "[Error] Invalid Episode Number. Usage: ros2 run <pkg> <node> <number>" << RESET << std::endl;
            return 1;
        }
    } else {
        std::cerr << RED << "[Warn] No episode number provided. Exit." << RESET << std::endl;
        return 1;
    }

    auto node = std::make_shared<AddNoobControlNode>(ep);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}