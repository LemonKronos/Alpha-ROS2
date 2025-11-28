#include "record_acrobatic/record_acrobatic.hpp"

using namespace std::chrono_literals;

RecordAcrobaticNode::RecordAcrobaticNode() 
    : Node("record_acrobatic_node"), 
      recording_(false), 
      pausing_(false),
      frame_count_(0),
      dim_input_(640, 480),
      dim_overview_(1280, 720)
{
    setup_for_simulation(this);

    // Path Config
    dataset_root_ = "/home/mr_lemon/MyCode/Project/Drone/AIBrain/datasets/acrobatic_oa_dataset/simulated_expert";
    fs::create_directories(dataset_root_);

    // Init Action buffer
    current_action_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // QoS
    auto qos_sensor = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // Subs
    sub_record_control_ = this->create_subscription<ros2_msgs::msg::RecordControl>(
        "logger/record_control", qos_reliable, 
        std::bind(&RecordAcrobaticNode::record_control_callback, this, std::placeholders::_1));

    sub_expert_action_ = this->create_subscription<ros2_msgs::msg::ControlInterface>(
        "control/input", qos_reliable,
        std::bind(&RecordAcrobaticNode::expert_action_callback, this, std::placeholders::_1));

    sub_perception_ = this->create_subscription<ros2_msgs::msg::FusePerception>(
        "/on_drone/sensor/fuse_perception", qos_sensor,
        std::bind(&RecordAcrobaticNode::perception_callback, this, std::placeholders::_1));

    sub_lidar_close_ = this->create_subscription<ros2_msgs::msg::Lidar2dObstacle>(
        "/on_drone/sensor/scan/lidar2d/close", qos_sensor,
        std::bind(&RecordAcrobaticNode::lidar_close_callback, this, std::placeholders::_1));

    sub_lidar_far_ = this->create_subscription<ros2_msgs::msg::Lidar2dObstacle>(
        "/on_drone/sensor/scan/lidar2d/far", qos_sensor,
        std::bind(&RecordAcrobaticNode::lidar_far_callback, this, std::placeholders::_1));

    // Service Client
    client_gz_ = this->create_client<ros_gz_interfaces::srv::ControlWorld>("/world/grasslands/control");

    // Timer
    timer_ = this->create_timer(
        std::chrono::duration<double>(SYSTEM_CYCLE), 
        std::bind(&RecordAcrobaticNode::node_loop_callback, this));
}

RecordAcrobaticNode::~RecordAcrobaticNode() {
    stop_image_subs();
}

// ------------------ Loop ------------------
void RecordAcrobaticNode::node_loop_callback() {
    if (!recording_ || pausing_) return;

    // --- NEW: THE GATEKEEPER ---
    // If we don't have an overview image yet, DO NOT record anything.
    // This "trims" the start of the episode until the camera is ready.
    if (!cache_img_overview_) {
        // Optional: Warn if waiting too long
        // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for Camera...");
        return; 
    }
    // ----------------------------

    double now = this->now().seconds();
    std::vector<float> vec;
    vec.reserve(40); 

    // 1. Perception (16 floats)
    if (is_alive(cache_perc_, now)) {
        auto& m = cache_perc_.msg;
        vec.insert(vec.end(), m->position.begin(), m->position.end());
        vec.insert(vec.end(), m->q.begin(), m->q.end());
        vec.insert(vec.end(), m->velocity.begin(), m->velocity.end());
        vec.insert(vec.end(), m->angular_velocity.begin(), m->angular_velocity.end());
        vec.push_back(m->bearable_contact ? 1.0f : 0.0f);
        vec.push_back(m->critical_contact ? 1.0f : 0.0f);
        vec.push_back(m->below_distance);
    } else {
        vec.insert(vec.end(), 16, 0.0f);
    }

    // 2. Lidar Close
    if (is_alive(cache_lidar_close_, now)) {
        auto sectors = extract_sectors(cache_lidar_close_.msg);
        vec.insert(vec.end(), sectors.begin(), sectors.end());
    } else {
        vec.insert(vec.end(), LIDAR_SECTORS, LIDAR_MAX_DIST);
    }

    // 3. Lidar Far
    if (is_alive(cache_lidar_far_, now)) {
        auto sectors = extract_sectors(cache_lidar_far_.msg);
        vec.insert(vec.end(), sectors.begin(), sectors.end());
    } else {
        vec.insert(vec.end(), LIDAR_SECTORS, LIDAR_MAX_DIST);
    }

    // 4. Append to Buffer (RAM)
    state_buffer_.insert(state_buffer_.end(), vec.begin(), vec.end());
    action_buffer_.insert(action_buffer_.end(), current_action_.begin(), current_action_.end());
    frame_count_++;

    // 5. Write Video (We KNOW cache_img_overview_ exists because of the check above)
    if (vw_overview_.isOpened()) 
        vw_overview_.write(cache_img_overview_->image);
    
    // Write others if they exist (best effort sync)
    if (vw_rgb_.isOpened() && cache_img_rgb_) 
        vw_rgb_.write(cache_img_rgb_->image);
    
    if (vw_depth_.isOpened() && cache_img_depth_) 
        vw_depth_.write(cache_img_depth_->image);
}

// ------------------ Helpers ------------------
bool RecordAcrobaticNode::is_alive(const DataCache& cache, double now) {
    if (!cache.valid) return false;
    return (now - cache.timestamp) <= (SYSTEM_CYCLE * TIMEOUT_CYCLES);
}

std::vector<float> RecordAcrobaticNode::extract_sectors(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr& msg) {
    std::vector<float> sectors(LIDAR_SECTORS, LIDAR_MAX_DIST);
    for (const auto& sec : msg->obstacles) {
        if (sec.sector_index < LIDAR_SECTORS) {
            sectors[sec.sector_index] = sec.min_distance;
        }
    }
    return sectors;
}

std::vector<float> RecordAcrobaticNode::msg_to_action_list(const ros2_msgs::msg::ControlInterface::SharedPtr& msg) {
    return {
        msg->roll, msg->pitch, msg->yaw,
        msg->forward, msg->left, msg->up,
        static_cast<float>(msg->wings_mode)
    };
}

// ------------------ Dynamic Subscription ------------------
void RecordAcrobaticNode::start_image_subs() {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    
    sub_img_rgb_ = this->create_subscription<sensor_msgs::msg::Image>(
        "sensor/rgb_cam/camera/image", qos,
        std::bind(&RecordAcrobaticNode::img_rgb_callback, this, std::placeholders::_1));
        
    sub_img_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
        "sensor/depth_cam/camera/image", qos,
        std::bind(&RecordAcrobaticNode::img_depth_callback, this, std::placeholders::_1));

    sub_img_overview_ = this->create_subscription<sensor_msgs::msg::Image>(
        "sensor/overview_cam/camera/image", qos,
        std::bind(&RecordAcrobaticNode::img_overview_callback, this, std::placeholders::_1));
}

void RecordAcrobaticNode::stop_image_subs() {
    sub_img_rgb_.reset();
    sub_img_depth_.reset();
    sub_img_overview_.reset();
    // Clear caches to prevent stale frames
    cache_img_rgb_.reset();
    cache_img_depth_.reset();
    cache_img_overview_.reset();
}

// ------------------ Episode Mgmt ------------------
void RecordAcrobaticNode::start_episode() {
    // 1. Meta Logic
    fs::path root_meta_path = fs::path(dataset_root_) / "meta.json";
    int episode_idx = 0;

    if (fs::exists(root_meta_path)) {
        try {
            std::ifstream f(root_meta_path);
            json j = json::parse(f);
            episode_idx = j.value("number_of_episode", 0);
        } catch (...) {}
    }

    char idx_str[16];
    snprintf(idx_str, sizeof(idx_str), "episode_%03d", episode_idx);
    episode_dir_ = fs::path(dataset_root_) / idx_str;
    fs::create_directories(episode_dir_);

    // Update root meta
    {
        json j;
        j["number_of_episode"] = episode_idx + 1;
        std::ofstream o(root_meta_path);
        o << j.dump(2);
    }

    // 2. Clear Buffers
    state_buffer_.clear();
    action_buffer_.clear();
    frame_count_ = 0;
    cache_img_overview_.reset(); 
    cache_img_rgb_.reset();
    cache_img_depth_.reset();

    // 3. Init Video
    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    vw_rgb_.open((episode_dir_ / "rgb_front.mp4").string(), fourcc, SYSTEM_RATE, dim_input_, true);
    vw_depth_.open((episode_dir_ / "depth_front.mp4").string(), fourcc, SYSTEM_RATE, dim_input_, true);
    vw_overview_.open((episode_dir_ / "overview.mp4").string(), fourcc, SYSTEM_RATE, dim_overview_, true);

    // 4. Start
    start_image_subs();
    // Short wait for first frame
    std::this_thread::sleep_for(200ms);
    
    start_timestamp_ = this->now().seconds();
    recording_ = true;
    RCLCPP_INFO(this->get_logger(), GREEN "REC: Started %s" RESET, idx_str);
}

void RecordAcrobaticNode::finish_episode() {
    recording_ = false;
    double end_ts = this->now().seconds();
    stop_image_subs();

    // Close Video
    if(vw_rgb_.isOpened()) vw_rgb_.release();
    if(vw_depth_.isOpened()) vw_depth_.release();
    if(vw_overview_.isOpened()) vw_overview_.release();

    // Save Numpy
    // Note: cnpy saves as flat array with shape info
    cnpy::npy_save((episode_dir_ / "state.npy").string(), &state_buffer_[0], {frame_count_, 40}, "w");
    cnpy::npy_save((episode_dir_ / "action.npy").string(), &action_buffer_[0], {frame_count_, 7}, "w");

    // Save Meta
    json meta;
    meta["expert_manuever"] = "unknown";
    meta["fps"] = SYSTEM_RATE;
    meta["timestamp_start"] = start_timestamp_;
    meta["timestamp_end"] = end_ts;
    meta["duration"] = end_ts - start_timestamp_;
    meta["frames"] = frame_count_;
    meta["state_dim"] = 40;
    meta["action_dim"] = 7;
    
    std::ofstream o(episode_dir_ / "meta.json");
    o << meta.dump(2);

    RCLCPP_INFO(this->get_logger(), GREEN "REC: Saved. Frames: %lu" RESET, frame_count_);
}

void RecordAcrobaticNode::start_pause() {
    auto req = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
    req->world_control.pause = true;
    client_gz_->async_send_request(req);
    pausing_ = true;
    RCLCPP_INFO(this->get_logger(), YELLOW "Simulation Paused" RESET);
}

void RecordAcrobaticNode::stop_pause() {
    auto req = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
    req->world_control.pause = false;
    client_gz_->async_send_request(req);
    pausing_ = false;
    RCLCPP_INFO(this->get_logger(), YELLOW "Simulation Continued" RESET);
}

// ------------------ Callbacks ------------------
void RecordAcrobaticNode::record_control_callback(const ros2_msgs::msg::RecordControl::SharedPtr msg) {
    if (msg->record && !recording_) start_episode();
    else if (!msg->record && recording_) finish_episode();

    if (msg->pause && !pausing_) start_pause();
    else if (!msg->pause && pausing_) stop_pause();
}

void RecordAcrobaticNode::expert_action_callback(const ros2_msgs::msg::ControlInterface::SharedPtr msg) {
    current_action_ = msg_to_action_list(msg);
}

void RecordAcrobaticNode::perception_callback(const ros2_msgs::msg::FusePerception::SharedPtr msg) {
    cache_perc_.msg = msg;
    cache_perc_.timestamp = this->now().seconds();
    cache_perc_.valid = true;
}

void RecordAcrobaticNode::lidar_close_callback(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg) {
    cache_lidar_close_.msg = msg;
    cache_lidar_close_.timestamp = this->now().seconds();
    cache_lidar_close_.valid = true;
}

void RecordAcrobaticNode::lidar_far_callback(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg) {
    cache_lidar_far_.msg = msg;
    cache_lidar_far_.timestamp = this->now().seconds();
    cache_lidar_far_.valid = true;
}

void RecordAcrobaticNode::img_rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try { cache_img_rgb_ = cv_bridge::toCvCopy(msg, "bgr8"); } catch(...) {}
}

void RecordAcrobaticNode::img_depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv_bridge::CvImagePtr raw = cv_bridge::toCvCopy(msg, "32FC1");
        cv::Mat img_float = raw->image;

        float max_dist = 30.0f; 

        cv::patchNaNs(img_float, max_dist);
        
        cv::threshold(img_float, img_float, max_dist, max_dist, cv::THRESH_TRUNC);

        cv::Mat norm;
        double scale_factor = 255.0 / max_dist; 
        img_float.convertTo(norm, CV_8U, scale_factor);

        cv::Mat color_depth;
        cv::cvtColor(norm, color_depth, cv::COLOR_GRAY2BGR);
        
        cache_img_depth_ = std::make_shared<cv_bridge::CvImage>();
        cache_img_depth_->header = msg->header;
        cache_img_depth_->encoding = "bgr8";
        cache_img_depth_->image = color_depth;

    } catch(const std::exception& e) {
        static bool logged = false;
        if (!logged) {
            RCLCPP_WARN(this->get_logger(), RED "Depth error: %s" RESET, e.what());
            logged = true;
        }
    }
}

void RecordAcrobaticNode::img_overview_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try { cache_img_overview_ = cv_bridge::toCvCopy(msg, "bgr8"); } catch(...) {}
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RecordAcrobaticNode>());
    rclcpp::shutdown();
    return 0;
}