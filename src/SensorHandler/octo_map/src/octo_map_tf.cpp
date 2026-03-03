#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>

class OctoMapComponent : public rclcpp::Node {
public:
    OctoMapComponent(const rclcpp::NodeOptions & options) 
    : Node("octo_map_node", options) {
        // Initialize TF2 Buffer and Listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscriber to one of the cameras (e.g., front)
        sub_front_cam_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/alpha_depth_cam/front/points", 
            rclcpp::SensorDataQoS(), 
            std::bind(&OctoMapComponent::pointCloudCallback, this, std::placeholders::_1)
        );
    }

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_front_cam_;

    // Fast-clock callback (30 Hz)
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped tf_body;
        geometry_msgs::msg::TransformStamped tf_world;

        try {
            // 1. Get the transform from the Camera Frame to the Drone Body Frame
            // We use the exact timestamp of the image to avoid lag!
            tf_body = tf_buffer_->lookupTransform(
                "base_link",             // Target frame (Drone Body)
                msg->header.frame_id,    // Source frame (e.g., alpha_front_cam_link)
                msg->header.stamp,       // Exact time the picture was taken
                rclcpp::Duration::from_seconds(0.05)
            );

            // Optional: Get Camera to World transform for the slow-clock OctoMap
            // tf_world = tf_buffer_->lookupTransform("world", msg->header.frame_id, msg->header.stamp, ...);

        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
            return;
        }

        // 2. Convert ROS Transform to an Eigen Matrix for hyper-fast math
        Eigen::Isometry3d eigen_transform_body = tf2::transformToEigen(tf_body);

        // 3. Set up the raw byte iterators (Zero deserialization overhead!)
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

        // 4. The Single-Pass Loop
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            // Skip invalid points (NaNs are common in depth cams)
            if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
                continue;
            }

            // Create a 3D vector for the raw camera point
            Eigen::Vector3d raw_point(*iter_x, *iter_y, *iter_z);

            // Apply the matrix multiplication to frame-transform the point to the Body Frame
            Eigen::Vector3d body_point = eigen_transform_body * raw_point;

            // --- HAZARD CHECK LOGIC HERE ---
            // Example: If the point is within 2.0 meters of the drone body
            if (body_point.norm() < 2.0) {
                // It's a hazard! 
                // Push 'body_point' to your MoodyCamel Queue for the Reactive OA node
            }

            // --- WORLD OCTOMAP LOGIC HERE ---
            // Multiply by eigen_transform_world and push to the World Update queue
        }
    }
};