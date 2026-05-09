#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <quac_interfaces/msg/image_bgrd.hpp>

#include <quac_interfaces/msg/detected_object.hpp>
#include <quac_interfaces/msg/detected_object_array.hpp>

#include <quac_interfaces/msg/bounding_box.hpp>
#include <quac_interfaces/msg/bounding_box_array.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/opencv.hpp>
#include <memory>
#include <cstdlib>
#include <mutex>
#include <atomic>

struct CameraHandler
{
  std::atomic<bool> busy{false};
  rclcpp::Subscription<quac_interfaces::msg::ImageBGRD>::SharedPtr image_subscriber;
  rclcpp::Publisher<quac_interfaces::msg::DetectedObjectArray>::SharedPtr object_publisher;
  rclcpp::Publisher<quac_interfaces::msg::BoundingBoxArray>::SharedPtr bb_publisher;
};

struct mapped_object
{
  float weight;
  quac_interfaces::msg::DetectedObject object;
  sensor_msgs::msg::CompressedImage::SharedPtr image;
};

using DetectionCallback = std::function<void(
    const quac_interfaces::msg::ImageBGRD::SharedPtr,
    int,
    std::vector<quac_interfaces::msg::BoundingBox>&
)>;

class DetectionServer : public rclcpp::Node
{
public:
  DetectionServer(const std::string& name, DetectionCallback callback);

  void run();
  void image_callback(const quac_interfaces::msg::ImageBGRD::SharedPtr msg, int i);
  void delete_callback(const std_msgs::msg::String::SharedPtr msg);
  void publish_objects_callback();
  void publish_images_callback();

protected:

  rclcpp::CallbackGroup::SharedPtr callback_group;
  DetectionCallback detection_callback;

  std::vector<std::unique_ptr<CameraHandler>> camera_handlers;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  struct
  {
    std::mutex mutex;

    struct
    {
      struct
      {
        std::atomic<bool> busy{false};
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<quac_interfaces::msg::DetectedObjectArray>::SharedPtr publisher;
        quac_interfaces::msg::DetectedObjectArray msg;
      } objects;
      struct
      {
        std::atomic<bool> busy{false};
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_publisher;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_publisher;
      } images;
    } publishing;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr delete_subscriber;
    
    std::vector<mapped_object> objects;
    int object_counter;
  } mapping;
  
  struct
  {
    std::vector<std::string> camera_names;
    std::string object_name;
    std::string reference_frame;
    double consideration_radius;
    double detection_weight;
    bool mapping;
  } param;
};