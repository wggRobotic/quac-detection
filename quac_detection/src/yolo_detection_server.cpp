#include "yolo_detection_server/yolo_detection_server.hpp"
#include <mutex>

YOLODetectionServer::YOLODetectionServer() : DetectionServer(
  "yolo_detection_server", 
  std::bind(&YOLODetectionServer::yolo_detection_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
)
{
  declare_parameter<std::string>("model_path", "model");
  model_path = get_parameter("model_path").as_string();

  declare_parameter<std::string>("engine_path", "engine");
  engine_path = get_parameter("engine_path").as_string();

  declare_parameter<std::string>("labels_path", "labels");
  labels_path = get_parameter("labels_path").as_string();

  declare_parameter<double>("confidence_threshold", 0.5);
  confidence_threshold = get_parameter("confidence_threshold").as_double();
}

int YOLODetectionServer::init()
{
  if (std::filesystem::exists(engine_path) == false)
  {
    RCLCPP_INFO(get_logger(), "%s doesn't exist. Generating ...", engine_path.c_str());

    std::string engine_cmd = 
      "/usr/src/tensorrt/bin/trtexec "
      "--onnx=" + model_path + " "
      "--saveEngine=" + engine_path + " "
      "--verbose "
      "--skipInference "
      "--fp16 ";

    ::system(engine_cmd.c_str());
  }

  if (std::filesystem::exists(engine_path) == false) return -1;

  detectors.resize(camera_handlers.size());
  for (int i = 0; i < detectors.size(); i++) detectors[i] = std::make_shared<yolos::det::YOLODetector>(engine_path, labels_path);

  return 0;
}

void YOLODetectionServer::yolo_detection_callback(const quac_interfaces::msg::ImageBGRD::SharedPtr msg, int i, std::vector<quac_interfaces::msg::BoundingBox>& detections)
{
  cv::Mat image(msg->height, msg->width, CV_8UC3, (void*)msg->bgr_data.data());
  std::vector<yolos::det::Detection> results = detectors[i]->detect(image, confidence_threshold);

  for (int j = 0; j < results.size(); j++)
  {
    quac_interfaces::msg::BoundingBox d;
    d.corners[0].x = results[j].box.x;
    d.corners[0].y = results[j].box.y;
    d.corners[1].x = results[j].box.x + results[j].box.width;
    d.corners[1].y = results[j].box.y;
    d.corners[2].x = results[j].box.x + results[j].box.width;
    d.corners[2].y = results[j].box.y + results[j].box.height;
    d.corners[3].x = results[j].box.x;
    d.corners[3].y = results[j].box.y + results[j].box.height;
    
    d.header.frame_id = detectors[i]->getClassNames()[results[j].classId];
    d.header.stamp = msg->header.stamp;
    d.confidence = results[j].conf;

    detections.push_back(d);
  }
}

int main (int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<YOLODetectionServer>();
  if (node->init() == 0) node->run();
  
  rclcpp::shutdown();
}