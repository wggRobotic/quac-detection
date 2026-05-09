#include "detection_server/detection_server.hpp"

#include <filesystem>
#include "yolos/tasks/detection.hpp"

class YOLODetectionServer : public DetectionServer
{
public:
  YOLODetectionServer();

  void yolo_detection_callback(const quac_interfaces::msg::ImageBGRD::SharedPtr msg, int i, std::vector<quac_interfaces::msg::BoundingBox>& detections);
  int init();

private:

  std::vector<std::shared_ptr<yolos::det::YOLODetector>> detectors;
  std::string model_path;
  std::string engine_path;
  std::string labels_path;
  std::string topic_name;
  double confidence_threshold;
};