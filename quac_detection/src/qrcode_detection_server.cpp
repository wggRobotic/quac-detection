#include "qrcode_detection_server/qrcode_detection_server.hpp"
#include <mutex>

QRCodeDetectionServer::QRCodeDetectionServer() : DetectionServer(
  "qrcode_detection_server",
  std::bind(&QRCodeDetectionServer::qrcode_detection_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
)
{
}

int QRCodeDetectionServer::init()
{
  detectors.resize(camera_handlers.size());
  for (int i = 0; i < detectors.size(); i++) detectors[i] = quirc_new();

  return 0;
}

void QRCodeDetectionServer::deinit()
{
  for (int i = 0; i < detectors.size(); i++) quirc_destroy(detectors[i]);
}

void QRCodeDetectionServer::qrcode_detection_callback(const quac_interfaces::msg::ImageBGRD::SharedPtr msg, int i, std::vector<quac_interfaces::msg::BoundingBox>& detections)
{
  cv::Mat grayscale;

  cv::Mat bgr_image(msg->height, msg->width, CV_8UC3, (void*)msg->bgr_data.data());
  cv::cvtColor(bgr_image, grayscale, cv::COLOR_BGR2GRAY);

  quirc_resize(detectors[i], msg->width, msg->height);

  uint8_t *qr_image = quirc_begin(detectors[i], (int*)&msg->width, (int*)&msg->height);
  memcpy(qr_image, grayscale.data, msg->width * msg->height);
  quirc_end(detectors[i]);

  int count = quirc_count(detectors[i]);

  for (int j = 0; j < count; j++)
  {
    struct quirc_code code;
    struct quirc_data data;

    quirc_extract(detectors[i], j, &code);

    if (quirc_decode(&code, &data) == QUIRC_SUCCESS)
    {
      quac_interfaces::msg::BoundingBox d;
      for (int k = 0; k < 4; k++) {d.corners[k].x = code.corners[k].x; d.corners[k].y = code.corners[k].y; }
      d.header.frame_id = std::string((char*)data.payload);
      d.header.stamp = msg->header.stamp;
      d.confidence = 1.;
      detections.push_back(d);
    }
  }
}

int main (int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<QRCodeDetectionServer>();
  if (node->init() == 0)
  {
    node->run();
    node->deinit();
  }

  rclcpp::shutdown();
}