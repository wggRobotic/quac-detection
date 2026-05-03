#include "detection_server/detection_server.hpp"

#include "quirc.h"

class QRCodeDetectionServer : public DetectionServer
{
public:
  QRCodeDetectionServer();

  void qrcode_detection_callback(const quac_interfaces::msg::ImageBGRD::SharedPtr msg, int i, std::vector<quac_interfaces::msg::BoundingBox>& detections);
  int init();
  void deinit();

private:
  std::vector<struct quirc*> detectors;
};