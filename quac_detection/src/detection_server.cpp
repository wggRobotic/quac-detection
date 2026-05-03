#include "detection_server/detection_server.hpp"
#include <cmath>

DetectionServer::DetectionServer(const std::string& name, DetectionCallback callback) : Node(name), tf_buffer(get_clock()), tf_listener(tf_buffer), detection_callback(callback)
{
  // declare and read parameters
  declare_parameter<std::vector<std::string>>("camera_names", std::vector<std::string>{"cam"});
  param.camera_names = get_parameter("camera_names").as_string_array();

  declare_parameter<std::string>("reference_frame", "map");
  param.reference_frame = get_parameter("reference_frame").as_string();

  declare_parameter<int>("publish_rate", 10);
  int publish_rate = get_parameter("publish_rate").as_int();

  declare_parameter<int>("publish_images_period", 10);
  int publish_images_period = get_parameter("publish_images_period").as_int();

  declare_parameter<std::string>("object_name", "object");
  param.object_name = get_parameter("object_name").as_string();

  declare_parameter<double>("consideration_radius", 0.1);
  param.consideration_radius = get_parameter("consideration_radius").as_double();

  mapping.object_counter = 0;

  // create callback group
  callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group;

  // global publishers / subscribers

  mapping.delete_subscriber = create_subscription<std_msgs::msg::String>(
    param.object_name + "s/delete",
    10,
    std::bind(&DetectionServer::delete_callback, this, std::placeholders::_1)
  );

  mapping.publishing.objects.timer = create_wall_timer(std::chrono::milliseconds((int)(1000.f/(float)publish_rate)), std::bind(&DetectionServer::publish_objects_callback, this), callback_group);
  mapping.publishing.objects.publisher = create_publisher<quac_interfaces::msg::DetectedObjectArray>(param.object_name + "s", 10);
  
  mapping.publishing.images.timer = create_wall_timer(std::chrono::milliseconds((int)(1000.f * (float)publish_images_period)), std::bind(&DetectionServer::publish_images_callback, this), callback_group);
  mapping.publishing.images.image_publisher = create_publisher<sensor_msgs::msg::CompressedImage>(param.object_name + "s/images", 10);
  mapping.publishing.images.json_publisher = create_publisher<std_msgs::msg::String>(param.object_name + "s/json", 10);

  // individual camera handlers
  camera_handlers.resize(param.camera_names.size());
  
  for (int i = 0; i < camera_handlers.size(); i++)
  {
    camera_handlers[i] = std::make_unique<CameraHandler>();

    camera_handlers[i]->image_subscriber = create_subscription<quac_interfaces::msg::ImageBGRD>(
      param.camera_names[i] + "/bgrd",
      10,
      [this, i](quac_interfaces::msg::ImageBGRD::SharedPtr msg) {image_callback(msg, i);}, 
      options
    );

    camera_handlers[i]->object_publisher = create_publisher<quac_interfaces::msg::DetectedObjectArray>(param.camera_names[i] + "/" + param.object_name + "s", 10);
    camera_handlers[i]->bb_publisher = create_publisher<quac_interfaces::msg::BoundingBoxArray>(param.camera_names[i] + "/" + param.object_name + "s/bounding_boxes", 10);
  }
}

void DetectionServer::publish_objects_callback()
{
  if (mapping.publishing.objects.busy.exchange(true)) return;

  quac_interfaces::msg::DetectedObjectArray& msg = mapping.publishing.objects.msg;

  msg.header.stamp = now();
  msg.header.frame_id = param.reference_frame;

  {
    std::lock_guard<std::mutex> lock(mapping.mutex);

    msg.objects.resize(mapping.objects.size());

    for (int i = 0; i < msg.objects.size(); i++) msg.objects[i] = mapping.objects[i].object;
  }

  if (msg.objects.size() > 0) mapping.publishing.objects.publisher->publish(msg);

  mapping.publishing.objects.busy.store(false);
}

void DetectionServer::publish_images_callback()
{
  if (mapping.publishing.images.busy.exchange(true)) return;

  std_msgs::msg::String json_msg;
  json_msg.data = (
    "{\n"
    "  \"reference_frame\": \"" + param.reference_frame + "\",\n"
    "  \"objects\": ["

  );

  std::vector<sensor_msgs::msg::CompressedImage::SharedPtr> image_msgs;

  {
    std::lock_guard<std::mutex> lock(mapping.mutex);

    image_msgs.resize(mapping.objects.size());

    for (int i = 0; i < image_msgs.size(); i++)
    {
      json_msg.data += (
        "\n"
        "    {\n"
        "      \"name\": \"" + mapping.objects[i].object.header.frame_id + "\",\n"
        "      \"type\": \"" + mapping.objects[i].object.box.header.frame_id + "\",\n"
        "      \"position\": {"
                              "\"x\": " + std::to_string(mapping.objects[i].object.pose.position.x) + ", "
                              "\"y\": " + std::to_string(mapping.objects[i].object.pose.position.y) + ", "
                              "\"z\": " + std::to_string(mapping.objects[i].object.pose.position.z) + ""
        "      }\n"
        "    },"
      );

      image_msgs[i] = mapping.objects[i].image;
    }
  }

  json_msg.data[json_msg.data.size() - 1] = '\n';
  json_msg.data += (
    "  ]\n"
    "}"
  );

  for (int i = 0; i < image_msgs.size(); i++) mapping.publishing.images.image_publisher->publish(*image_msgs[i]);

  if (image_msgs.size() > 0) mapping.publishing.images.json_publisher->publish(json_msg);

  mapping.publishing.images.busy.store(false);
}

void DetectionServer::delete_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mapping.mutex);

  for (int i = 0; i < mapping.objects.size(); i++)
  {
    if (mapping.objects[i].object.header.frame_id == msg->data)
    {
      mapping.objects.erase(mapping.objects.begin() + i);
      break;
    }
  }
}

void create_mapped_image(
  sensor_msgs::msg::CompressedImage::SharedPtr& compressed_image, 
  quac_interfaces::msg::ImageBGRD::SharedPtr bgrd_image, 
  const quac_interfaces::msg::BoundingBox& detection,
  const std::string& name
)
{
  cv::Mat view(bgrd_image->height, bgrd_image->width, CV_8UC3, bgrd_image->bgr_data.data());
  cv::Mat copy = view.clone();

  // put bounding box
  {
    cv::Scalar color(0, 0, 255);

    int x_min = copy.cols, y_min = copy.rows;
    for (int i = 0; i < 4; i++)
    {
      if (x_min > detection.corners[i].x) x_min = detection.corners[i].x;
      if (y_min > detection.corners[i].y) y_min = detection.corners[i].y;
    }

    std::vector<cv::Point> pts;
    for (int i = 0; i < 4; i++) pts.emplace_back(detection.corners[i].x, detection.corners[i].y);
    
    std::vector<std::vector<cv::Point>> contour = {pts};

    cv::polylines(copy, contour, true, color, 2, cv::LINE_AA);

    std::string label = detection.header.frame_id;

    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = std::min(copy.rows, copy.cols) * 0.0008;
    fontScale = std::max(fontScale, 0.4);
    int textThickness = std::max(1, static_cast<int>(std::min(copy.rows, copy.cols) * 0.002));
    int baseline = 0;

    cv::Size textSize = cv::getTextSize(label, fontFace, fontScale, textThickness, &baseline);

    int labelY = std::max(y_min, textSize.height + 5);
    cv::Point labelTopLeft(x_min, labelY - textSize.height - 5);
    cv::Point labelBottomRight(x_min + textSize.width + 5, labelY + baseline - 5);

    cv::rectangle(copy, labelTopLeft, labelBottomRight, color, cv::FILLED);
    cv::putText(copy, label, cv::Point(x_min + 2, labelY - 2), fontFace, fontScale, cv::Scalar(255, 255, 255), textThickness, cv::LINE_AA);
  }

  sensor_msgs::msg::CompressedImage::SharedPtr msg = std::make_shared<sensor_msgs::msg::CompressedImage>();

  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
  cv::imencode(".jpg", copy, msg->data, params);

  msg->header.stamp = bgrd_image->header.stamp;
  msg->header.frame_id = name;
  msg->format = "jpeg";

  compressed_image = msg;
}

void DetectionServer::image_callback(quac_interfaces::msg::ImageBGRD::SharedPtr msg, int i)
{
  if (camera_handlers[i]->busy.exchange(true)) return;

  std::vector<quac_interfaces::msg::BoundingBox> detections;

  detection_callback(msg, i, detections);
  if (detections.size() == 0) { camera_handlers[i]->busy.store(false); return; }

  quac_interfaces::msg::DetectedObjectArray object_msg;
  object_msg.header.stamp = msg->header.stamp;
  object_msg.header.frame_id = msg->header.frame_id;
  object_msg.objects.reserve(detections.size());

  for (int j = 0; j < detections.size(); j++)
  {
    quac_interfaces::msg::DetectedObject object;
    object.header.frame_id = "detection_" + std::to_string(j);
    object.header.stamp = msg->header.stamp;
    object.box = detections[j];

    object.pose.orientation.x = 0.0;
    object.pose.orientation.y = 0.0;
    object.pose.orientation.z = 0.0;
    object.pose.orientation.w = 1.0;

    int u = (detections[j].corners[0].x + detections[j].corners[1].x + detections[j].corners[2].x + detections[j].corners[3].x)/ 4;
    int v = (detections[j].corners[0].y + detections[j].corners[1].y + detections[j].corners[2].y + detections[j].corners[3].y)/ 4;

    object.pose.position.z = ((float)msg->depth_data[v * msg->width + u]) * msg->depth_scale;
    object.pose.position.x = (u - msg->ppx) * object.pose.position.z / msg->fx;
    object.pose.position.y = (v - msg->ppy) * object.pose.position.z / msg->fy;

    object_msg.objects.push_back(object);
  }

  camera_handlers[i]->object_publisher->publish(object_msg);

  geometry_msgs::msg::TransformStamped tf_msg;

  try
  {
    tf_msg = tf_buffer.lookupTransform(
      param.reference_frame,
      msg->header.frame_id,
      msg->header.stamp
    );
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(get_logger(), "TF lookup failed: %s", ex.what());
    camera_handlers[i]->busy.store(false);
    return;
  }

  rclcpp::Duration diff = rclcpp::Time(msg->header.stamp) - rclcpp::Time(tf_msg.header.stamp);
  if (diff.seconds() < 0.0) diff = rclcpp::Duration::from_seconds(-diff.seconds());

  rclcpp::Duration tolerance = rclcpp::Duration::from_seconds(0.05);

  if (diff > tolerance)
  {
    RCLCPP_ERROR(get_logger(), "TF frame from %s to %s is too old", msg->header.frame_id.c_str(), param.reference_frame.c_str());
    camera_handlers[i]->busy.store(false);
    return;
  }

  tf2::Transform global_to_camera;
  tf2::fromMsg(tf_msg.transform, global_to_camera);

  for (auto &object : object_msg.objects)
  {
    tf2::Transform camera_to_object;
    tf2::fromMsg(object.pose, camera_to_object);

    tf2::toMsg(global_to_camera * camera_to_object, object.pose);
  }

  std::lock_guard<std::mutex> lock(mapping.mutex);

  for (int j = 0; j < object_msg.objects.size(); j++)
  {
    for (int k = 0; ; k++)
    {
      if (k < mapping.objects.size())
      {
        if (object_msg.objects[j].box.header.frame_id == mapping.objects[k].object.box.header.frame_id)
        {
          geometry_msgs::msg::Point& mapped_object_pos = mapping.objects[k].object.pose.position;
          geometry_msgs::msg::Point& object_pos = object_msg.objects[j].pose.position;

          double distance = std::sqrt(
            (mapped_object_pos.x - object_pos.x) * (mapped_object_pos.x - object_pos.x) +
            (mapped_object_pos.y - object_pos.y) * (mapped_object_pos.y - object_pos.y) +
            (mapped_object_pos.z - object_pos.z) * (mapped_object_pos.z - object_pos.z)
          );
          if (distance < param.consideration_radius)
          {
            mapped_object_pos.x = 
              (mapped_object_pos.x * (double)mapping.objects[k].times_detected + object_pos.x) / 
              (double)(mapping.objects[k].times_detected + 1);

            mapped_object_pos.y = 
              (mapped_object_pos.y * (double)mapping.objects[k].times_detected + object_pos.y) / 
              (double)(mapping.objects[k].times_detected + 1);

            mapped_object_pos.z = 
              (mapped_object_pos.z * (double)mapping.objects[k].times_detected + object_pos.z) / 
              (double)(mapping.objects[k].times_detected + 1);

            mapping.objects[k].times_detected++;

            if (object_msg.objects[j].box.confidence > mapping.objects[k].object.box.confidence)
            {
              mapping.objects[k].object.box.confidence = object_msg.objects[j].box.confidence;

              create_mapped_image(
                mapping.objects[k].image, 
                msg, 
                detections[j], 
                mapping.objects[k].object.header.frame_id
              );
            }

            break;
          }
        }
      }
      else
      {
        mapped_object obj;
        
        obj.object = object_msg.objects[j];
        obj.object.header.frame_id = param.object_name + "_" + std::to_string(mapping.object_counter);
        obj.times_detected = 1;
        create_mapped_image(
          obj.image, 
          msg, 
          detections[j], 
          obj.object.header.frame_id
        );

        mapping.objects.push_back(obj);

        mapping.object_counter++;

        break;
      }
      
    }
  }

  camera_handlers[i]->busy.store(false);
}

void DetectionServer::run()
{
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), camera_handlers.size() + 2 + 2);
  executor.add_node(shared_from_this());
  executor.spin();
}