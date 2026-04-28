# quac-detection
quac_detection package

## detection_server
### subscribers
- `<camera_names[i]>/bgrd` : `quac_interfaces/msgs/ImageBGRD`
- `<object_name>/delete` : `std_msgs/msg/String`

### publishers
- `<camera_names[i]>/<object_name>s` : `quac_interfaces/msgs/DetectedObjectArray`
- `<camera_names[i]>/<object_name>s/bounding_boxes` : `quac_interfaces/msgs/BoundingBox`
- `<object_name>s` : `quac_interfaces/msgs/DetectedObjectArray`
- `<object_name>s/images` : `sensor_msgs/msg/CompressedImage`

### parameters
```
object_name: String            # type of object detected by the server
camera_names: String[]         # names of the cameras
publish_rate: int              # rate that globally mapped objects are published at
publish_images_period: int     # rate that images of globally mapped objects are published at
reference_frame: String        # frame objects are mapped relative to
consideration_radius: double   # radius around a mapped object in which detected objects are counted towards it 
```

## qrcode_detection_server : detection_server

## yolo_detection_server : detection_server

### parameters
```
model_path: String      # path to the onnx model 
engine_path: String     # path to the tensorrt engine
labels_path: String     # path to the labels file
```
