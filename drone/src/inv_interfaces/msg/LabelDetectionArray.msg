# All labels one detector decoded in one image. Published on /perception/detections.
# header.stamp is the image capture time and header.frame_id the camera optical frame,
# so detections from different detectors on the same image share a stamp.
std_msgs/Header header
# Detector that produced this array, for example "zxing" or "apriltag_ros".
string source
# Image size, so consumers can reason about pixel coordinates.
uint32 image_width
uint32 image_height
LabelDetection[] detections
