# One decoded label (barcode, QR code or AprilTag) in one camera image.
# Perception does not interpret the payload: deciding whether a code is an item or a
# rack/slot location label is the inventory node's job (it knows the warehouse layout).

# Symbology names match contracts/scan_record.schema.json.
string SYMBOLOGY_QR="qr"
string SYMBOLOGY_EAN13="ean13"
string SYMBOLOGY_CODE128="code128"
string SYMBOLOGY_APRILTAG="apriltag"
string SYMBOLOGY_OTHER="other"

string symbology
# Decoded payload. AprilTags use "<family>:<id>", for example "tag36h11:17".
string code
# Decoder confidence in [0, 1]. 1.0 when the decoder gives no score.
float32 confidence

# Image geometry, in pixels of the image named by the array header.
float32 center_x_px
float32 center_y_px
# Corner points x0, y0, x1, y1, ... clockwise from the decoder's first corner. May be empty.
float32[] corners_px

# Label pose in the camera optical frame (array header frame_id), when it could be estimated.
bool has_pose
geometry_msgs/Pose pose
# Physical edge length used for pose estimation, in metres (0 when unknown).
float32 size_m
