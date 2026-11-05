# One item read, attributed to a rack slot. Field names follow contracts/scan_record.schema.json;
# inv_uplink turns this into a scan record.
std_msgs/Header header

string ATTRIBUTION_LABEL="label"
string ATTRIBUTION_POSE="pose"
string ATTRIBUTION_NONE="none"

# UUID, idempotency key for the backend.
string scan_id
string mission_id
string code
# One of the LabelDetection SYMBOLOGY_* values.
string symbology
# Empty when the item could not be attributed to a slot.
string slot_code
string rack_id
# Shelf level, 0 = lowest. -1 when unknown.
int32 level
# Label position in the map frame, when known.
bool has_position
geometry_msgs/Point position_map
float32 confidence
# How slot_code was decided: ATTRIBUTION_LABEL (location label/tag seen nearby),
# ATTRIBUTION_POSE (pose-only fallback) or ATTRIBUTION_NONE.
string attribution_method
