# One dynamic obstacle to be patched into the live ESDF.
# KIND_SPHERE and KIND_CUBE are honored by ReplanFSM.
uint8 KIND_SPHERE=0
uint8 KIND_CUBE=1
uint8 KIND_CYLINDER=2

uint8 kind
geometry_msgs/Point center
float64 radius
geometry_msgs/Vector3 size
# Visual mesh model name (catalog key, e.g. "building", "car"). Empty = default.
string model
