# One dynamic obstacle to be patched into the live ESDF.
# Only KIND_SPHERE is honored by ReplanFSM today; other kinds reserved for forward compatibility.
uint8 KIND_SPHERE=0
uint8 KIND_CUBE=1
uint8 KIND_CYLINDER=2

uint8 kind
geometry_msgs/Point center
float64 radius
geometry_msgs/Vector3 size
