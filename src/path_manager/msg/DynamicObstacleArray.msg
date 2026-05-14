# Batch of dynamic obstacles, applied in order.
# If `replace` is true, the receiver clears all existing dynamic obstacles
# before adding the spheres in `obstacles`. The clear+add is atomic in the
# callback, so there is no transient empty/loaded race window.
bool replace
DynamicObstacleSpec[] obstacles
