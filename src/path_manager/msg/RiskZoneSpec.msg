# V3 risk zone (soft cost field, quadratic moat).
# moat(x) = peak * max(0, 1 - ||x - center|| / reach)^2
geometry_msgs/Point center
float64 reach    # meters; risk is exactly zero outside this ball
float64 peak     # dimensionless in (0, 1]
