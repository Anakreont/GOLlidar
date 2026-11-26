-- Jak carto.lua, ale z predkoscia katowa z kursu BNO085 (temat /imu).
include "carto.lua"
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(8.)
return options
