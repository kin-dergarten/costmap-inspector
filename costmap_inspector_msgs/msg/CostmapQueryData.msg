std_msgs/Header header

int8 RESULT_NO_LETHAL_DATA=1
int8 RESULT_LETHAL_DATA=2
int8 RESULT_PERSISTED_LETHAL_DATA=3
int8 RESULT_FAILURE=-1
int8 result

int64 num_cells_in_polygon
int64 num_lethal_cells_in_most_lethal_layer

# Distance and angle to the barycenter of all detected lethal cells.
float64 mean_object_distance # in meters
float64 mean_object_angle # in radians, relative to the robot's forward direction. Positive is counter-clockwise.

string most_lethal_layer # Name of the costmap layer containing the most lethal obstacles. Empty if no obstacles detected.
string[] persisted_visible_sources # Human-readable names of obstacle sources that have detected obstacles within the timeout period.
string[] persisted_lethal_layer_names # Same as above, but for the layer names.
costmap_inspector_msgs/CostmapQueryLayerInfo[] layer_info # detailed results for each layer (current, not with timeout).
int64 DID_NOT_QUERY_LAYER=-1
int64 ERROR_QUERYING_LAYER=-2
