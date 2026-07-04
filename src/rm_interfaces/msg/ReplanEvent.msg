uint8 UNKNOWN=0
uint8 GOAL_CHANGED=1
uint8 TIME_EXPIRED=2
uint8 PATH_BLOCKED=3
uint8 FORCED=4

std_msgs/Header header
bool need_replan
uint64 event_id
uint8 reason
builtin_interfaces/Time candidate_path_stamp
geometry_msgs/PoseStamped goal
