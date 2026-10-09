# Executor publication time; elapsed_ms follows the actual scaled trajectory.
std_msgs/Header header
uint8 RUNNING=0
uint8 COMPLETED=1
uint8 ABORTED=2
uint32 id
uint32 elapsed_ms
uint8 status
