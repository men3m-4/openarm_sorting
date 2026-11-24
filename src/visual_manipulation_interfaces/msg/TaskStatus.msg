# Status of a task coordinator (published on every state change).
std_msgs/Header header
string task                            # e.g. "portafilter"
string state                           # current state, e.g. "ALIGN"
string previous_state
string detail                          # human-readable progress / error message
bool active                            # a sequence is running
bool failed                            # the last sequence ended in FAILED
int32 attempt                          # retry counter of the current state
