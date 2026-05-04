# MTT raw CAN frame snapshot for bagging and diagnostics.
# Published only for known MTT frames when `publish_can_debug` is enabled.

std_msgs/Header header

string can_interface
uint32 can_id
bool is_extended
bool is_tx
bool handled_by_driver
string frame_name

uint8 dlc
uint8[8] data
