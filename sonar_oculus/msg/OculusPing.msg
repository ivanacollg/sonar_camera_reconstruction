std_msgs/Header header

OculusFire fire_msg

uint32     ping_id
uint16     part_number
uint32     start_time

int16[]    bearings         # bearings of beams (bearing * PI / 18000)
float64    range_resolution # length of a single range bin
uint32     num_ranges       # number of range lines in the image
uint32     num_beams        # number of bearings in the image     

sensor_msgs/CompressedImage ping