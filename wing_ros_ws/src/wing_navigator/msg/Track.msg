# This message is for transferring the information about a track determined
# by the tracker algorithm on the funnywing.

uint8 TRACK_STATE_UNLOCKED = 0
uint8 TRACK_STATE_LOCKED = 1

uint32 time_stamp
uint16 track_id
uint32 frame_count  # used for collecting the tracks belong to one frame
uint8 track_state
float32 top_left_x
float32 top_left_y
float32 bottom_right_x
float32 bottom_right_y
