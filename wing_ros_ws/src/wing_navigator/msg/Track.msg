# This message is for transferring the information about a track determined
# by the tracker algorithm on the funnywing.

uint8 TRACK_STATE_UNLOCKED = 0
uint8 TRACK_STATE_LOCKED = 1

uint32 time_stamp
uint32 track_id
uint32 frame_count  # used for collecting the tracks belong to one frame
uint8 track_state
float32 rect_top_x
float32 rect_top_y
float32 rect_bottom_x
float32 rect_bottom_y
