# Status for one stepper motor. stepper_number is 1-based (1–4).
uint8  stepper_number
bool   enabled
uint8  motion_state    # 0=idle  1=accel  2=cruise  3=decel  4=homing  5=fault
uint8  limit_flags     # bit0=min limit  bit1=max limit
int32  count
int32  target_count
int32  current_speed
uint32 timestamp
