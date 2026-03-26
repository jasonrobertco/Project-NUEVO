# Status for one DC motor. motor_number is 1-based (1–4).
uint8  motor_number
uint8  mode          # 0=disabled  1=position  2=velocity  3=pwm  4=homing
uint8  fault_flags   # bit0=home_limit_active  bit1=encoder_fault
int32  position
int32  velocity
int32  target_pos
int32  target_vel
int16  pwm_output
int16  current_ma
uint32 timestamp
