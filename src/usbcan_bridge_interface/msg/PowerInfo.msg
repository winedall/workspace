# 电源板信息
# 电压电流数组数量
# POWER_CHIP_NUM                3U 
# POWER_CHANNEL_PER_CHIP        2U 
# POWER_TOTAL_CHANNEL           (POWER_CHIP_NUM * POWER_CHANNEL_PER_CHIP)

uint8 o_p5v_en

uint32[] max_vol
uint32[] avg_vol
uint32[] max_cur
uint32[] avg_cur

# GPIO 状态寄存器1 - 电源板
uint32 gpio_stat
uint8 p24v_h_pg
uint8 p24v_pg
uint8 p48v_left_hand_flt
uint8 p48v_left_leg_flt
uint8 p48v_right_hand_flt
uint8 p48v_right_leg_flt
uint8 p48v_right_waist_flt
uint8 p48v_left_hand_fltgd
uint8 p48v_left_leg_fltgd
uint8 p48v_right_hand_fltgd
uint8 p48v_right_leg_fltgd
uint8 p48v_right_waist_fltgd

# ADC 温度值 (°C)
float32 adc0
float32 adc1
float32 adc2

# GPIO 状态寄存器2 - 控制板
uint32 gpio_stat2
uint8 usb4_5_fault
uint8 usb6_7_fault
uint8 p5v_gh_fault
uint8 ptypec2_3_fault
uint8 p3v3_pg
uint8 p5v_pg
uint8 p12v_pg
uint8 p1v8_pg
uint8 p1v_pg

# TMP112 温度传感器 (°C)
float32 tmp112_temp0
float32 tmp112_temp1