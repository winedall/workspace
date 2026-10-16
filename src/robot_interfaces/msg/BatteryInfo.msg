# BMS 电池状态消息

# 1. 总信息
uint16 total_volt         # 总压 0.1V → 实际值 = total_volt * 0.1
int16 current             # 电流 0.1A → 实际值 = current * 0.1
uint16 soc                # 剩余容量 0.1%
uint8 life                # 寿命 0~100%

# 2. 功率/温度/加热
uint16 power              # 功率 W
uint16 energy             # 总能量 Wh
int8 mos_temp             # MOS温度 ℃
int8 board_temp           # 主板温度 ℃
int8 heat_temp            # 加热膜温度 ℃
uint8 heat_current        # 加热电流 A

# 3. 电压统计
uint16 max_volt           # 最高单体电压 mV
uint8 max_volt_no         # 最高电压电芯号
uint16 min_volt           # 最低单体电压 mV
uint8 min_volt_no         # 最低电压电芯号
uint16 diff_volt          # 电压差 mV

# 4. 温度统计
int8 max_temp             # 最高温度 ℃
uint8 max_temp_no         # 最高温度传感器号
int8 min_temp             # 最低温度 ℃
uint8 min_temp_no         # 最低温度传感器号
uint8 diff_temp           # 温差 ℃

# 5. MOS状态
uint8 chg_mos             # 充电MOS 0/1
uint8 dis_mos             # 放电MOS 0/1
uint8 pre_mos             # 预充MOS 0/1
uint8 heat_mos            # 加热MOS 0/1
uint8 fan_mos             # 风扇MOS 0/1

# 6. 工作状态
uint8 bat_state           # 0静止 1充电 2放电
uint8 chg_detect          # 充电器接入 0/1
uint8 load_detect         # 负载接入 0/1
uint8 do_state            # 输出状态
uint8 di_state            # 输入状态

# 7. 系统配置
uint8 cell_num            # 电池串数
uint8 ntc_num             # NTC数量
uint32 remain_capacity    # 剩余容量 mAh
uint16 cycle_count        # 循环次数

# 8. 充电信息
uint16 rest_charge_time   # 剩余充电时间 min
uint8 wake_source         # 唤醒源

# 9. 限流/保护
uint8 limit_state         # 限流状态
int16 limit_current       # 限流值 0.1A
uint16 soh                # 健康度 0.1%
uint16 pwm                # PWM占空比
