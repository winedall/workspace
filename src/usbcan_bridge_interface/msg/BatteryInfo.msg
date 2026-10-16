# BMS 电池管理系统 完整数据消息

# 1. 单体电压 & 温度
uint16[] cell_volt        # 单体电压 单位：mV (长度15)
int8[] cell_temp          # 单体温度 单位：℃ (长度15)

# 2. 总信息
uint16 total_volt         # 总压 0.1V → 实际值 = total_volt * 0.1
int16 current             # 电流 0.1A → 实际值 = current * 0.1
uint16 soc                # 剩余容量 0.1%
uint8 life                # 寿命 0~100%

# 3. 功率/温度/加热
uint16 power              # 功率 W
uint16 energy             # 总能量 Wh
int8 mos_temp             # MOS温度 ℃
int8 board_temp           # 主板温度 ℃
int8 heat_temp            # 加热膜温度 ℃
uint8 heat_current        # 加热电流 A

# 4. 电压统计
uint16 max_volt           # 最高单体电压 mV
uint8 max_volt_no         # 最高电压电芯号
uint16 min_volt           # 最低单体电压 mV
uint8 min_volt_no         # 最低电压电芯号
uint16 diff_volt          # 电压差 mV

# 5. 温度统计
int8 max_temp             # 最高温度 ℃
uint8 max_temp_no         # 最高温度传感器号
int8 min_temp             # 最低温度 ℃
uint8 min_temp_no         # 最低温度传感器号
uint8 diff_temp           # 温差 ℃

# 6. MOS状态
uint8 chg_mos             # 充电MOS 0/1
uint8 dis_mos             # 放电MOS 0/1
uint8 pre_mos             # 预充MOS 0/1
uint8 heat_mos            # 加热MOS 0/1
uint8 fan_mos             # 风扇MOS 0/1

# 7. 工作状态
uint8 bat_state           # 0静止 1充电 2放电
uint8 chg_detect          # 充电器接入 0/1
uint8 load_detect         # 负载接入 0/1
uint8 do_state            # 输出状态
uint8 di_state            # 输入状态

# 8. 系统配置
uint8 cell_num            # 电池串数
uint8 ntc_num             # NTC数量
uint32 remain_capacity    # 剩余容量 mAh
uint16 cycle_count        # 循环次数

# 9. 充电信息
uint16 rest_charge_time   # 剩余充电时间 min
uint8 wake_source         # 唤醒源

# 10. 限流/保护
uint8 limit_state         # 限流状态
int16 limit_current       # 限流值 0.1A
uint16 soh                # 健康度 0.1%
uint16 pwm                # PWM占空比

# 11. 动力故障 - Page1
uint8 fault_cell_over_volt_level       # 单体过压告警等级 0-7
uint8 fault_cell_under_volt_level      # 单体欠压告警等级 0-7
uint8 fault_smart_chg_connect          # 智能充电器连接 0/1
uint8 fault_smart_chg_connect_fail     # 智能充电器连接失败 0/1

uint8 fault_volt_diff_level            # 压差过大告警等级 0-7
uint8 fault_charge_over_temp_level     # 充电高温告警等级 0-7
uint8 fault_smart_dischg_connect       # 智能放电设备连接 0/1
uint8 fault_smart_dischg_connect_fail  # 智能放电设备连接失败 0/1

uint8 fault_charge_low_temp_level      # 充电低温告警等级 0-7
uint8 fault_dischg_over_temp_level     # 放电高温告警等级 0-7
uint8 fault_chg_mos_over_temp          # 充电MOS温度过高 0/1
uint8 fault_chg_mos_temp_sensor        # 充电MOS温度检测故障 0/1

uint8 fault_dischg_low_temp_level      # 放电低温告警等级 0-7
uint8 fault_temp_diff_level           # 温差过大告警等级 0-7
uint8 fault_dischg_mos_over_temp      # 放电MOS温度过高 0/1
uint8 fault_dischg_mos_temp_sensor    # 放电MOS温度检测故障 0/1

uint8 fault_total_over_volt_level     # 总压过高告警等级 0-7
uint8 fault_total_under_volt_level    # 总压过低告警等级 0-7
uint8 fault_short_protect            # 短路保护 0/1

uint8 fault_charge_over_cur_level     # 充电过流告警等级 0-7
uint8 fault_dischg_over_cur_level     # 放电过流告警等级 0-7
uint8 fault_low_volt_forbid_chg       # 低压禁止充电 0/1
uint8 fault_high_volt_forbid_dischg   # 高压禁止放电 0/1

uint8 fault_soc_low_level             # SOC过低告警等级 0-7
uint8 fault_soh_low_level             # SOH过低告警等级 0-7
uint8 fault_parallel_comm_succ        # 并联通信成功 0/1
uint8 fault_parallel_comm_fail        # 并联通信失败 0/1

# 12. 动力故障 - Page2
uint8 fault_mos_over_temp_level       # MOS温度过高告警等级 0-7
uint8 fault_thermal_runaway_level     # 热失控告警等级 0-7

uint8 fault_afe_chip_err              # AFE芯片故障 0/1
uint8 fault_afe_comm_err              # AFE通信故障 0/1
uint8 fault_afe_sample_err            # AFE采样故障 0/1
uint8 fault_volt_detect_err           # 电压检测故障 0/1
uint8 fault_volt_line_disconnect      # 电压采集线掉线 0/1
uint8 fault_total_volt_detect_err     # 总压检测故障 0/1
uint8 fault_cur_detect_err            # 电流检测故障 0/1
uint8 fault_temp_detect_err           # 温度检测故障 0/1

uint8 fault_temp_line_disconnect      # 温度采集线掉线 0/1
uint8 fault_eeprom_err2               # EEPROM故障 0/1
uint8 fault_flash_err                 # FLASH故障 0/1
uint8 fault_rtc_err2                  # RTC故障 0/1
uint8 fault_chg_mos_err               # 充电MOS故障 0/1
uint8 fault_dischg_mos_err            # 放电MOS故障 0/1
uint8 fault_pre_mos_err               # 预充MOS故障 0/1
uint8 fault_precharge_fail            # 预充失败 0/1

uint8 fault_cmd_ctrl_chg_mos_off      # 通信指令控制充电MOS OFF 0/1
uint8 fault_cmd_ctrl_dischg_mos_off   # 通信指令控制放电MOS OFF 0/1
uint8 fault_sw_ctrl_chg_mos_off       # 开关控制充电MOS OFF 0/1
uint8 fault_sw_ctrl_dischg_mos_off    # 开关控制放电MOS OFF 0/1
uint8 fault_fan_working               # 风扇工作 0/1
uint8 fault_heat_working              # 加热工作 0/1
uint8 fault_limit_cur_working         # 限流模块工作 0/1
uint8 fault_heat_err                  # 加热故障 0/1