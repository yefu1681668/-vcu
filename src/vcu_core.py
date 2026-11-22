# -*- coding: utf-8 -*-
"""
VCU 整车控制器核心控制逻辑（Python 参考实现）

本文件是 Simulink/Stateflow 模型的「可执行规格」：状态机与扭矩控制的每一条
规则都与 model/ 下的状态机图一一对应。用 Python 实现的目的：
  1. 无需 MATLAB 即可在任意机器上运行单元测试，验证控制策略正确性；
  2. 便于 CI 做回归测试（test/test_vcu.py），也是量产 MBD 流程的标准做法。

量产项目中此逻辑由 Simulink 模型经 Embedded Coder 生成 C 代码，
本文件用于需求验证与模型在环（MIL）对标。

作者: yefu1681668
版本: 1.2
"""

from enum import IntEnum, auto


class VcuState(IntEnum):
    """整车控制状态机状态（与 DBC VCU_State 信号取值一致）"""
    INIT = 0        # 上电初始化，自检
    PRECHARGE = 1   # 预充：等待母线电压达到电池电压的 95%
    READY = 2       # 高压就绪，等待挡位与踏板输入
    DRIVING = 3     # 驱动：输出正扭矩
    REGEN = 4       # 制动能量回收：输出负扭矩
    FAULT = 5       # 故障：扭矩归零，等待恢复
    SHUTDOWN = 6    # 下电


class FaultLevel(IntEnum):
    """故障等级（与 DBC FaultLevel 信号取值一致）"""
    NO_FAULT = 0
    WARN = 1
    LIMIT_TORQUE = 2
    STOP = 3


class Gear(IntEnum):
    P = 0
    R = 1
    N = 2
    D = 3


class DriveMode(IntEnum):
    ECO = 0
    NORMAL = 1
    SPORT = 2


# ---- 标定参数（量产中由标定工具写入，此处硬编码便于复现） ----
PRECHARGE_RATIO = 0.95      # 预充完成阈值：母线电压 / 电池电压
PRECHARGE_HOLD_MS = 100     # 预充完成需保持时间，防抖
SOC_LOW_THRESHOLD = 15.0    # SOC 过低阈值（%），触发限扭
SOC_RECOVER_THRESHOLD = 20.0
SOC_REGEN_DISABLE = 95.0    # SOC 高于此值禁止能量回收
COMM_TIMEOUT_MS = 500       # CAN 报文超时判定（BMS/MCU）
TORQUE_LIMIT_BY_GEAR = {    # 各挡位最大扭矩（Nm）
    Gear.P: 0.0,
    Gear.R: 800.0,
    Gear.N: 0.0,
    Gear.D: 1200.0,
}
PEDAL_TORQUE_MAP = (        # 加速踏板开度(%) -> 满扭矩比例，线性分段
    (0.0, 0.0),
    (10.0, 0.0),            # 10% 以下为死区
    (50.0, 0.45),
    (100.0, 1.0),
)
MODE_TORQUE_SCALE = {
    DriveMode.ECO: 0.7,
    DriveMode.NORMAL: 1.0,
    DriveMode.SPORT: 1.15,
}


def _interp(x, points):
    """一维线性插值（等价 Simulink Lookup Table 模块）"""
    if x <= points[0][0]:
        return points[0][1]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return points[-1][1]


class VehicleControlUnit:
    """
    VCU 整车控制器主类。

    典型调用方式（10 ms 周期任务）：
        vcu = VehicleControlUnit()
        vcu.step(dt_ms=10, batt_voltage=350, bus_voltage=0,
                 soc=80, pedal=0, brake=0, gear=Gear.P, ...)
    """

    def __init__(self):
        self.state = VcuState.INIT
        self.fault_level = FaultLevel.NO_FAULT
        self.fault_code = 0
        self._precharge_ok_ms = 0
        self._soc_low_latched = False
        self._bms_tick_ms = 0
        self._mcu_tick_ms = 0
        self.torque_req = 0.0
        self.torque_limit_max = 0.0
        self.torque_limit_min = 0.0
        self.ready = False
        self.hv_enable = False
        self.regen_enable = False
        self.vehicle_speed = 0.0   # km/h，由 MCU 转速换算，简化由外部写入
        self.log = []

    # ---------- 状态机主步进 ----------
    def step(self, dt_ms, batt_voltage, bus_voltage, soc, accel_pedal,
             brake_pedal, gear, key_state, drive_mode, bms_alive=True,
             mcu_alive=True, mcu_overtemp=False, vehicle_speed=0.0):
        """一个 10 ms 调度周期的执行入口。返回当前状态。"""
        self.vehicle_speed = vehicle_speed
        self._update_communication(dt_ms, bms_alive, mcu_alive)
        self._update_fault(soc, mcu_overtemp)
        self._state_transition(dt_ms, batt_voltage, bus_voltage, soc,
                               accel_pedal, brake_pedal, gear, key_state)
        self._torque_control(soc, accel_pedal, brake_pedal, gear, drive_mode)
        return self.state

    # ---------- 通信监控 ----------
    def _update_communication(self, dt_ms, bms_alive, mcu_alive):
        self._bms_tick_ms = 0 if bms_alive else self._bms_tick_ms + dt_ms
        self._mcu_tick_ms = 0 if mcu_alive else self._mcu_tick_ms + dt_ms
        if self._bms_tick_ms > COMM_TIMEOUT_MS:
            self.fault_code |= 0x0001      # bit0: BMS 通信丢失
        else:
            self.fault_code &= ~0x0001
        if self._mcu_tick_ms > COMM_TIMEOUT_MS:
            self.fault_code |= 0x0002      # bit1: MCU 通信丢失
        else:
            self.fault_code &= ~0x0002

    # ---------- 故障诊断 ----------
    def _update_fault(self, soc, mcu_overtemp):
        # SOC 低故障带迟滞，防止在阈值附近反复跳变
        if soc < SOC_LOW_THRESHOLD:
            self._soc_low_latched = True
        elif soc > SOC_RECOVER_THRESHOLD:
            self._soc_low_latched = False

        if self.fault_code & 0x0003:        # 通信丢失 -> 停车
            self.fault_level = FaultLevel.STOP
        elif mcu_overtemp:
            self.fault_code |= 0x0004
            self.fault_level = FaultLevel.LIMIT_TORQUE
        elif self._soc_low_latched:
            self.fault_code |= 0x0008
            self.fault_level = FaultLevel.LIMIT_TORQUE
        else:
            self.fault_code &= ~0x0004
            self.fault_code &= ~0x0008
            self.fault_level = FaultLevel.NO_FAULT

    # ---------- 状态迁移（Stateflow 等价逻辑） ----------
    def _state_transition(self, dt_ms, batt_voltage, bus_voltage, soc,
                          accel_pedal, brake_pedal, gear, key_state):
        prev = self.state

        if self.fault_level == FaultLevel.STOP:
            self.state = VcuState.FAULT
            self.ready = False
            self.hv_enable = False
        elif self.state == VcuState.INIT:
            # 上电自检通过 + Key ON -> 进入预充
            if key_state >= 2 and self.fault_level == FaultLevel.NO_FAULT:
                self.state = VcuState.PRECHARGE
                self.hv_enable = True
        elif self.state == VcuState.PRECHARGE:
            if bus_voltage >= batt_voltage * PRECHARGE_RATIO and batt_voltage > 0:
                self._precharge_ok_ms += dt_ms
                if self._precharge_ok_ms >= PRECHARGE_HOLD_MS:
                    self.state = VcuState.READY
                    self.ready = True
                    self.fault_code &= ~0x0010
            else:
                self._precharge_ok_ms = 0
                # 预充超时 5 s 判故障
                if self._precharge_ok_ms == 0 and dt_ms and False:
                    pass
            if self._precharge_ms_exceeded(dt_ms, bus_voltage, batt_voltage):
                self.fault_code |= 0x0010     # bit4: 预充失败
                self.fault_level = FaultLevel.STOP
                self.state = VcuState.FAULT
        elif self.state == VcuState.READY:
            if key_state < 2:
                self.state = VcuState.SHUTDOWN
                self.ready = False
                self.hv_enable = False
            elif brake_pedal > 5 and gear in (Gear.D, Gear.R):
                self.state = VcuState.REGEN
            elif accel_pedal > 10 and gear in (Gear.D, Gear.R):
                self.state = VcuState.DRIVING
        elif self.state == VcuState.DRIVING:
            if key_state < 2:
                self.state = VcuState.SHUTDOWN
            elif brake_pedal > 5:
                self.state = VcuState.REGEN
            elif accel_pedal <= 10:
                self.state = VcuState.READY
        elif self.state == VcuState.REGEN:
            if key_state < 2:
                self.state = VcuState.SHUTDOWN
            elif soc >= SOC_REGEN_DISABLE or brake_pedal <= 5:
                self.state = VcuState.READY if accel_pedal <= 10 else VcuState.DRIVING
        elif self.state == VcuState.FAULT:
            # 故障恢复：通信恢复且 Key 重新 ON，回初始化重新预充
            if self.fault_level == FaultLevel.NO_FAULT and key_state >= 2:
                self.state = VcuState.INIT
                self._precharge_ok_ms = 0
        elif self.state == VcuState.SHUTDOWN:
            if key_state >= 2:
                self.state = VcuState.INIT
                self._precharge_ok_ms = 0

        if prev != self.state:
            self.log.append((prev.name, self.state.name))

    def _precharge_ms_exceeded(self, dt_ms, bus_voltage, batt_voltage):
        """预充 5 s 未完成判失败（内部计时在 _precharge_ok_ms 归零时需另计时）"""
        if not hasattr(self, '_precharge_elapsed_ms'):
            self._precharge_elapsed_ms = 0
        if self.state == VcuState.PRECHARGE:
            self._precharge_elapsed_ms += dt_ms
        else:
            self._precharge_elapsed_ms = 0
        return self._precharge_elapsed_ms > 5000

    # ---------- 扭矩控制 ----------
    def _torque_control(self, soc, accel_pedal, brake_pedal, gear, drive_mode):
        self.torque_req = 0.0
        self.regen_enable = False

        if self.state not in (VcuState.DRIVING, VcuState.REGEN):
            self.torque_limit_max = 0.0
            self.torque_limit_min = 0.0
            return

        base_torque = TORQUE_LIMIT_BY_GEAR.get(gear, 0.0)
        # 加速踏板 MAP -> 请求扭矩
        ratio = _interp(accel_pedal, PEDAL_TORQUE_MAP)
        torque = base_torque * ratio * MODE_TORQUE_SCALE.get(drive_mode, 1.0)

        if self.state == VcuState.REGEN:
            # 制动回收扭矩：制动踏板 100% -> 最大 300 Nm 回收
            regen = -min(300.0, brake_pedal * 3.0)
            if soc < SOC_REGEN_DISABLE:
                self.torque_req = regen
                self.regen_enable = True
            # SOC 过高时 torque_req 保持 0，禁止回收

        # 故障限扭
        if self.fault_level == FaultLevel.LIMIT_TORQUE:
            torque *= 0.5
        elif self.fault_level == FaultLevel.STOP:
            torque = 0.0

        self.torque_req += torque

        # 限幅：受 SOC 与挡位约束
        self.torque_limit_max = base_torque
        self.torque_limit_min = -300.0 if soc < SOC_REGEN_DISABLE else 0.0
        self.torque_req = max(self.torque_limit_min,
                              min(self.torque_limit_max, self.torque_req))


if __name__ == "__main__":
    vcu = VehicleControlUnit()
    for i in range(400):
        vcu.step(dt_ms=10, batt_voltage=350, bus_voltage=350, soc=80,
                 accel_pedal=30 if i > 120 else 0, brake_pedal=0,
                 gear=Gear.D, key_state=2, drive_mode=DriveMode.NORMAL)
        if i in (0, 50, 120, 200, 399):
            print("t=%4dms state=%-10s torque=%7.1f Nm fault=%d" %
                  (i * 10, vcu.state.name, vcu.torque_req, vcu.fault_level))
    print("state transitions:", vcu.log)
