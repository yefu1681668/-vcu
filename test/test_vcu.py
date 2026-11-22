# -*- coding: utf-8 -*-
"""
VCU 控制策略单元测试（无需 MATLAB，任意 Python 环境可运行）

运行: python -m unittest discover -s test -v
覆盖：预充握手、扭矩 MAP、能量回收、SOC 限幅、通信丢失故障、迟滞、故障恢复
"""

import unittest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from vcu_core import (VehicleControlUnit, VcuState, FaultLevel, Gear,
                      DriveMode, SOC_LOW_THRESHOLD, SOC_REGEN_DISABLE)


def drive(vcu, steps=1, **kw):
    """默认参数跑若干步，便于测试里覆盖个别输入"""
    defaults = dict(dt_ms=10, batt_voltage=350, bus_voltage=350, soc=80,
                    accel_pedal=0, brake_pedal=0, gear=Gear.D,
                    key_state=2, drive_mode=DriveMode.NORMAL)
    defaults.update(kw)
    for _ in range(steps):
        vcu.step(**defaults)
    return vcu


def to_ready(vcu, soc=80):
    """把 VCU 推到 READY 状态（预充 100 ms 保持）"""
    drive(vcu, 12, soc=soc)          # INIT->PRECHARGE->READY
    assert vcu.state == VcuState.READY, "预充握手失败"


class TestPowerOnSequence(unittest.TestCase):
    def test_01_precharge_handshake(self):
        """上电自检通过后进入预充，母线达到 95% 并保持 100ms 才 READY"""
        vcu = VehicleControlUnit()
        drive(vcu, 1, bus_voltage=0)          # 母线未建立
        self.assertEqual(vcu.state, VcuState.PRECHARGE)
        self.assertFalse(vcu.ready)
        drive(vcu, 12, bus_voltage=350)       # 母线到位，保持 100ms+
        self.assertEqual(vcu.state, VcuState.READY)
        self.assertTrue(vcu.ready)

    def test_02_precharge_voltage_not_enough(self):
        """母线电压不足 95% 时不得进入 READY"""
        vcu = VehicleControlUnit()
        drive(vcu, 100, bus_voltage=300)      # 300/350 = 85.7% < 95%
        self.assertEqual(vcu.state, VcuState.PRECHARGE)

    def test_03_key_off_to_shutdown(self):
        """READY 状态下 Key OFF 应立即下电"""
        vcu = VehicleControlUnit()
        to_ready(vcu)
        drive(vcu, 1, key_state=0)
        self.assertEqual(vcu.state, VcuState.SHUTDOWN)
        self.assertFalse(vcu.ready)


class TestTorqueControl(unittest.TestCase):
    def test_04_pedal_deadzone(self):
        """加速踏板 10% 以下为死区，不输出扭矩"""
        vcu = VehicleControlUnit()
        to_ready(vcu)
        drive(vcu, 1, accel_pedal=8)
        self.assertEqual(vcu.torque_req, 0.0)
        self.assertEqual(vcu.state, VcuState.READY)

    def test_05_pedal_map_linear(self):
        """踏板 MAP 线性插值：50% -> 45% 满扭矩 = 540 Nm"""
        vcu = VehicleControlUnit()
        to_ready(vcu)
        drive(vcu, 1, accel_pedal=50)
        self.assertEqual(vcu.state, VcuState.DRIVING)
        self.assertAlmostEqual(vcu.torque_req, 1200 * 0.45, places=1)

    def test_06_sport_mode_scale(self):
        """SPORT 模式扭矩放大 1.15 倍"""
        vcu = VehicleControlUnit()
        to_ready(vcu)
        drive(vcu, 1, accel_pedal=50, drive_mode=DriveMode.SPORT)
        self.assertAlmostEqual(vcu.torque_req, 1200 * 0.45 * 1.15, places=1)

    def test_07_p_torque_zero(self):
        """P 挡不输出驱动扭矩"""
        vcu = VehicleControlUnit()
        to_ready(vcu)
        drive(vcu, 1, accel_pedal=100, gear=Gear.P)
        self.assertEqual(vcu.torque_req, 0.0)


class TestRegen(unittest.TestCase):
    def test_08_brake_enters_regen(self):
        """踩制动进入回收状态并输出负扭矩"""
        vcu = VehicleControlUnit()
        to_ready(vcu)
        drive(vcu, 1, brake_pedal=50)
        self.assertEqual(vcu.state, VcuState.REGEN)
        self.assertLess(vcu.torque_req, 0)
        self.assertTrue(vcu.regen_enable)

    def test_09_soc_high_disables_regen(self):
        """SOC >= 95% 禁止能量回收（扭矩归零）"""
        vcu = VehicleControlUnit()
        to_ready(vcu, soc=SOC_REGEN_DISABLE)
        drive(vcu, 1, brake_pedal=50, soc=SOC_REGEN_DISABLE)
        self.assertEqual(vcu.torque_req, 0.0)
        self.assertFalse(vcu.regen_enable)


class TestFaultManagement(unittest.TestCase):
    def test_10_bms_comm_loss_stop(self):
        """BMS 通信超时 500ms -> STOP 故障，扭矩归零"""
        vcu = VehicleControlUnit()
        to_ready(vcu)
        drive(vcu, 60, bms_alive=False, accel_pedal=50)
        self.assertEqual(vcu.fault_level, FaultLevel.STOP)
        self.assertEqual(vcu.state, VcuState.FAULT)
        self.assertEqual(vcu.torque_req, 0.0)
        self.assertTrue(vcu.fault_code & 0x0001)

    def test_11_soc_low_derate(self):
        """SOC 低于 15% 触发限扭（50%）"""
        vcu = VehicleControlUnit()
        to_ready(vcu, soc=80)
        drive(vcu, 1, soc=10, accel_pedal=50)
        self.assertEqual(vcu.fault_level, FaultLevel.LIMIT_TORQUE)
        # 1200*0.45*0.5 = 270
        self.assertAlmostEqual(vcu.torque_req, 270.0, places=1)

    def test_12_soc_hysteresis(self):
        """SOC 迟滞：低于 15% 锁存，回升到 18% 仍限扭，到 21% 才解除"""
        vcu = VehicleControlUnit()
        to_ready(vcu, soc=80)
        drive(vcu, 1, soc=10)
        self.assertTrue(vcu._soc_low_latched)
        drive(vcu, 1, soc=18)
        self.assertTrue(vcu._soc_low_latched, "18% 不应解除迟滞")
        drive(vcu, 1, soc=21)
        self.assertFalse(vcu._soc_low_latched)

    def test_13_fault_recovery(self):
        """通信恢复后故障可恢复到 INIT 重新预充"""
        vcu = VehicleControlUnit()
        to_ready(vcu)
        drive(vcu, 60, bms_alive=False)
        self.assertEqual(vcu.state, VcuState.FAULT)
        drive(vcu, 1, bms_alive=True, key_state=2)
        self.assertEqual(vcu.state, VcuState.INIT)

    def test_14_mcu_overtemp_derate(self):
        """MCU 过温触发限扭"""
        vcu = VehicleControlUnit()
        to_ready(vcu)
        drive(vcu, 1, accel_pedal=50, mcu_overtemp=True)
        self.assertEqual(vcu.fault_level, FaultLevel.LIMIT_TORQUE)
        self.assertTrue(vcu.fault_code & 0x0004)


if __name__ == "__main__":
    unittest.main(verbosity=2)
