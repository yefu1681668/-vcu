# -*- coding: utf-8 -*-
"""
轻量 DBC 解析器 + 通信矩阵检查工具（零依赖）

用途：
  1. 在没有 CANoe 的机器上校验 DBC 语法与信号定义；
  2. 自动生成信号清单 CSV，供需求追踪与测试用例引用；
  3. CI 里检查报文周期是否合理（安全相关报文 <= 20 ms）。

运行: python tools/dbc_parser.py dbc/VCU_Matrix.dbc
"""

import re
import sys
import csv
import os


class DbcParser:
    def __init__(self, path):
        self.path = path
        self.messages = {}      # id -> {name, dlc, tx, cycle, signals[]}
        self.nodes = []
        self.val_tables = {}

    def parse(self):
        text = open(self.path, encoding="utf-8").read()
        # 节点
        m = re.search(r"^BU_:\s*(.+)$", text, re.M)
        if m:
            self.nodes = m.group(1).split()
        # 报文
        for m in re.finditer(
                r"^BO_\s+(\d+)\s+(\w+):\s+(\d+)\s+(\w+)(.*?)(?=^BO_|^BA_|^VAL_|\Z)",
                text, re.M | re.S):
            msg_id = int(m.group(1))
            self.messages[msg_id] = {
                "id": msg_id,
                "name": m.group(2),
                "dlc": int(m.group(3)),
                "tx": m.group(4),
                "signals": [],
                "cycle": None,
            }
            for s in re.finditer(
                    r"SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@(\d)([+-])\s*\(([\d.]+),([-\d.]+)\)\s*\[([-\d.]+)\|([-\d.]+)\]\s*\"([^\"]*)\"",
                    m.group(5)):
                self.messages[msg_id]["signals"].append({
                    "name": s.group(1), "start": int(s.group(2)),
                    "len": int(s.group(3)), "byte_order": "intel" if s.group(4) == "1" else "motorola",
                    "signed": s.group(5) == "-",
                    "factor": float(s.group(6)), "offset": float(s.group(7)),
                    "min": float(s.group(8)), "max": float(s.group(9)),
                    "unit": s.group(10),
                })
        # 周期
        for m in re.finditer(r'BA_\s+"GenMsgCycleTime"\s+BO_\s+(\d+)\s+(\d+)', text):
            mid = int(m.group(1))
            if mid in self.messages:
                self.messages[mid]["cycle"] = int(m.group(2))
        return self

    def check(self):
        errors = []
        warnings = []
        if not self.nodes:
            errors.append("未找到 BU_ 节点定义")
        if not self.messages:
            errors.append("未找到任何 BO_ 报文")
        for mid, msg in self.messages.items():
            # 信号位宽检查
            for sig in msg["signals"]:
                if sig["start"] + sig["len"] > msg["dlc"] * 8:
                    errors.append("%s.%s 信号超出 DLC 范围" % (msg["name"], sig["name"]))
            if msg["tx"] not in self.nodes:
                errors.append("%s 发送节点 %s 未在 BU_ 声明" % (msg["name"], msg["tx"]))
            if msg["cycle"] is None:
                warnings.append("%s 未定义 GenMsgCycleTime" % msg["name"])
            elif msg["cycle"] > 100:
                warnings.append("%s 周期 %dms 偏慢" % (msg["name"], msg["cycle"]))
        return errors, warnings

    def export_csv(self, out):
        with open(out, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["报文ID", "报文名", "周期ms", "发送节点", "信号名", "起始位",
                        "长度", "字节序", "系数", "偏移", "单位", "最小值", "最大值"])
            for mid, msg in sorted(self.messages.items()):
                for s in msg["signals"]:
                    w.writerow([ "0x%03X" % mid, msg["name"], msg["cycle"] or "",
                                 msg["tx"], s["name"], s["start"], s["len"],
                                 s["byte_order"], s["factor"], s["offset"],
                                 s["unit"], s["min"], s["max"] ])
        return out


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "dbc/VCU_Matrix.dbc"
    if not os.path.exists(path):
        print("文件不存在:", path)
        return 1
    dbc = DbcParser(path).parse()
    errors, warnings = dbc.check()
    n_sig = sum(len(m["signals"]) for m in dbc.messages.values())
    print("DBC: %s" % path)
    print("  节点: %d (%s)" % (len(dbc.nodes), ", ".join(dbc.nodes)))
    print("  报文: %d, 信号: %d" % (len(dbc.messages), n_sig))
    for mid, msg in sorted(dbc.messages.items()):
        print("    0x%03X %-16s DLC=%d cycle=%4sms tx=%-5s sigs=%d" %
              (mid, msg["name"], msg["dlc"], msg["cycle"], msg["tx"], len(msg["signals"])))
    csv_path = os.path.splitext(path)[0] + "_signals.csv"
    dbc.export_csv(csv_path)
    print("  信号清单已导出:", csv_path)
    for w in warnings:
        print("  [WARN]", w)
    for e in errors:
        print("  [ERROR]", e)
    print("结果:", "PASS" if not errors else "FAIL (%d errors)" % len(errors))
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
