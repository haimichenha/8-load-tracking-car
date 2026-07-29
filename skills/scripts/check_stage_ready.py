from __future__ import annotations

import argparse
import sys
from pathlib import Path

STAGE_RULES = {
    "C2": {
        "description": "负载最小移动 PWM 复测",
        "required_fields": ["time_ms", "stage", "test_pwm", "wheel", "enc_delta", "speed", "moved", "event"],
        "pass_gates": [
            "左右轮都能重复测出稳定移动下限",
            "不能把偶发抖动记为 moved=1",
            "至少完成左轮和右轮各 3 轮测试",
        ],
        "prerequisites": ["C0 电机方向通过", "编码器读数正常", "日志串口可导出"],
    },
    "C3": {
        "description": "开环直行与高 PWM 测试",
        "required_fields": ["time_ms", "stage", "pwm_l", "pwm_r", "enc_l_delta", "enc_r_delta", "speed_l", "speed_r", "duration_ms", "event"],
        "pass_gates": [
            "同 PWM 下左右速度差可记录",
            "速度随 PWM 基本单调增加",
            "1000 PWM 仅做短脉冲测试",
        ],
        "prerequisites": ["C2 已得到左右稳定起转阈值", "测试场地安全", "日志串口可导出"],
    },
    "C3.5": {
        "description": "双轮差速直线平衡 trim 测试",
        "required_fields": ["time_ms", "run_id", "base_pwm", "trim", "pwm_l", "pwm_r", "duration_ms", "enc_l_delta", "enc_r_delta", "speed_l", "speed_r", "event"],
        "pass_gates": [
            "固定 base_pwm=800，trim 从 0 开始按 20 递增或递减",
            "输出满足 left=base-trim，right=base+trim",
            "每个 trim 至少短直行一次并人工记录偏左/偏右/直",
            "找到大致直线组合后再进入速度闭环",
        ],
        "prerequisites": ["C2 已得到负载最小移动 PWM", "C3 已确认短脉冲安全", "PG9..PG13 按键或 USART1 测试命令已独立验证"],
    },
    "C3.6": {
        "description": "编码器单轮/双轮诊断",
        "required_fields": ["time_ms", "run_id", "pwm", "mode", "pwm_l", "pwm_r", "duration_ms", "enc_l_delta", "enc_r_delta", "speed_l", "speed_r", "left_ab", "right_ab", "event"],
        "pass_gates": [
            "单轮 L、单轮 R、双轮 B 和日志导出命令均已映射到当前按键/串口接口",
            "mode=L 时主要看左编码器，mode=R 时主要看右编码器，mode=B 时左右都应有变化",
            "某轮转动但对应 delta 为 0 时，优先查编码器供电、A/B 接线、GPIO 输入配置",
            "某轮转动但另一侧 delta 变化时，优先怀疑左右编码器线接反",
            "delta 方向反只改符号映射，不进入 PID 调参",
        ],
        "prerequisites": ["C0 电机方向通过", "PG9..PG13 按键或 USART1 测试命令已独立触发", "调试串口可导出日志"],
    },
    "C4": {
        "description": "单轮与双轮速度闭环",
        "required_fields": ["time_ms", "run_id", "mode", "target_l", "target_r", "speed_l", "speed_r", "err_l", "err_r", "pwm_l", "pwm_r", "i_l", "i_r", "sat_l", "sat_r", "left_ab", "right_ab", "event"],
        "pass_gates": [
            "C4.2 使用明确记录的低/中两档目标速度或 PWM，分别验证直线和阻挡恢复",
            "启动、停止、档位切换和日志导出均通过当前按键/USART1 命令完成",
            "当前先只开 Kp，Ki=0",
            "连续 3 轮无提升时先查方向/单位/采样周期",
        ],
        "prerequisites": ["C2 已固定死区初值", "C3 已确认开环速度趋势", "编码器速度符号正确"],
    },
    "C5": {
        "description": "转弯动作标定",
        "required_fields": ["time_ms", "stage", "turn_mode", "pwm_l", "pwm_r", "target_l", "target_r", "speed_l", "speed_r", "yaw_start", "yaw_now", "duration_ms", "event"],
        "pass_gates": [
            "原地左转和右转方向可解释",
            "单边死区转向不过度前冲",
            "仅标定动作，不接导航",
        ],
        "prerequisites": ["C4 速度闭环基本可用", "如无可靠 yaw 需注明人工观察", "测试场地安全"],
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check whether an STM32F103ZET6 car stage is ready for testing.")
    parser.add_argument("stage", choices=sorted(STAGE_RULES.keys()), help="Stage name, e.g. C2/C3/C4/C5")
    parser.add_argument("--header", help="Comma-separated CSV header exported by firmware")
    parser.add_argument("--header-file", type=Path, help="Read the first non-empty line as CSV header")
    parser.add_argument("--show-template", action="store_true", help="Print a suggested CSV header")
    return parser.parse_args()


def load_header(args: argparse.Namespace) -> list[str]:
    if args.header:
        return [item.strip() for item in args.header.split(",") if item.strip()]

    if args.header_file:
        if not args.header_file.exists():
            raise FileNotFoundError(f"header file not found: {args.header_file}")
        for line in args.header_file.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line:
                return [item.strip() for item in line.split(",") if item.strip()]
        return []

    return []


def main() -> int:
    args = parse_args()
    rule = STAGE_RULES[args.stage]
    print(f"stage={args.stage}")
    print(f"desc={rule['description']}")
    print("prerequisites:")
    for item in rule["prerequisites"]:
        print(f"- {item}")
    print("pass_gates:")
    for item in rule["pass_gates"]:
        print(f"- {item}")

    header = load_header(args)
    if args.show_template or not header:
        print("csv_template=")
        print(",".join(rule["required_fields"]))

    if not header:
        print("header_check=SKIPPED")
        return 0

    missing = [field for field in rule["required_fields"] if field not in header]
    extra = [field for field in header if field not in rule["required_fields"]]

    print("header_check=OK" if not missing else "header_check=FAIL")
    if missing:
        print("missing_fields=" + ",".join(missing))
    if extra:
        print("extra_fields=" + ",".join(extra))

    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
