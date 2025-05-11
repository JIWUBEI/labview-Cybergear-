import math
from typing import Tuple

def inverse_kinematics(x: float, y: float, L1: float = 0.3, L2: float = 0.25, cfg: int = 0) -> Tuple[float, float]:
    """
    计算共轴双电机二维二连杆结构的逆解 (Inverse Kinematics)

    输入:
        x (float): 目标点 X 坐标（单位: 米）
        y (float): 目标点 Y 坐标（单位: 米）
        L1 (float): 连杆 C 长度（单位: 米），默认 0.3
        L2 (float): 连杆 D 长度（单位: 米），默认 0.25
        cfg (int): 解的选择（0: elbow-down, 1: elbow-up），默认 0

    输出:
        theta1 (float): 电机 A 的旋转角度（单位: 弧度）
        theta2 (float): 电机 B 的旋转角度（单位: 弧度）
    """
    # 计算 D，基于余弦定理
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)

    # 检查目标点是否可达
    if abs(D) > 1:
        raise ValueError("目标点超出两段连杆的最大工作范围")

    # 第二节相对角度
    theta2_rel = math.acos(D) if cfg == 0 else -math.acos(D)

    # 计算第一节角度
    k1 = L1 + L2 * math.cos(theta2_rel)
    k2 = L2 * math.sin(theta2_rel)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)

    # 计算第二节“绝对角度”（对地）
    theta2 = theta1 + theta2_rel

    return theta1, theta2
