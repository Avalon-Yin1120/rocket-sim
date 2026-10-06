# -*- coding: utf-8 -*-
"""渲染小工具"""
import math


def l2w(pos, theta, lx, ly):
    """本地坐标 (lx, ly) → 世界坐标 (wx, wy)"""
    c = math.cos(theta)
    s = math.sin(theta)
    return (pos.x + lx * c + ly * s, pos.y - lx * s + ly * c)