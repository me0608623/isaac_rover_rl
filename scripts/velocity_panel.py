"""三視角合成右下角的資訊欄：文字＋即時線速度／角速度曲線（2026-09-25）。

速度一律從**真值位姿**（pose.csv，30 Hz）算，不用 policy 的指令：
影片要呈現的是車「實際」怎麼動（含加速度限制、差速控制器的延遲）。

    線速度 v = 位移在車頭方向的投影 / dt   （倒車為負）
    角速度 ω = 朝向變化 / dt              （逆時針為正）

數值微分會有鋸齒，取置中移動平均 SMOOTH_S 秒。
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np

#: 平滑視窗（秒）。0.3 s 足以去掉 30 Hz 微分的鋸齒，又不會把轉向反應抹平。
SMOOTH_S = 0.3
#: 曲線顯示最近多少秒
WINDOW_S = 10.0
#: 縱軸範圍：車速上限 0.7 m/s（允許倒車）、policy 角速度上限 1.2 rad/s
V_RANGE = (-0.4, 1.1)
W_RANGE = (-1.3, 1.3)


def yaw_from_quat(qw, qx, qy, qz):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def velocities(t, x, y, yaw, smooth_s: float = SMOOTH_S):
    """回傳 ``(v, w)``（與 t 等長）。t 單調遞增；yaw 允許跨 ±π。"""
    t = np.asarray(t, float)
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    yaw = np.unwrap(np.asarray(yaw, float))
    if len(t) < 2:
        return np.zeros(len(t)), np.zeros(len(t))
    dt = np.gradient(t)
    dt[dt <= 0] = np.nan
    vx, vy = np.gradient(x) / dt, np.gradient(y) / dt
    v = vx * np.cos(yaw) + vy * np.sin(yaw)
    w = np.gradient(yaw) / dt
    v, w = np.nan_to_num(v), np.nan_to_num(w)
    med = float(np.nanmedian(np.diff(t)))
    k = max(1, int(round(smooth_s / med))) if med > 0 else 1
    if k > 1:
        ker = np.ones(k) / k
        v = np.convolve(v, ker, mode="same")
        w = np.convolve(w, ker, mode="same")
    return v, w


def load_pose(path: Path):
    t, x, y, yaw = [], [], [], []
    for r in csv.DictReader(open(path)):
        t.append(float(r["t"]))
        x.append(float(r["x"]))
        y.append(float(r["y"]))
        yaw.append(yaw_from_quat(float(r["qw"]), float(r["qx"]),
                                 float(r["qy"]), float(r["qz"])))
    return np.array(t), np.array(x), np.array(y), np.array(yaw)


def load_frame_times(path: Path):
    return np.array([float(r["sim_time"]) for r in csv.DictReader(open(path))])


class PanelRenderer:
    """每一格畫一張 W×H 的資訊欄（RGB bytes）。靜態文字只畫一次。"""

    BG = (27, 35, 32)
    FG = (240, 244, 242)
    MUTED = (140, 158, 151)
    GRID = (55, 68, 63)
    V_COL = (95, 205, 175)      # 線速度：青綠
    W_COL = (240, 170, 90)      # 角速度：橘

    def __init__(self, lines, pose_t, v, w, width=960, height=540,
                 font_path="/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc"):
        from PIL import Image, ImageDraw, ImageFont

        self.W, self.H = width, height
        self.t, self.v, self.w = pose_t, v, w
        self.f_txt = ImageFont.truetype(font_path, 25)
        self.f_big = ImageFont.truetype(font_path, 30)
        self.f_small = ImageFont.truetype(font_path, 19)
        base = Image.new("RGB", (width, height), self.BG)
        d = ImageDraw.Draw(base)
        # 文字分兩欄，把下半部留給曲線
        half = (len(lines) + 1) // 2
        for col, chunk in enumerate((lines[:half], lines[half:])):
            yy = 20
            for ln in chunk:
                d.text((36 + col * 450, yy), ln, font=self.f_txt, fill=self.FG)
                yy += 33
        self.base = base
        # 兩個曲線框
        self.boxes = {"v": (90, 212, width - 36, 356), "w": (90, 400, width - 36, 526)}

    def _plot(self, d, key, tt, series, lo, hi, col, label, unit, refs):
        x0, y0, x1, y1 = self.boxes[key]
        d.rectangle((x0, y0, x1, y1), outline=self.GRID)

        def ypix(val):
            val = min(hi, max(lo, val))
            return y1 - (val - lo) / (hi - lo) * (y1 - y0)
        for r in refs:                                   # 參考線＋刻度
            yy = ypix(r)
            d.line((x0, yy, x1, yy), fill=self.GRID)
            d.text((x0 - 8, yy), f"{r:g}", font=self.f_small, fill=self.MUTED, anchor="rm")
        m = (self.t >= tt - WINDOW_S) & (self.t <= tt)
        ts, vs = self.t[m], series[m]
        if len(ts) > 1:
            px = x0 + (ts - (tt - WINDOW_S)) / WINDOW_S * (x1 - x0)
            d.line(list(zip(px.tolist(), [ypix(s) for s in vs])), fill=col, width=3)
        cur = float(np.interp(tt, self.t, series)) if len(self.t) else 0.0
        # 標籤與當下數值放在框外上方，不會被曲線蓋住
        d.text((x0, y0 - 6), label, font=self.f_small, fill=self.MUTED, anchor="ld")
        d.text((x1, y0 - 4), f"{cur:+.2f} {unit}", font=self.f_big, fill=col, anchor="rd")

    def frame(self, sim_t: float, video_t: float) -> bytes:
        from PIL import ImageDraw

        img = self.base.copy()
        d = ImageDraw.Draw(img)
        self._plot(d, "v", sim_t, self.v, *V_RANGE, self.V_COL,
                   f"線速度（真值，最近 {WINDOW_S:.0f} 秒）", "m/s", (0.0, 0.7))
        self._plot(d, "w", sim_t, self.w, *W_RANGE, self.W_COL,
                   f"角速度（真值，最近 {WINDOW_S:.0f} 秒）", "rad/s", (-1.0, 0.0, 1.0))
        d.text((self.W - 36, 20), f"{video_t:5.1f} s", font=self.f_txt,
               fill=self.V_COL, anchor="ra")
        return img.tobytes()
