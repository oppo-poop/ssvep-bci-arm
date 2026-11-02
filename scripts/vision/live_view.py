#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""后台常驻取流 + 实时显示，给 vision_pick 用。

为什么不用 grasp_xy.grab_frame()：它是"打开设备 → 连读 12 帧丢缓冲 → 关设备"，
中间没有画面。要全程实时，必须有一个线程一直占着设备读帧，
主流程只从中取"最新一帧"分析。
"""
import threading
import time

import cv2

from camera import find_camera_index as find_astra_color_index

W, H = 1280, 720


class LiveView:
    """线程里持续 read + imshow；主线程 set_status / set_target / snapshot。"""

    def __init__(self, win="vision_pick"):
        idx = find_astra_color_index()
        if idx is None:
            raise RuntimeError("找不到 Astra 彩色设备")
        self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        self.win = win
        self.frame = None
        self.status = ""
        self.target = None          # (u, v, 等效半径)
        self._lock = threading.Lock()
        self._stop = False
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()
        t0 = time.time()
        while self.frame is None and time.time() - t0 < 4.0:
            time.sleep(0.05)
        if self.frame is None:
            raise RuntimeError("4 秒内没读到画面")

    def _loop(self):
        try:
            cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.win, 800, int(800 * H / W))
            # 固定到和点选窗口同一位置：点选窗关掉后实时窗就在原处出现，
            # 不然窗口管理器可能把它塞到别的角落，看着像"画面没了"。
            cv2.moveWindow(self.win, 80, 60)
        except Exception:
            pass
        while not self._stop:
            ok, f = self.cap.read()
            if not ok:
                time.sleep(0.02)
                continue
            with self._lock:
                self.frame = f
                tgt, st = self.target, self.status
            view = f.copy()          # 画在副本上，别污染给主流程分析的原始帧
            if tgt is not None:
                u, v, r = tgt
                cv2.circle(view, (int(u), int(v)), int(max(14, r)), (0, 255, 0), 2)
                cv2.drawMarker(view, (int(u), int(v)), (0, 0, 255),
                               cv2.MARKER_CROSS, 30, 2)
                cv2.putText(view, "TARGET", (int(u) + int(max(14, r)) + 6,
                                             int(v) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if st:
                # 状态行统一英文：OpenCV 的 Hershey 字体不支持中文，
                # 写中文只会显示成一排 "?"。先铺一条深色底再写字，
                # 否则画面偏亮时黄字看不清。
                font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2
                (tw, th), _base = cv2.getTextSize(st, font, scale, thick)
                pad = 8
                cv2.rectangle(view, (8, 10), (8 + tw + pad * 2, 10 + th + pad * 2),
                              (32, 32, 32), -1)
                cv2.putText(view, st, (8 + pad, 10 + pad + th), font,
                            scale, (0, 255, 255), thick, cv2.LINE_AA)
            try:
                cv2.imshow(self.win, view)
                cv2.waitKey(1)
            except Exception:
                break               # 没有 GUI 后端就别反复重试

    def snapshot(self):
        with self._lock:
            return None if self.frame is None else self.frame.copy()

    def set_status(self, text):
        with self._lock:
            self.status = text

    def set_target(self, uv, area):
        with self._lock:
            self.target = (float(uv[0]), float(uv[1]),
                           (max(1.0, float(area)) / 3.14159) ** 0.5)

    def close(self):
        """停线程并释放设备。

        必须 join：只 sleep 就返回的话，后台线程可能还在 cv2.imshow，
        主线程紧接着 destroyWindow/开新窗口，Qt 里就是悬空指针 ——
        实测直接 "已杀死"（SIGSEGV/SIGABRT），点选窗口根本弹不出来。
        """
        self._stop = True
        try:
            if self._th.is_alive():
                self._th.join(timeout=1.5)
        except Exception:
            pass
        try:
            cv2.destroyWindow(self.win)
        except Exception:
            pass
        try:
            cv2.waitKey(1)          # 让 destroyWindow 真正生效
        except Exception:
            pass
        try:
            self.cap.release()
        except Exception:
            pass
