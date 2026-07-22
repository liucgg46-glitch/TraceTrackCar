# -*- coding: utf-8 -*-

import sys
import gc

if "/sd" not in sys.path:
    sys.path.append("/sd")

gc.collect()
print("MAIN start", gc.mem_free())

import uart_protocol
import rect_detect

corners, laser_roi = rect_detect.lock_rectangle(
    uart_protocol.send_laser
)

print("MAIN rectangle locked", corners, laser_roi)

# 解除矩形模块引用，回收其函数和临时对象。
try:
    del sys.modules["rect_detect"]
except Exception:
    pass

del rect_detect
gc.collect()
print("MAIN rectangle released", gc.mem_free())

import laser_detect

laser_detect.run(
    laser_roi,
    corners,
    uart_protocol.send_laser
)
