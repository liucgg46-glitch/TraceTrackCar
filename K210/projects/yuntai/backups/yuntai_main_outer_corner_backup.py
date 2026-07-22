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

try:
    del sys.modules["rect_detect"]
except Exception:
    pass

del rect_detect
gc.collect()
print("MAIN rectangle released", gc.mem_free())

import laser_detect

target_x = corners[0][0]
target_y = corners[0][1]


def send_laser_and_target(laser_x, laser_y, laser_valid):
    uart_protocol.send_target(
        target_x,
        target_y,
        True
    )

    uart_protocol.send_laser(
        laser_x,
        laser_y,
        laser_valid
    )


print("MAIN fixed target:", target_x, target_y)

laser_detect.run(
    laser_roi,
    corners,
    send_laser_and_target
)
