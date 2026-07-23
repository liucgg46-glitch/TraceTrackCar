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


# Path parameters
PATH_STEP_PIXEL = 4
ARRIVE_DISTANCE_PIXEL = 3
ARRIVE_CONFIRM_FRAMES = 5
TARGET_INSET_DIVISOR = 4


# Calculate rectangle center
center_x = (
    corners[0][0]
    + corners[1][0]
    + corners[2][0]
    + corners[3][0]
) // 4

center_y = (
    corners[0][1]
    + corners[1][1]
    + corners[2][1]
    + corners[3][1]
) // 4


# Move every corner 1/8 toward the rectangle center.
path_corners = []

for corner in corners:
    path_x = (
        corner[0] * (TARGET_INSET_DIVISOR - 1)
        + center_x
    ) // TARGET_INSET_DIVISOR

    path_y = (
        corner[1] * (TARGET_INSET_DIVISOR - 1)
        + center_y
    ) // TARGET_INSET_DIVISOR

    path_corners.append((path_x, path_y))


edge_index = 0
edge_step_index = 0
edge_step_total = 1

target_x = path_corners[0][0]
target_y = path_corners[0][1]

arrive_count = 0
path_complete = False


def start_current_edge():
    global edge_step_index
    global edge_step_total
    global target_x
    global target_y

    start_point = path_corners[edge_index]
    end_point = path_corners[(edge_index + 1) % 4]

    delta_x = end_point[0] - start_point[0]
    delta_y = end_point[1] - start_point[1]

    maximum_delta = abs(delta_x)

    if abs(delta_y) > maximum_delta:
        maximum_delta = abs(delta_y)

    edge_step_total = (
        maximum_delta + PATH_STEP_PIXEL - 1
    ) // PATH_STEP_PIXEL

    if edge_step_total < 1:
        edge_step_total = 1

    edge_step_index = 0
    target_x = start_point[0]
    target_y = start_point[1]


def advance_target():
    global edge_index
    global edge_step_index
    global target_x
    global target_y
    global path_complete

    edge_step_index += 1

    if edge_step_index <= edge_step_total:
        start_point = path_corners[edge_index]
        end_point = path_corners[(edge_index + 1) % 4]

        target_x = start_point[0] + (
            (end_point[0] - start_point[0])
            * edge_step_index
        ) // edge_step_total

        target_y = start_point[1] + (
            (end_point[1] - start_point[1])
            * edge_step_index
        ) // edge_step_total

        print(
            "PATH target edge=%d step=%d/%d point=(%d,%d)"
            % (
                edge_index,
                edge_step_index,
                edge_step_total,
                target_x,
                target_y
            )
        )

        return

    edge_index += 1

    if edge_index >= 4:
        path_complete = True
        target_x = path_corners[0][0]
        target_y = path_corners[0][1]

        print("PATH COMPLETE")
        return

    start_current_edge()


def send_laser_and_target(laser_x, laser_y, laser_valid):
    global arrive_count

    if not path_complete and laser_valid:
        difference_x = target_x - laser_x
        difference_y = target_y - laser_y

        if (
            abs(difference_x) <= ARRIVE_DISTANCE_PIXEL
            and abs(difference_y) <= ARRIVE_DISTANCE_PIXEL
        ):
            arrive_count += 1

            if arrive_count >= ARRIVE_CONFIRM_FRAMES:
                arrive_count = 0
                advance_target()
        else:
            arrive_count = 0
    else:
        arrive_count = 0

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


print("PATH corners:", path_corners)
print("PATH start:", target_x, target_y)

start_current_edge()

laser_detect.run(
    laser_roi,
    corners,
    send_laser_and_target
)
