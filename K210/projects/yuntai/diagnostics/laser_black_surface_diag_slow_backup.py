# -*- coding: utf-8 -*-

import sensor
import time
import gc


IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

# 全黑表面应覆盖这个区域。固定曝光与当前夜间黑框成功参数一致，
# 避免自动曝光因“整幅全黑”而不断改变激光像素外观。
DIAG_ROI = (40, 20, 240, 200)
FIXED_GAIN_DB = 16
FIXED_EXPOSURE_US = 120000

SCAN_STEP = 2
RING_RADIUS = 3
MIN_CONTRAST_SCAN_SUM = 100
PRINT_INTERVAL = 5


def pixel_sum(img, x, y):
    pixel = img.get_pixel(x, y)
    return pixel[0] + pixel[1] + pixel[2]


def local_contrast(img, x, y, center_sum):
    ring_sum = 0
    ring_sum += pixel_sum(img, x - RING_RADIUS, y)
    ring_sum += pixel_sum(img, x + RING_RADIUS, y)
    ring_sum += pixel_sum(img, x, y - RING_RADIUS)
    ring_sum += pixel_sum(img, x, y + RING_RADIUS)
    ring_sum += pixel_sum(img, x - RING_RADIUS, y - RING_RADIUS)
    ring_sum += pixel_sum(img, x + RING_RADIUS, y - RING_RADIUS)
    ring_sum += pixel_sum(img, x - RING_RADIUS, y + RING_RADIUS)
    ring_sum += pixel_sum(img, x + RING_RADIUS, y + RING_RADIUS)
    return center_sum - ring_sum // 8


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

sensor.set_auto_gain(False, gain_db=FIXED_GAIN_DB)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(
    False,
    exposure_us=FIXED_EXPOSURE_US
)

sensor.skip_frames(time=1000)

clock = time.clock()
frame_count = 0

print("BLACK SURFACE LASER DIAG")
print("ROI:", DIAG_ROI)
print("gain db:", FIXED_GAIN_DB)
print("exposure us:", FIXED_EXPOSURE_US)


while True:
    clock.tick()
    img = sensor.snapshot()

    left = DIAG_ROI[0] + RING_RADIUS
    top = DIAG_ROI[1] + RING_RADIUS
    right = DIAG_ROI[0] + DIAG_ROI[2] - 1 - RING_RADIUS
    bottom = DIAG_ROI[1] + DIAG_ROI[3] - 1 - RING_RADIUS

    bright_x = left
    bright_y = top
    bright_sum = -1
    bright_r = 0
    bright_g = 0
    bright_b = 0

    contrast_x = left
    contrast_y = top
    contrast_sum = 0
    contrast_delta = -1000
    contrast_r = 0
    contrast_g = 0
    contrast_b = 0

    for y in range(top, bottom + 1, SCAN_STEP):
        for x in range(left, right + 1, SCAN_STEP):
            pixel = img.get_pixel(x, y)
            r = pixel[0]
            g = pixel[1]
            b = pixel[2]
            current_sum = r + g + b

            if current_sum > bright_sum:
                bright_x = x
                bright_y = y
                bright_sum = current_sum
                bright_r = r
                bright_g = g
                bright_b = b

            if current_sum < MIN_CONTRAST_SCAN_SUM:
                continue

            current_delta = local_contrast(
                img,
                x,
                y,
                current_sum
            )

            if current_delta > contrast_delta:
                contrast_x = x
                contrast_y = y
                contrast_sum = current_sum
                contrast_delta = current_delta
                contrast_r = r
                contrast_g = g
                contrast_b = b

    img.draw_rectangle(
        DIAG_ROI,
        color=(0, 0, 255),
        thickness=1
    )

    # 黄色十字：绝对最亮像素。
    img.draw_cross(
        bright_x,
        bright_y,
        color=(255, 255, 0),
        size=7,
        thickness=1
    )

    # 白色十字：相对周围黑表面增量最大的像素。
    img.draw_cross(
        contrast_x,
        contrast_y,
        color=(255, 255, 255),
        size=9,
        thickness=2
    )

    img.draw_string(
        2,
        2,
        "B:%d,%d S:%d" % (
            bright_x,
            bright_y,
            bright_sum
        ),
        color=(255, 255, 0),
        scale=1
    )

    img.draw_string(
        2,
        14,
        "C:%d,%d D:%d" % (
            contrast_x,
            contrast_y,
            contrast_delta
        ),
        color=(255, 255, 255),
        scale=1
    )

    frame_count += 1

    if frame_count >= PRINT_INTERVAL:
        frame_count = 0
        gc.collect()
        print(
            "black diag "
            "bright=(%d,%d,rgb=%d,%d,%d,sum=%d) "
            "contrast=(%d,%d,rgb=%d,%d,%d,sum=%d,delta=%d) "
            "fps=%.1f mem=%d"
            % (
                bright_x,
                bright_y,
                bright_r,
                bright_g,
                bright_b,
                bright_sum,
                contrast_x,
                contrast_y,
                contrast_r,
                contrast_g,
                contrast_b,
                contrast_sum,
                contrast_delta,
                clock.fps(),
                gc.mem_free()
            )
        )
