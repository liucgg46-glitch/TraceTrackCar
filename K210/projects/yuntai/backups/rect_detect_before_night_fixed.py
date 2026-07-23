# -*- coding: utf-8 -*-

import sensor
import image
import time
import math
import gc


# ============================================================
# 1. 黑胶带矩形识别参数
# ============================================================

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

# 只覆盖靶纸允许移动、旋转的区域，排除右侧桌椅和画面边缘。
RECT_SEARCH_ROI = (60, 30, 200, 180)

# 根据当前现场标定成功的黑胶带LAB亮度上限。
BLACK_L_MAX = 15
BLACK_THRESHOLD = (
    0, BLACK_L_MAX,
    -128, 127,
    -128, 127
)

# 允许黑胶带边缘的局部断开，并将相邻边缘合并。
BLOB_PIXELS_THRESHOLD = 25
BLOB_AREA_THRESHOLD = 80
BLOB_MERGE_MARGIN = 12

# 1米距离下，移动或旋转A4黑框时允许的外接框范围。
MIN_BOX_AREA = 1800
MAX_BOX_AREA = 11000
MIN_BOX_WIDTH = 25
MIN_BOX_HEIGHT = 15
MAX_BOX_WIDTH = 160
MAX_BOX_HEIGHT = 140

# 黑胶带是空心框。旋转后水平包围框会变大，所以密度会降低。
MIN_DENSITY_PERCENT = 8
MAX_DENSITY_PERCENT = 45

# 水平包围框的长宽比。旋转接近45度时可能接近1.0。
MIN_ASPECT_X100 = 90
MAX_ASPECT_X100 = 230

# 当前1米距离下的成功标定值。
REFERENCE_PIXELS = 1134
REFERENCE_DENSITY = 25
REFERENCE_ASPECT_X100 = 144
REFERENCE_BOX_AREA = 4536

# 允许的黑胶带实际像素数量。
# 相同距离下，旋转只会改变外接框，胶带像素数基本保持稳定。
MIN_TAPE_PIXELS = 650
MAX_TAPE_PIXELS = 1800

# 识别出矩形后，在四角包围区域外扩15像素，作为后续激光ROI。
LASER_ROI_MARGIN = 35

PRINT_INTERVAL = 10

# 连续稳定5帧后才锁定矩形。
LOCK_REQUIRED_FRAMES = 5

# 单帧之间每个角允许的最大移动量，单位为像素。
LOCK_FRAME_MAX_MOVE = 5

# 从本轮确认第1帧到当前帧，每个角允许的最大漂移量。
# 这样可以避免有人缓慢移动靶纸时被错误锁定。
LOCK_ANCHOR_MAX_MOVE = 7


# ============================================================
# 2. 摄像头初始化
# ============================================================

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)
sensor.skip_frames(time=2000)

# 当前现场已经验证可以清楚看到黑框的参数。
sensor.set_auto_gain(False, gain_db=16)
sensor.set_auto_whitebal(False)
auto_exposure_us = sensor.get_exposure_us()
fixed_exposure_us = (auto_exposure_us * 2) // 3

if fixed_exposure_us < 60000:
    fixed_exposure_us = 60000

if fixed_exposure_us > 120000:
    fixed_exposure_us = 120000

print('RECT auto exposure:', auto_exposure_us)
print('RECT fixed exposure:', fixed_exposure_us)

sensor.set_auto_exposure(
    False,
    exposure_us=fixed_exposure_us
)
sensor.skip_frames(time=1000)

clock = time.clock()
frame_count = 0

# 矩形确认与锁定状态。
rect_locked = False
confirm_count = 0
confirm_anchor_corners = None
confirm_filtered_corners = None
locked_corners = None
locked_center_x = 0
locked_center_y = 0


# ============================================================
# 3. 基础几何函数
# ============================================================

def absolute_value(value):
    if value < 0:
        return -value

    return value


def sort_corners_clockwise(corners):
    """
    四角按照图像中的顺时针方向排列。
    第0项为x+y最小的角，通常对应左上角。
    """
    center_x = 0
    center_y = 0

    for point in corners:
        center_x += point[0]
        center_y += point[1]

    center_x //= 4
    center_y //= 4

    angle_points = []

    for point in corners:
        angle = math.atan2(
            point[1] - center_y,
            point[0] - center_x
        )
        angle_points.append([angle, point])

    # 简单排序，兼容K210 MicroPython。
    for i in range(3):
        for j in range(i + 1, 4):
            if angle_points[j][0] < angle_points[i][0]:
                temp = angle_points[i]
                angle_points[i] = angle_points[j]
                angle_points[j] = temp

    ordered = []

    for item in angle_points:
        ordered.append(item[1])

    start_index = 0
    min_sum = ordered[0][0] + ordered[0][1]

    for i in range(1, 4):
        current_sum = ordered[i][0] + ordered[i][1]

        if current_sum < min_sum:
            min_sum = current_sum
            start_index = i

    result = []

    for i in range(4):
        result.append(ordered[(start_index + i) % 4])

    return result, center_x, center_y


def get_blob_corners(blob):
    """
    兼容当前CanMV K210旧固件。

    新固件若支持min_corners则直接使用；否则使用blob.rotation()、
    外接框宽高和A4长宽比估算旋转四角；如果rotation也不可用，
    最后回退为水平外接框。
    """
    try:
        corners = blob.min_corners()

        if corners is not None and len(corners) == 4:
            return corners
    except Exception:
        pass

    try:
        angle = blob.rotation()
        center_x = blob.cx()
        center_y = blob.cy()
        box_width = blob.w()
        box_height = blob.h()

        cos_value = math.cos(angle)
        sin_value = math.sin(angle)
        abs_cos = absolute_value(cos_value)
        abs_sin = absolute_value(sin_value)

        a4_ratio = 1.414
        denominator_x = a4_ratio * abs_cos + abs_sin
        denominator_y = a4_ratio * abs_sin + abs_cos

        short_from_x = 0.0
        short_from_y = 0.0
        estimate_count = 0

        if denominator_x > 0.01:
            short_from_x = box_width / denominator_x
            estimate_count += 1

        if denominator_y > 0.01:
            short_from_y = box_height / denominator_y
            estimate_count += 1

        if estimate_count == 2:
            short_side = (short_from_x + short_from_y) * 0.5
        elif estimate_count == 1:
            short_side = short_from_x + short_from_y
        else:
            raise ValueError("invalid rectangle projection")

        long_side = short_side * a4_ratio
        half_long = long_side * 0.5
        half_short = short_side * 0.5

        long_x = cos_value
        long_y = sin_value
        short_x = -sin_value
        short_y = cos_value

        return (
            (
                int(center_x - half_long * long_x - half_short * short_x),
                int(center_y - half_long * long_y - half_short * short_y)
            ),
            (
                int(center_x + half_long * long_x - half_short * short_x),
                int(center_y + half_long * long_y - half_short * short_y)
            ),
            (
                int(center_x + half_long * long_x + half_short * short_x),
                int(center_y + half_long * long_y + half_short * short_y)
            ),
            (
                int(center_x - half_long * long_x + half_short * short_x),
                int(center_y - half_long * long_y + half_short * short_y)
            )
        )

    except Exception:
        x = blob.x()
        y = blob.y()
        width = blob.w()
        height = blob.h()

        return (
            (x, y),
            (x + width - 1, y),
            (x + width - 1, y + height - 1),
            (x, y + height - 1)
        )


# ============================================================
# 4. 候选过滤和目标选择
# ============================================================

def candidate_is_valid(blob):
    width = blob.w()
    height = blob.h()
    box_area = width * height
    pixels = blob.pixels()

    if width < MIN_BOX_WIDTH or width > MAX_BOX_WIDTH:
        return False, box_area, 0, 0, pixels

    if height < MIN_BOX_HEIGHT or height > MAX_BOX_HEIGHT:
        return False, box_area, 0, 0, pixels

    if box_area < MIN_BOX_AREA or box_area > MAX_BOX_AREA:
        return False, box_area, 0, 0, pixels

    if pixels < MIN_TAPE_PIXELS or pixels > MAX_TAPE_PIXELS:
        return False, box_area, 0, 0, pixels

    density_percent = (pixels * 100) // box_area

    if density_percent < MIN_DENSITY_PERCENT:
        return False, box_area, density_percent, 0, pixels

    if density_percent > MAX_DENSITY_PERCENT:
        return False, box_area, density_percent, 0, pixels

    long_side = width
    short_side = height

    if height > width:
        long_side = height
        short_side = width

    if short_side <= 0:
        return False, box_area, density_percent, 0, pixels

    aspect_x100 = (long_side * 100) // short_side

    if aspect_x100 < MIN_ASPECT_X100:
        return False, box_area, density_percent, aspect_x100, pixels

    if aspect_x100 > MAX_ASPECT_X100:
        return False, box_area, density_percent, aspect_x100, pixels

    return True, box_area, density_percent, aspect_x100, pixels


def calculate_candidate_score(
    pixels,
    density,
    aspect_x100,
    box_area
):
    pixel_error = absolute_value(pixels - REFERENCE_PIXELS)
    density_error = absolute_value(density - REFERENCE_DENSITY)
    aspect_error = absolute_value(
        aspect_x100 - REFERENCE_ASPECT_X100
    )
    area_error = absolute_value(box_area - REFERENCE_BOX_AREA)

    # 1米距离固定时，胶带像素数最稳定，因此权重最高。
    # 密度和外接框面积会随旋转变化，只作为辅助判断。
    return (
        100000
        - pixel_error * 40
        - density_error * 80
        - aspect_error * 10
        - area_error // 4
    )


def find_best_black_frame(img):
    blobs = img.find_blobs(
        [BLACK_THRESHOLD],
        roi=RECT_SEARCH_ROI,
        pixels_threshold=BLOB_PIXELS_THRESHOLD,
        area_threshold=BLOB_AREA_THRESHOLD,
        merge=False,
        margin=0
    )

    raw_count = len(blobs)
    valid_count = 0

    best_blob = None
    best_corners = None
    best_center_x = 0
    best_center_y = 0
    best_box_area = 0
    best_density = 0
    best_aspect_x100 = 0
    best_pixels = 0
    best_score = -1000000

    for blob in blobs:
        # 黄色表示所有达到find_blobs初步要求的黑色区域。
        img.draw_rectangle(
            blob.rect(),
            color=(255, 255, 0),
            thickness=1
        )

        (
            valid,
            box_area,
            density,
            aspect_x100,
            pixels
        ) = candidate_is_valid(blob)

        diagnostic_width = blob.w()
        diagnostic_height = blob.h()
        diagnostic_area = (
            diagnostic_width * diagnostic_height
        )
        diagnostic_pixels = blob.pixels()

        if diagnostic_area > 0:
            diagnostic_density = (
                diagnostic_pixels * 100
            ) // diagnostic_area
        else:
            diagnostic_density = 0

        diagnostic_long = diagnostic_width
        diagnostic_short = diagnostic_height

        if diagnostic_height > diagnostic_width:
            diagnostic_long = diagnostic_height
            diagnostic_short = diagnostic_width

        if diagnostic_short > 0:
            diagnostic_aspect = (
                diagnostic_long * 100
            ) // diagnostic_short
        else:
            diagnostic_aspect = 0

        print(
            "RECT RAW valid=%d "
            "box=(%d,%d,%d,%d) "
            "area=%d pixels=%d density=%d aspect=%d"
            % (
                1 if valid else 0,
                blob.x(),
                blob.y(),
                diagnostic_width,
                diagnostic_height,
                diagnostic_area,
                diagnostic_pixels,
                diagnostic_density,
                diagnostic_aspect
            )
        )

        if not valid:
            continue

        raw_corners = get_blob_corners(blob)

        if raw_corners is None:
            continue

        corners, center_x, center_y = sort_corners_clockwise(
            raw_corners
        )

        valid_count += 1

        score = calculate_candidate_score(
            pixels,
            density,
            aspect_x100,
            box_area
        )

        if score > best_score:
            best_score = score
            best_blob = blob
            best_corners = corners
            best_center_x = center_x
            best_center_y = center_y
            best_box_area = box_area
            best_density = density
            best_aspect_x100 = aspect_x100
            best_pixels = pixels

    return (
        best_blob,
        best_corners,
        best_center_x,
        best_center_y,
        best_box_area,
        best_density,
        best_aspect_x100,
        best_pixels,
        best_score,
        raw_count,
        valid_count
    )


# ============================================================
# 5. 动态激光ROI和绘制
# ============================================================

def make_laser_roi(corners):
    min_x = corners[0][0]
    max_x = corners[0][0]
    min_y = corners[0][1]
    max_y = corners[0][1]

    for point in corners:
        if point[0] < min_x:
            min_x = point[0]

        if point[0] > max_x:
            max_x = point[0]

        if point[1] < min_y:
            min_y = point[1]

        if point[1] > max_y:
            max_y = point[1]

    min_x -= LASER_ROI_MARGIN
    min_y -= LASER_ROI_MARGIN
    max_x += LASER_ROI_MARGIN
    max_y += LASER_ROI_MARGIN

    if min_x < 0:
        min_x = 0

    if min_y < 0:
        min_y = 0

    if max_x >= IMAGE_WIDTH:
        max_x = IMAGE_WIDTH - 1

    if max_y >= IMAGE_HEIGHT:
        max_y = IMAGE_HEIGHT - 1

    return (
        min_x,
        min_y,
        max_x - min_x + 1,
        max_y - min_y + 1
    )


def draw_selected_frame(img, corners, center_x, center_y):
    for i in range(4):
        p1 = corners[i]
        p2 = corners[(i + 1) % 4]

        img.draw_line(
            (p1[0], p1[1], p2[0], p2[1]),
            color=(0, 255, 0),
            thickness=2
        )

        img.draw_circle(
            p1[0],
            p1[1],
            4,
            color=(255, 0, 0),
            thickness=2
        )

        img.draw_string(
            p1[0] + 4,
            p1[1] + 4,
            str(i),
            color=(255, 255, 255),
            scale=1
        )

    img.draw_cross(
        center_x,
        center_y,
        color=(0, 255, 255),
        size=8,
        thickness=2
    )


# ============================================================
# 6. 连续确认、角点对应与锁定
# ============================================================

def copy_corners(corners):
    result = []

    for point in corners:
        result.append((point[0], point[1]))

    return result


def corners_center(corners):
    center_x = 0
    center_y = 0

    for point in corners:
        center_x += point[0]
        center_y += point[1]

    return center_x // 4, center_y // 4


def align_corners_to_reference(corners, reference):
    """
    当前帧和上一帧都是顺时针排列，但第0角可能发生循环切换。
    尝试4种循环偏移，选择总距离最小的一种，保持角点编号连续。
    """
    best_shift = 0
    best_error = None

    for shift in range(4):
        total_error = 0

        for i in range(4):
            point = corners[(i + shift) % 4]
            ref_point = reference[i]
            dx = point[0] - ref_point[0]
            dy = point[1] - ref_point[1]
            total_error += dx * dx + dy * dy

        if best_error is None or total_error < best_error:
            best_error = total_error
            best_shift = shift

    aligned = []

    for i in range(4):
        aligned.append(corners[(i + best_shift) % 4])

    return aligned


def maximum_corner_distance_square(corners1, corners2):
    maximum = 0

    for i in range(4):
        dx = corners1[i][0] - corners2[i][0]
        dy = corners1[i][1] - corners2[i][1]
        distance_sq = dx * dx + dy * dy

        if distance_sq > maximum:
            maximum = distance_sq

    return maximum


def filter_corners(old_corners, new_corners):
    """四角低通滤波：旧值占3/4，新值占1/4。"""
    result = []

    for i in range(4):
        x = (old_corners[i][0] * 3 + new_corners[i][0] + 2) // 4
        y = (old_corners[i][1] * 3 + new_corners[i][1] + 2) // 4
        result.append((x, y))

    return result


def reset_rectangle_confirmation():
    global confirm_count
    global confirm_anchor_corners
    global confirm_filtered_corners

    confirm_count = 0
    confirm_anchor_corners = None
    confirm_filtered_corners = None


def update_rectangle_lock(new_corners):
    """
    返回值：
    SEARCH  表示刚取得第1帧；
    CONFIRM 表示正在累计稳定帧；
    LOCKED  表示本帧完成锁定。
    """
    global rect_locked
    global confirm_count
    global confirm_anchor_corners
    global confirm_filtered_corners
    global locked_corners
    global locked_center_x
    global locked_center_y

    if rect_locked:
        return "LOCKED"

    if confirm_filtered_corners is None:
        confirm_anchor_corners = copy_corners(new_corners)
        confirm_filtered_corners = copy_corners(new_corners)
        confirm_count = 1
        return "SEARCH"

    aligned = align_corners_to_reference(
        new_corners,
        confirm_filtered_corners
    )

    frame_move_sq = maximum_corner_distance_square(
        aligned,
        confirm_filtered_corners
    )
    anchor_move_sq = maximum_corner_distance_square(
        aligned,
        confirm_anchor_corners
    )

    frame_limit_sq = LOCK_FRAME_MAX_MOVE * LOCK_FRAME_MAX_MOVE
    anchor_limit_sq = LOCK_ANCHOR_MAX_MOVE * LOCK_ANCHOR_MAX_MOVE

    if (
        frame_move_sq <= frame_limit_sq
        and anchor_move_sq <= anchor_limit_sq
    ):
        confirm_filtered_corners = filter_corners(
            confirm_filtered_corners,
            aligned
        )
        confirm_count += 1
    else:
        # 靶纸仍在移动，使用当前帧重新开始计数。
        confirm_anchor_corners = copy_corners(aligned)
        confirm_filtered_corners = copy_corners(aligned)
        confirm_count = 1
        return "SEARCH"

    if confirm_count >= LOCK_REQUIRED_FRAMES:
        locked_corners = copy_corners(confirm_filtered_corners)
        locked_center_x, locked_center_y = corners_center(
            locked_corners
        )
        rect_locked = True
        return "LOCKED"

    return "CONFIRM"


# ============================================================
# 7. 主循环
# ============================================================

# 保存锁定前最后一次有效识别的特征，供锁定日志使用。
last_box_area = 0
last_density = 0
last_aspect_x100 = 0
last_tape_pixels = 0
last_candidate_score = 0

def lock_rectangle(send_invalid):
    global frame_count

    print("RECT module start", gc.mem_free())

    while True:
        clock.tick()
        img = sensor.snapshot()

        if img is None:
            continue

        frame_count += 1

        (
            best_blob,
            corners,
            center_x,
            center_y,
            box_area,
            density,
            aspect_x100,
            tape_pixels,
            candidate_score,
            raw_count,
            valid_count
        ) = find_best_black_frame(img)

        send_invalid(0, 0, False)

        img.draw_rectangle(
            RECT_SEARCH_ROI,
            color=(0, 0, 255),
            thickness=1
        )

        if corners is not None:
            update_rectangle_lock(corners)

            if rect_locked:
                draw_selected_frame(
                    img,
                    locked_corners,
                    locked_center_x,
                    locked_center_y
                )

                laser_roi = make_laser_roi(locked_corners)

                print(
                    "RECT LOCKED p0=%s p1=%s p2=%s p3=%s ROI=%s"
                    % (
                        locked_corners[0],
                        locked_corners[1],
                        locked_corners[2],
                        locked_corners[3],
                        laser_roi
                    )
                )

                result_corners = copy_corners(locked_corners)
                gc.collect()
                print("RECT return", gc.mem_free())
                return result_corners, laser_roi

            draw_selected_frame(
                img,
                confirm_filtered_corners,
                center_x,
                center_y
            )
        else:
            reset_rectangle_confirmation()

        if frame_count >= PRINT_INTERVAL:
            frame_count = 0
            gc.collect()
            print(
                "RECT confirm=%d/%d raw=%d valid=%d free=%d"
                % (
                    confirm_count,
                    LOCK_REQUIRED_FRAMES,
                    raw_count,
                    valid_count,
                    gc.mem_free()
                )
            )

        img.draw_string(
            2,
            220,
            "FPS:%.1f" % clock.fps(),
            color=(255, 255, 255),
            scale=1
        )
