# -*- coding: utf-8 -*-

import sensor
import image
import time
import gc


# ============================================================
# 1. 图像参数
# ============================================================

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

# 白板搜索区域
# X：60～259
# Y：40～199
LASER_ROI = (60, 40, 200, 160)


# ============================================================
# 2. 完全失锁时使用的严格阈值
# ============================================================

# 严格红色阈值
LASER_RED_THRESHOLD = (
    35, 100,
    10, 127,
    -20, 127
)

# 严格高亮阈值
LASER_BRIGHT_THRESHOLD = (
    75, 100,
    -25, 25,
    -25, 25
)


# ============================================================
# 3. 已锁定时使用的局部阈值
# ============================================================

# 局部红色阈值
LASER_RED_TRACK_THRESHOLD = (
    30, 100,
    6, 127,
    -25, 127
)

# 局部高亮阈值
LASER_BRIGHT_TRACK_THRESHOLD = (
    68, 100,
    -30, 30,
    -30, 30
)


# ============================================================
# 4. 光斑尺寸限制
# ============================================================

# find_blobs 的基础最小限制
MIN_PIXELS = 1
MIN_AREA = 1

# 最大光斑限制
MAX_WIDTH = 10
MAX_HEIGHT = 10
MAX_AREA = 25

# 强目标判断条件
#
# 满足下面任意一项，就认为是较可靠的光斑：
# pixels >= 4
# area >= 2
STRONG_MIN_PIXELS = 4
STRONG_MIN_AREA = 2

# 完全失锁后重新捕获时的最低要求
REACQUIRE_MIN_PIXELS = 3
REACQUIRE_MIN_AREA = 1


# ============================================================
# 5. 距离限制
# ============================================================

# 正常跟踪时允许的最大移动距离
MAX_MOVE_NORMAL = 16

# 每丢失一帧，允许移动距离增加多少
MAX_MOVE_ADD_PER_LOST = 4

# 恢复阶段最大允许移动距离
MAX_MOVE_RECOVER_MAX = 32

# 弱目标只有在上一坐标附近时才允许接受
WEAK_TARGET_MAX_DISTANCE = 6


# ============================================================
# 6. 丢失处理
# ============================================================

# 连续丢失5帧后彻底失锁
LOST_MAX_COUNT = 10

# 短暂丢失1～3帧时继续发送上一可靠坐标，避免STM32状态频繁跳变。
SEND_HOLD_MAX_LOST = 3

# 全局捕获时必须连续多帧出现在相近位置，避免单帧噪点直接建立锁定。
ACQUIRE_CONFIRM_FRAMES = 3
ACQUIRE_CONFIRM_DISTANCE = 6

# 已锁定时每5帧检查一次全局候选，但不会直接解除当前锁定。
GLOBAL_VERIFY_INTERVAL = 5
GLOBAL_SWITCH_CONFIRM_FRAMES = 2
GLOBAL_DENSITY_ADVANTAGE = 80


# ============================================================
# 7. 动态 ROI 参数
# ============================================================

# 正常跟踪区域为上一坐标周围50×50像素
TRACK_HALF_WIDTH = 30
TRACK_HALF_HEIGHT = 30

# 每丢失一帧，ROI向四周扩大多少像素
TRACK_EXPAND_PER_LOST = 6


# ============================================================
# 8. 坐标滤波参数
# ============================================================

# filtered = 1/4旧值 + 3/4新值
FILTER_OLD_WEIGHT = 1
FILTER_NEW_WEIGHT = 3
FILTER_TOTAL = 4


# ============================================================
# 9. 调试输出参数
# ============================================================

# 每5帧打印一次
PRINT_INTERVAL = 5

debug_frame_count = 0

# 黑胶带局部搜索诊断。只记录统计值，不参与候选判定。
core_diag_bright_x = 0
core_diag_bright_y = 0
core_diag_bright_r = 0
core_diag_bright_g = 0
core_diag_bright_b = 0
core_diag_bright_sum = -1
core_diag_red_x = 0
core_diag_red_y = 0
core_diag_red_r = 0
core_diag_red_g = 0
core_diag_red_b = 0
core_diag_red_score = -1000


# ============================================================
# 10. 摄像头初始化
# ============================================================

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

# 先自动适应环境
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)

sensor.skip_frames(time=2000)

auto_exposure_us = sensor.get_exposure_us()

# 锁定增益和白平衡
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# 先使用自动曝光值的2/3，避免真实小光点变得太暗
fixed_exposure_us = (
    auto_exposure_us * 2
) // 3

if fixed_exposure_us < 1000:
    fixed_exposure_us = 1000

sensor.set_auto_exposure(
    False,
    fixed_exposure_us
)

print(
    "auto exposure us:",
    auto_exposure_us
)

print(
    "fixed exposure us:",
    fixed_exposure_us
)

sensor.skip_frames(time=500)

clock = time.clock()
# ============================================================
# 11. 跟踪状态
# ============================================================

# 是否正式锁定激光
laser_valid = False

# 最终输出坐标
laser_x = IMAGE_WIDTH // 2
laser_y = IMAGE_HEIGHT // 2

# 滤波坐标
filtered_x = laser_x
filtered_y = laser_y

# 上一次正式接受的原始坐标
last_raw_x = IMAGE_WIDTH // 2
last_raw_y = IMAGE_HEIGHT // 2

# 连续丢失帧数
lost_count = 0

# 尚未正式锁定时的连续确认状态。
acquire_count = 0
acquire_x = IMAGE_WIDTH // 2
acquire_y = IMAGE_HEIGHT // 2

# 全局验证状态。只有更优候选连续出现才允许切换。
global_verify_count = 0
switch_confirm_count = 0
switch_x = IMAGE_WIDTH // 2
switch_y = IMAGE_HEIGHT // 2


# ============================================================
# 12. 基础函数
# ============================================================

def limit_value(value, min_value, max_value):
    if value < min_value:
        return min_value

    if value > max_value:
        return max_value

    return value


def distance_square(x1, y1, x2, y2):
    dx = x1 - x2
    dy = y1 - y2

    return dx * dx + dy * dy


def is_strong_blob(blob):
    """
    判断候选是否为强目标。

    pixels或area满足其中一个条件即可。
    """

    if blob.pixels() >= STRONG_MIN_PIXELS:
        return True

    if blob.area() >= STRONG_MIN_AREA:
        return True

    return False


def blob_density(blob):
    area = blob.area()

    if area <= 0:
        area = 1

    return (blob.pixels() * 100) // area


# ============================================================
# 13. 生成动态 ROI
# ============================================================

def get_tracking_roi(
    current_valid,
    center_x,
    center_y,
    current_lost_count
):
    # 完全失锁时搜索整个白板
    if not current_valid:
        return LASER_ROI

    expand = current_lost_count * TRACK_EXPAND_PER_LOST

    half_w = TRACK_HALF_WIDTH + expand
    half_h = TRACK_HALF_HEIGHT + expand

    roi_left = LASER_ROI[0]
    roi_top = LASER_ROI[1]
    roi_right = LASER_ROI[0] + LASER_ROI[2]
    roi_bottom = LASER_ROI[1] + LASER_ROI[3]

    # 防止动态ROI超过整个白板区域
    if half_w * 2 > LASER_ROI[2]:
        half_w = LASER_ROI[2] // 2

    if half_h * 2 > LASER_ROI[3]:
        half_h = LASER_ROI[3] // 2

    roi_x = center_x - half_w
    roi_y = center_y - half_h

    roi_x = limit_value(
        roi_x,
        roi_left,
        roi_right - 2 * half_w
    )

    roi_y = limit_value(
        roi_y,
        roi_top,
        roi_bottom - 2 * half_h
    )

    return (
        roi_x,
        roi_y,
        2 * half_w,
        2 * half_h
    )


# ============================================================
# 14. 计算当前最大移动距离
# ============================================================


class LocalCoreBlob:
    def __init__(self, x, y):
        self._x = x
        self._y = y

    def cx(self):
        return self._x

    def cy(self):
        return self._y

    def x(self):
        return self._x

    def y(self):
        return self._y

    def w(self):
        return 1

    def h(self):
        return 1

    def area(self):
        return 1

    def pixels(self):
        return 1

    def rect(self):
        return (self._x, self._y, 1, 1)


def find_black_tape_laser_core(
    img,
    center_x,
    center_y,
    max_move
):
    global core_diag_bright_x
    global core_diag_bright_y
    global core_diag_bright_r
    global core_diag_bright_g
    global core_diag_bright_b
    global core_diag_bright_sum
    global core_diag_red_x
    global core_diag_red_y
    global core_diag_red_r
    global core_diag_red_g
    global core_diag_red_b
    global core_diag_red_score

    search_half = 18

    core_diag_bright_sum = -1
    core_diag_red_score = -1000

    roi_left = LASER_ROI[0]
    roi_top = LASER_ROI[1]
    roi_right = LASER_ROI[0] + LASER_ROI[2] - 1
    roi_bottom = LASER_ROI[1] + LASER_ROI[3] - 1

    x0 = center_x - search_half
    y0 = center_y - search_half
    x1 = center_x + search_half
    y1 = center_y + search_half

    if x0 < roi_left + 1:
        x0 = roi_left + 1

    if y0 < roi_top + 1:
        y0 = roi_top + 1

    if x1 > roi_right - 1:
        x1 = roi_right - 1

    if y1 > roi_bottom - 1:
        y1 = roi_bottom - 1

    allowed_move = max_move

    if allowed_move > search_half:
        allowed_move = search_half

    allowed_move_sq = allowed_move * allowed_move

    best_x = 0
    best_y = 0
    best_score = -1000000
    found = False

    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            pixel = img.get_pixel(x, y)

            r = pixel[0]
            g = pixel[1]
            b = pixel[2]

            brightness = r + g + b

            dist_sq = distance_square(
                x,
                y,
                center_x,
                center_y
            )

            if dist_sq > allowed_move_sq:
                continue

            if brightness > core_diag_bright_sum:
                core_diag_bright_sum = brightness
                core_diag_bright_x = x
                core_diag_bright_y = y
                core_diag_bright_r = r
                core_diag_bright_g = g
                core_diag_bright_b = b

            red_score = 2 * r - g - b

            if red_score > core_diag_red_score:
                core_diag_red_score = red_score
                core_diag_red_x = x
                core_diag_red_y = y
                core_diag_red_r = r
                core_diag_red_g = g
                core_diag_red_b = b

            # 2026-07-18 黑胶带实测：同一激光核心会随帧变化为
            # (99,130,197)~(222,255,255)，亮度和约426~732。
            # 这里只放宽 target/上一激光点附近的局部判据；全局
            # 红色阈值保持不变，避免背景亮点误捕获。
            if brightness < 400:
                continue

            if r < 80 or g < 100 or b < 160:
                continue

            # 检查白色核心周围是否存在红黄色光晕。
            halo_score = -1000

            # 实测红色光晕可能距离核心2像素，因此检查5x5邻域。
            for offset_y in range(-2, 3):
                for offset_x in range(-2, 3):
                    if offset_x == 0 and offset_y == 0:
                        continue

                    near_pixel = img.get_pixel(
                        x + offset_x,
                        y + offset_y
                    )

                    nr = near_pixel[0]
                    ng = near_pixel[1]
                    nb = near_pixel[2]

                    current_halo = 2 * nr - ng - nb

                    if nr >= 60 and current_halo > halo_score:
                        halo_score = current_halo

            # 本轮黑胶带样本red_score约为36~244。
            if halo_score < 30:
                continue

            score = (
                brightness * 2
                + halo_score * 4
                - dist_sq * 8
            )

            if score > best_score:
                best_score = score
                best_x = x
                best_y = y
                found = True

    if not found:
        return None

    return LocalCoreBlob(best_x, best_y)


def get_max_move(current_lost_count):
    max_move = (
        MAX_MOVE_NORMAL
        + current_lost_count * MAX_MOVE_ADD_PER_LOST
    )

    if max_move > MAX_MOVE_RECOVER_MAX:
        max_move = MAX_MOVE_RECOVER_MAX

    return max_move


# ============================================================
# 15. 选择最佳候选
# ============================================================

def select_best_blob(
    blobs,
    current_valid,
    center_x,
    center_y,
    max_move
):
    best_blob = None
    best_score = -1000000

    max_move_sq = max_move * max_move
    weak_distance_sq = (
        WEAK_TARGET_MAX_DISTANCE
        * WEAK_TARGET_MAX_DISTANCE
    )

    for blob in blobs:

        # 排除尺寸过大的反光区域
        if blob.w() > MAX_WIDTH:
            continue

        if blob.h() > MAX_HEIGHT:
            continue

        if blob.area() > MAX_AREA:
            continue

        cx = blob.cx()
        cy = blob.cy()

        if current_valid:
            dist_sq = distance_square(
                cx,
                cy,
                center_x,
                center_y
            )

            # 超过当前允许移动范围，直接拒绝
            if dist_sq > max_move_sq:
                continue

            # 弱目标必须非常靠近上一坐标
            if not is_strong_blob(blob):
                if dist_sq > weak_distance_sq:
                    continue

        else:
            # 完全失锁时，要求候选具有最低可靠性
            if blob.pixels() < REACQUIRE_MIN_PIXELS:
                if blob.area() < REACQUIRE_MIN_AREA:
                    continue

            dist_sq = 0

        area = blob.area()

        if area <= 0:
            area = 1

        density_score = (
            blob.pixels() * 100
        ) // area

        if current_valid:
            # 已锁定：距离优先，使运动激光能够连续跟踪。
            score = (
                10000
                - dist_sq * 25
                + blob.pixels() * 8
                + density_score
            )
        else:
            # 未锁定：真实激光在当前装置中表现为“小而密”的红点。
            # 不使用绝对坐标，因此激光位于白板任意位置都适用。
            # 提高密度权重并惩罚过大面积，避免选择较大的红色反光。
            score = (
                density_score * 20
                + blob.pixels() * 4
                - area * 30
            )

        # 强目标略微增加评分
        if is_strong_blob(blob):
            score += 100

        if score > best_score:
            best_score = score
            best_blob = blob

    return best_blob


# ============================================================
# 16. 使用单个阈值寻找候选
# ============================================================

def find_blob_by_threshold(
    img,
    threshold,
    tracking_roi,
    current_valid,
    center_x,
    center_y,
    max_move
):
    blobs = img.find_blobs(
        [threshold],
        roi=tracking_roi,
        pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_AREA,
        merge=True,
        margin=1
    )

    return select_best_blob(
        blobs,
        current_valid,
        center_x,
        center_y,
        max_move
    )


# ============================================================
# 17. 寻找激光
# ============================================================

def find_laser(
    img,
    current_valid,
    center_x,
    center_y,
    current_lost_count,
    use_target_seed
):
    tracking_roi = get_tracking_roi(
        current_valid,
        center_x,
        center_y,
        current_lost_count
    )

    max_move = get_max_move(current_lost_count)

    # 已锁定时使用局部阈值
    if current_valid:
        red_threshold = LASER_RED_TRACK_THRESHOLD
        bright_threshold = LASER_BRIGHT_TRACK_THRESHOLD
        threshold_mode = 1

    # 完全失锁时使用严格阈值
    else:
        red_threshold = LASER_RED_THRESHOLD
        bright_threshold = LASER_BRIGHT_THRESHOLD
        threshold_mode = 0

    # 第一阶段：优先寻找红色光斑
    best_blob = find_blob_by_threshold(
        img,
        red_threshold,
        tracking_roi,
        current_valid,
        center_x,
        center_y,
        max_move
    )

    detect_type = 1

    # 第二阶段：红色未找到时使用高亮阈值兜底
    if best_blob is None:
        best_blob = find_blob_by_threshold(
            img,
            bright_threshold,
            tracking_roi,
            current_valid,
            center_x,
            center_y,
            max_move
        )

        detect_type = 2

    # 黑胶带上的激光可能只有“高亮核心 + 红色光晕”，无法通过
    # 全局红色/高亮阈值。已锁定时围绕上一激光点恢复；第一次
    # 锁定时则围绕 yuntai_main 传入的当前 target 搜索。
    if best_blob is None and (current_valid or use_target_seed):
        core_max_move = max_move

        # 首次锁定没有上一帧激光移动量可供约束，应允许使用完整的
        # 18 像素局部搜索半径。核心和光晕阈值保持不变。
        if use_target_seed and not current_valid:
            core_max_move = 18

        best_blob = find_black_tape_laser_core(
            img,
            center_x,
            center_y,
            core_max_move
        )

        if best_blob is not None:
            detect_type = 3

    if best_blob is None:
        return (

            False,
            0,
            0,
            None,
            tracking_roi,
            max_move,
            0,
            threshold_mode
        )

    return (
        True,
        best_blob.cx(),
        best_blob.cy(),
        best_blob,
        tracking_roi,
        max_move,
        detect_type,
        threshold_mode
    )


# ============================================================
# 18. 接受激光坐标
# ============================================================

def accept_laser(raw_x, raw_y):
    global laser_valid
    global laser_x
    global laser_y
    global filtered_x
    global filtered_y
    global last_raw_x
    global last_raw_y
    global lost_count

    if not laser_valid:
        # 重新捕获时直接使用当前坐标
        filtered_x = raw_x
        filtered_y = raw_y
    else:
        # 正常跟踪时进行轻度滤波
        filtered_x = (
            FILTER_OLD_WEIGHT * filtered_x
            + FILTER_NEW_WEIGHT * raw_x
        ) // FILTER_TOTAL

        filtered_y = (
            FILTER_OLD_WEIGHT * filtered_y
            + FILTER_NEW_WEIGHT * raw_y
        ) // FILTER_TOTAL

    last_raw_x = raw_x
    last_raw_y = raw_y

    laser_x = filtered_x
    laser_y = filtered_y

    laser_valid = True
    lost_count = 0


# ============================================================
# 19. 发送激光坐标到STM32
# ============================================================

def send_laser_point(x, y, valid):
    """
    固定7字节帧：AA 55 03 X_H X_L Y CHECKSUM。
    CHECKSUM为前6字节累加和的低8位。
    本帧无可靠坐标时发送Y=0xFF。
    """
    if valid:
        tx_frame[3] = (x >> 8) & 0xFF
        tx_frame[4] = x & 0xFF
        tx_frame[5] = y & 0xFF
    else:
        tx_frame[3] = 0
        tx_frame[4] = 0
        tx_frame[5] = 0xFF

    checksum = 0

    for i in range(6):
        checksum = (checksum + tx_frame[i]) & 0xFF

    tx_frame[6] = checksum
    uart.write(tx_frame)


# ============================================================
# 20. 主循环
# ============================================================

def run(
    dynamic_roi,
    locked_corners,
    send_callback,
    get_target_callback
):
    global LASER_ROI
    global debug_frame_count
    global global_verify_count
    global switch_confirm_count
    global switch_x
    global switch_y
    global laser_valid
    global lost_count
    global acquire_count
    global acquire_x
    global acquire_y

    LASER_ROI = dynamic_roi
    initial_target_x, initial_target_y = get_target_callback()
    gc.collect()
    print("LASER module start ROI:", LASER_ROI)
    print(
        "LASER first-lock seed:",
        initial_target_x,
        initial_target_y
    )
    print("LASER module free:", gc.mem_free())

    while True:
        clock.tick()
    
        img = sensor.snapshot()
    
        debug_frame_count += 1
        global_verify_count += 1
    
        search_center_x = last_raw_x
        search_center_y = last_raw_y
        use_target_seed = False

        if not laser_valid:
            search_center_x, search_center_y = get_target_callback()
            use_target_seed = True

        (
            candidate_found,
            raw_x,
            raw_y,
            laser_blob,
            tracking_roi,
            current_max_move,
            detect_type,
            threshold_mode
        ) = find_laser(
            img,
            laser_valid,
            search_center_x,
            search_center_y,
            lost_count,
            use_target_seed
        )
    
        # --------------------------------------------------------
        # 已锁定后的全局质量验证
        # --------------------------------------------------------
    
        verified_switch = False
    
        if (
            laser_valid
            and candidate_found
            and global_verify_count >= GLOBAL_VERIFY_INTERVAL
        ):
            global_verify_count = 0
    
            verify_blob = find_blob_by_threshold(
                img,
                LASER_RED_THRESHOLD,
                LASER_ROI,
                False,
                last_raw_x,
                last_raw_y,
                MAX_MOVE_RECOVER_MAX
            )
    
            if verify_blob is not None:
                current_density = blob_density(laser_blob)
                verify_density = blob_density(verify_blob)
    
                # 新候选必须明显更密，并且面积不能比当前错误反光更大。
                better_candidate = (
                    verify_density
                    >= current_density + GLOBAL_DENSITY_ADVANTAGE
                    and verify_blob.area() <= laser_blob.area()
                )
    
                if better_candidate:
                    verify_x = verify_blob.cx()
                    verify_y = verify_blob.cy()
    
                    if (
                        switch_confirm_count > 0
                        and distance_square(
                            verify_x,
                            verify_y,
                            switch_x,
                            switch_y
                        ) <= (
                            ACQUIRE_CONFIRM_DISTANCE
                            * ACQUIRE_CONFIRM_DISTANCE
                        )
                    ):
                        switch_confirm_count += 1
                    else:
                        switch_x = verify_x
                        switch_y = verify_y
                        switch_confirm_count = 1
    
                    if (
                        switch_confirm_count
                        >= GLOBAL_SWITCH_CONFIRM_FRAMES
                    ):
                        raw_x = verify_x
                        raw_y = verify_y
                        laser_blob = verify_blob
                        tracking_roi = LASER_ROI
                        detect_type = 1
                        threshold_mode = 0
                        verified_switch = True
                        switch_confirm_count = 0
    
                else:
                    switch_confirm_count = 0
    
            else:
                switch_confirm_count = 0
    
        # 本帧是否正式接受坐标
        accepted = False
    
        if candidate_found:
            if verified_switch:
                # 经连续全局验证确认了更小、更密的候选，立即清除旧滤波
                # 并切换，不受旧坐标的MAX_MOVE_NORMAL限制。
                laser_valid = False
                accept_laser(raw_x, raw_y)
                accepted = True
    
            elif laser_valid:
                # 已锁定后继续使用原有动态ROI和距离约束跟随运动。
                accept_laser(raw_x, raw_y)
                accepted = True
    
            else:
                # 未锁定时要求候选连续多帧出现在相近位置。
                if acquire_count == 0:
                    acquire_x = raw_x
                    acquire_y = raw_y
                    acquire_count = 1
    
                elif distance_square(
                    raw_x,
                    raw_y,
                    acquire_x,
                    acquire_y
                ) <= (
                    ACQUIRE_CONFIRM_DISTANCE
                    * ACQUIRE_CONFIRM_DISTANCE
                ):
                    acquire_x = raw_x
                    acquire_y = raw_y
                    acquire_count += 1
    
                else:
                    acquire_x = raw_x
                    acquire_y = raw_y
                    acquire_count = 1
    
                if acquire_count >= ACQUIRE_CONFIRM_FRAMES:
                    accept_laser(raw_x, raw_y)
                    acquire_count = 0
                    accepted = True
    
        else:
            acquire_count = 0
    
            if laser_valid:
                lost_count += 1
    
            # 连续丢失达到阈值后彻底解除锁定。下一帧将以
            # yuntai_main 的当前 target 为局部搜索种子重新捕获。
            if (
                laser_valid
                and lost_count >= LOST_MAX_COUNT
            ):
                laser_valid = False
                lost_count = 0
                acquire_count = 0
                switch_confirm_count = 0
                reseed_x, reseed_y = get_target_callback()
                print(
                    "LASER unlock seed=(%d,%d)"
                    % (reseed_x, reseed_y)
                )
    
        # 当前帧识别成功时发送最新坐标。
        # 短暂丢失1～3帧时保留上一可靠坐标；持续丢失后再报告无效。
        if accepted:
            send_callback(laser_x, laser_y, True)
    
        elif (
            laser_valid
            and lost_count > 0
            and lost_count <= SEND_HOLD_MAX_LOST
        ):
            send_callback(laser_x, laser_y, True)
    
        else:
            send_callback(0, 0, False)
    
        # --------------------------------------------------------
        # 绘制总 ROI
        # --------------------------------------------------------
    
        img.draw_rectangle(
            LASER_ROI,
            color=(255, 255, 255),
            thickness=1
        )
    
        # --------------------------------------------------------
        # 绘制动态 ROI
        # --------------------------------------------------------
    
        img.draw_rectangle(
            tracking_roi,
            color=(0, 0, 255),
            thickness=1
        )
    
        # --------------------------------------------------------
        # 绘制图像中心
        # --------------------------------------------------------
    
        img.draw_cross(
            IMAGE_WIDTH // 2,
            IMAGE_HEIGHT // 2,
            color=(0, 255, 0),
            size=8,
            thickness=1
        )

        # 尚未锁定时，用青色十字标出当前 target 搜索种子。
        if not laser_valid:
            img.draw_cross(
                search_center_x,
                search_center_y,
                color=(0, 255, 255),
                size=6,
                thickness=1
            )
    
        # --------------------------------------------------------
        # 绘制当前候选
        # --------------------------------------------------------
    
        if candidate_found:
    
            # 红色阈值成功：绿色框
            if detect_type == 1:
                box_color = (0, 255, 0)
    
            # 高亮阈值成功：黄色框
            else:
                box_color = (255, 255, 0)
    
            img.draw_rectangle(
                laser_blob.rect(),
                color=box_color,
                thickness=2
            )
    
            # 原始坐标使用红色小十字
            img.draw_cross(
                raw_x,
                raw_y,
                color=(255, 0, 0),
                size=4,
                thickness=1
            )
    
        # --------------------------------------------------------
        # 绘制最终输出坐标
        # --------------------------------------------------------
    
        if laser_valid:
    
            # 白色十字为最终输出坐标
            img.draw_cross(
                laser_x,
                laser_y,
                color=(255, 255, 255),
                size=8,
                thickness=2
            )
    
            img.draw_string(
                2,
                2,
                "OUT:%d,%d" % (
                    laser_x,
                    laser_y
                ),
                color=(255, 255, 255),
                scale=1
            )
    
            if accepted:
                img.draw_string(
                    2,
                    14,
                    "LOCK T:%d M:%d" % (
                        detect_type,
                        threshold_mode
                    ),
                    color=(0, 255, 0),
                    scale=1
                )
            else:
                img.draw_string(
                    2,
                    14,
                    "HOLD L:%d" % lost_count,
                    color=(255, 255, 0),
                    scale=1
                )
    
        else:
    
            img.draw_string(
                2,
                2,
                "LASER LOST",
                color=(255, 0, 0),
                scale=1
            )
    
            img.draw_string(
                2,
                14,
                "GLOBAL SEARCH",
                color=(255, 0, 255),
                scale=1
            )
    
        # --------------------------------------------------------
        # 串口调试输出
        # --------------------------------------------------------
    
        if debug_frame_count >= PRINT_INTERVAL:
            debug_frame_count = 0
    
            # 定期回收临时对象，并观察剩余堆内存是否稳定。
            gc.collect()
            print("memory free:", gc.mem_free())
    
            if accepted:
                print(
                    "laser valid=1 "
                    "raw=(%d,%d) "
                    "filtered=(%d,%d) "
                    "type=%d "
                    "mode=%d "
                    "pixels=%d "
                    "area=%d "
                    "w=%d "
                    "h=%d "
                    "strong=%d "
                    "max_move=%d"
                    % (
                        raw_x,
                        raw_y,
                        laser_x,
                        laser_y,
                        detect_type,
                        threshold_mode,
                        laser_blob.pixels(),
                        laser_blob.area(),
                        laser_blob.w(),
                        laser_blob.h(),
                        1 if is_strong_blob(laser_blob) else 0,
                        current_max_move
                    )
                )
    
            elif candidate_found:
                print(
                    "laser valid=%d "
                    "candidate=(%d,%d) "
                    "confirm=%d/%d "
                    "type=%d mode=%d"
                    % (
                        1 if laser_valid else 0,
                        raw_x,
                        raw_y,
                        acquire_count,
                        ACQUIRE_CONFIRM_FRAMES,
                        detect_type,
                        threshold_mode
                    )
                )

            else:
                print(
                    "laser valid=%d "
                    "candidate=none "
                    "output=(%d,%d) "
                    "lost=%d "
                    "max_move=%d"
                    % (
                        1 if laser_valid else 0,
                        laser_x,
                        laser_y,
                        lost_count,
                        current_max_move
                    )
                )

                if use_target_seed:
                    print(
                        "core diag seed=(%d,%d) "
                        "bright=(%d,%d,%d,%d,%d,sum=%d) "
                        "red=(%d,%d,%d,%d,%d,score=%d)"
                        % (
                            search_center_x,
                            search_center_y,
                            core_diag_bright_x,
                            core_diag_bright_y,
                            core_diag_bright_r,
                            core_diag_bright_g,
                            core_diag_bright_b,
                            core_diag_bright_sum,
                            core_diag_red_x,
                            core_diag_red_y,
                            core_diag_red_r,
                            core_diag_red_g,
                            core_diag_red_b,
                            core_diag_red_score
                        )
                    )
    
        # --------------------------------------------------------
        # 显示 FPS
        # --------------------------------------------------------
    
        img.draw_string(
            2,
            220,
            "FPS:%.1f" % clock.fps(),
            color=(255, 255, 255),
            scale=1
        )
