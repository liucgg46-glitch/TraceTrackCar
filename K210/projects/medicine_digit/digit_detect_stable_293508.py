# -*- coding: utf-8 -*-
"""Stable variable-count digit detection for model-293508 on K210.

Pipeline:
camera -> single letterbox or dual windows -> global coordinates -> filters
-> position-based cross-window merge -> temporal tracking/voting
-> left-to-right stable output -> USB terminal and STM32 UART.
"""

import gc
import image
import sensor
import lcd
import time
from maix import KPU
from machine import UART
from fpioa_manager import fm


MODEL_PATH = "/sd/projects/medicine_digit/model/model-293508.kmodel"

# Exact parameters exported with model-293508. Do not reuse old model values.
LABELS = ("2", "6", "1", "5", "4", "7", "3", "8")
ANCHORS = (1.19, 1.47, 2.00, 2.69, 1.34,
           1.81, 0.94, 0.94, 1.03, 1.09)

# Start with one full-frame, aspect-ratio-preserving letterbox inference.
# Change to "dual" only if distant/small digits are missed too often.
INFERENCE_MODE = "single"              # "single" or "dual"

CONF_THRESHOLD = 0.58
NMS_THRESHOLD = 0.30
MIN_BOX_AREA = 120
MAX_BOX_AREA = 30000
MIN_ASPECT_RATIO = 0.20
MAX_ASPECT_RATIO = 2.00

# Full-frame ROI. Narrow this only after the real camera mounting is fixed.
ROI_LEFT = 0
ROI_TOP = 0
ROI_RIGHT = 320
ROI_BOTTOM = 240

# Single-inference letterbox: 320x240 -> 224x168, black padding top/bottom.
NET_SIZE = 224
LETTERBOX_HEIGHT = 168
LETTERBOX_PAD_Y = 28

# Dual 224x224 windows. Right-window global X = local X + 96.
DUAL_WINDOWS = ((0, 8, 0), (96, 8, 1))
WINDOW_SIZE = 224
WINDOW_SPLIT_X = 160

# A pair is the same physical object if any position test passes.
DUPLICATE_IOU = 0.30
DUPLICATE_CONTAINMENT = 0.65
DUPLICATE_CENTER_SCALE = 0.45
DUPLICATE_CENTER_MIN_PX = 12

# Temporal stability: keep five recent observations, confirm after two
# consecutive hits, and delete only after three consecutive misses.
TRACK_HISTORY = 5
TRACK_CONFIRM_HITS = 2
TRACK_DELETE_MISSES = 3
TRACK_DISTANCE_SCALE = 0.70
TRACK_DISTANCE_MIN_PX = 28

VFLIP = False
HORIZONTAL_MIRROR = False

# Same STM32 UART wiring used by the existing yuntai project.
ENABLE_STM32_UART = True
UART_TX_PIN = 6
UART_BAUDRATE = 115200
OUTPUT_INTERVAL_MS = 200

# STM32 currently uses a fixed seven-byte frame:
# AA 55 CMD DATA1 DATA2 DATA3 CHECKSUM.
# One stable digit snapshot is sent as BEGIN + ITEM(s) + END so that a
# variable number of detections does not require changing the UART driver.
UART_FRAME_HEAD1 = 0xAA
UART_FRAME_HEAD2 = 0x55
UART_CMD_SNAPSHOT_BEGIN = 0x10
UART_CMD_SNAPSHOT_ITEM = 0x11
UART_CMD_SNAPSHOT_END = 0x12

UART_RESULT_EMPTY = 0
UART_RESULT_NORMAL = 1
UART_RESULT_AMBIGUOUS = 2
UART_RESULT_OVERFLOW = 3

# This is only the communication buffer capacity. The detector itself still
# has no fixed output count and no result is silently truncated.
UART_MAX_RESULTS = 8


def init_camera():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_vflip(VFLIP)
    sensor.set_hmirror(HORIZONTAL_MIRROR)
    sensor.run(1)
    sensor.skip_frames(time=1500)


def init_uart():
    if not ENABLE_STM32_UART:
        return None
    try:
        fm.register(UART_TX_PIN, fm.fpioa.UART1_TX, force=True)
        uart = UART(UART.UART1, UART_BAUDRATE, 8, 0, 1,
                    timeout=100, read_buf_len=64)
        print("STM32 UART READY pin=%d baud=%d" %
              (UART_TX_PIN, UART_BAUDRATE))
        return uart
    except Exception as error:
        print("STM32 UART DISABLED:", error)
        return None


def intersection_area(a, b):
    ax2 = a[0] + a[2]
    ay2 = a[1] + a[3]
    bx2 = b[0] + b[2]
    by2 = b[1] + b[3]
    ix1 = max(a[0], b[0])
    iy1 = max(a[1], b[1])
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = ix2 - ix1
    ih = iy2 - iy1
    if iw <= 0 or ih <= 0:
        return 0
    return iw * ih


def box_iou(a, b):
    inter = intersection_area(a, b)
    if inter <= 0:
        return 0.0
    union = a[2] * a[3] + b[2] * b[3] - inter
    if union <= 0:
        return 0.0
    return inter / union


def box_containment(a, b):
    inter = intersection_area(a, b)
    smaller = min(a[2] * a[3], b[2] * b[3])
    if smaller <= 0:
        return 0.0
    return inter / smaller


def center_xy(box):
    return box[0] + box[2] // 2, box[1] + box[3] // 2


def same_position(a, b):
    # Label is deliberately ignored: two windows can disagree on class.
    if box_iou(a, b) >= DUPLICATE_IOU:
        return True
    if box_containment(a, b) >= DUPLICATE_CONTAINMENT:
        return True

    acx, acy = center_xy(a)
    bcx, bcy = center_xy(b)
    dx = acx - bcx
    dy = acy - bcy
    min_diagonal = min((a[2] * a[2] + a[3] * a[3]) ** 0.5,
                       (b[2] * b[2] + b[3] * b[3]) ** 0.5)
    limit = max(DUPLICATE_CENTER_MIN_PX,
                int(min_diagonal * DUPLICATE_CENTER_SCALE))
    return dx * dx + dy * dy <= limit * limit


def valid_box(candidate):
    x, y, w, h = candidate[0], candidate[1], candidate[2], candidate[3]
    if w <= 0 or h <= 0:
        return False
    area = w * h
    if area < MIN_BOX_AREA or area > MAX_BOX_AREA:
        return False
    ratio = w / h
    if ratio < MIN_ASPECT_RATIO or ratio > MAX_ASPECT_RATIO:
        return False
    cx, cy = center_xy(candidate)
    if cx < ROI_LEFT or cx >= ROI_RIGHT:
        return False
    if cy < ROI_TOP or cy >= ROI_BOTTOM:
        return False
    return True


def clip_global_box(x, y, w, h):
    if x < 0:
        w += x
        x = 0
    if y < 0:
        h += y
        y = 0
    if x + w > 320:
        w = 320 - x
    if y + h > 240:
        h = 240 - y
    return x, y, w, h


def run_detector(detector, ai_image):
    ai_image.pix_to_ai()
    detector.run_with_output(ai_image)
    return detector.regionlayer_yolo2()


def detect_single_letterbox(detector, frame):
    """Return detections mapped from padded 224x224 to global 320x240."""
    resized = frame.resize(NET_SIZE, LETTERBOX_HEIGHT)
    padded = image.Image(size=(NET_SIZE, NET_SIZE))
    padded.draw_image(resized, 0, LETTERBOX_PAD_Y)
    output = run_detector(detector, padded)
    candidates = []

    if output:
        content_top = LETTERBOX_PAD_Y
        content_bottom = LETTERBOX_PAD_Y + LETTERBOX_HEIGHT
        for det in output:
            lx1 = int(det[0])
            ly1 = int(det[1])
            lx2 = lx1 + int(det[2])
            ly2 = ly1 + int(det[3])
            class_id = int(det[4])
            confidence = float(det[5])

            if class_id < 0 or class_id >= len(LABELS):
                continue

            # Remove black padding from the predicted box before mapping.
            lx1 = max(0, min(NET_SIZE, lx1))
            lx2 = max(0, min(NET_SIZE, lx2))
            ly1 = max(content_top, min(content_bottom, ly1))
            ly2 = max(content_top, min(content_bottom, ly2))
            if lx2 <= lx1 or ly2 <= ly1:
                continue

            x1 = int(lx1 * 320 / NET_SIZE)
            x2 = int(lx2 * 320 / NET_SIZE)
            y1 = int((ly1 - content_top) * 240 / LETTERBOX_HEIGHT)
            y2 = int((ly2 - content_top) * 240 / LETTERBOX_HEIGHT)
            candidate = [x1, y1, x2 - x1, y2 - y1,
                         LABELS[class_id], confidence, True]
            if valid_box(candidate):
                candidates.append(candidate)

    del resized
    del padded
    return candidates


def detect_dual_window(detector, frame, offset_x, offset_y, window_id):
    """Map local window results into the original 320x240 coordinates."""
    crop = frame.cut(offset_x, offset_y, WINDOW_SIZE, WINDOW_SIZE)
    output = run_detector(detector, crop)
    candidates = []

    if output:
        for det in output:
            # Required global conversion: left +0, right +96.
            x = int(det[0]) + offset_x
            y = int(det[1]) + offset_y
            w = int(det[2])
            h = int(det[3])
            class_id = int(det[4])
            confidence = float(det[5])
            if class_id < 0 or class_id >= len(LABELS):
                continue

            x, y, w, h = clip_global_box(x, y, w, h)
            cx = x + w // 2
            preferred_owner = ((window_id == 0 and cx < WINDOW_SPLIT_X) or
                               (window_id == 1 and cx >= WINDOW_SPLIT_X))
            candidate = [x, y, w, h,
                         LABELS[class_id], confidence, preferred_owner]
            if valid_box(candidate):
                candidates.append(candidate)

    del crop
    return candidates


def position_clusters(candidates):
    """Cluster by location, then keep the highest-confidence class per object."""
    preferred = []
    secondary = []
    for item in candidates:
        if item[6]:
            preferred.append(item)
        else:
            secondary.append(item)
    preferred.sort(key=lambda item: item[5], reverse=True)
    secondary.sort(key=lambda item: item[5], reverse=True)

    clusters = []
    for candidate in preferred + secondary:
        target = None
        for cluster in clusters:
            for member in cluster:
                if same_position(candidate, member):
                    target = cluster
                    break
            if target is not None:
                break
        if target is None:
            clusters.append([candidate])
        else:
            target.append(candidate)

    winners = []
    for cluster in clusters:
        cluster.sort(key=lambda item: item[5], reverse=True)
        winners.append(cluster[0])
    return winners


def global_deduplicate(candidates):
    """Second global location-only de-duplication pass."""
    candidates.sort(key=lambda item: item[5], reverse=True)
    kept = []
    for candidate in candidates:
        duplicate_index = -1
        for index, existing in enumerate(kept):
            if same_position(candidate, existing):
                duplicate_index = index
                break
        if duplicate_index < 0:
            kept.append(candidate)
        elif candidate[5] > kept[duplicate_index][5]:
            kept[duplicate_index] = candidate
    return kept


def collect_raw_detections(detector, frame, mode):
    if mode == "single":
        return global_deduplicate(
            detect_single_letterbox(detector, frame)
        )

    candidates = []
    for offset_x, offset_y, window_id in DUAL_WINDOWS:
        candidates.extend(
            detect_dual_window(detector, frame,
                               offset_x, offset_y, window_id)
        )
    return global_deduplicate(position_clusters(candidates))


def new_track(detection):
    return {
        "box": detection[0:4],
        "history": [[detection[4], detection[5]]],
        "consecutive": 1,
        "missed": 0,
        "confirmed": False
    }


def track_match_distance(track, detection):
    tcx, tcy = center_xy(track["box"])
    dcx, dcy = center_xy(detection)
    dx = tcx - dcx
    dy = tcy - dcy
    size = max(track["box"][2], track["box"][3],
               detection[2], detection[3])
    limit = max(TRACK_DISTANCE_MIN_PX,
                int(size * TRACK_DISTANCE_SCALE))
    distance_sq = dx * dx + dy * dy
    if distance_sq <= limit * limit:
        return distance_sq
    return None


def update_track(track, detection):
    old = track["box"]
    # Light box smoothing reduces coordinate jitter sent to STM32.
    track["box"] = [
        (old[0] * 2 + detection[0]) // 3,
        (old[1] * 2 + detection[1]) // 3,
        (old[2] * 2 + detection[2]) // 3,
        (old[3] * 2 + detection[3]) // 3
    ]
    track["history"].append([detection[4], detection[5]])
    if len(track["history"]) > TRACK_HISTORY:
        track["history"].pop(0)
    track["consecutive"] += 1
    track["missed"] = 0
    if track["consecutive"] >= TRACK_CONFIRM_HITS:
        track["confirmed"] = True


def update_tracks(tracks, detections):
    existing_count = len(tracks)
    matched_tracks = []

    # High-confidence observations claim tracks first; class is not used here.
    detections.sort(key=lambda item: item[5], reverse=True)
    for detection in detections:
        best_index = -1
        best_distance = None
        for index in range(existing_count):
            if index in matched_tracks:
                continue
            distance = track_match_distance(tracks[index], detection)
            if distance is not None:
                if best_distance is None or distance < best_distance:
                    best_index = index
                    best_distance = distance

        if best_index >= 0:
            update_track(tracks[best_index], detection)
            matched_tracks.append(best_index)
        else:
            tracks.append(new_track(detection))

    for index in range(existing_count):
        if index not in matched_tracks:
            tracks[index]["missed"] += 1
            tracks[index]["consecutive"] = 0

    # Keep a confirmed track through one or two missing frames; delete on third.
    alive = []
    for track in tracks:
        if track["missed"] < TRACK_DELETE_MISSES:
            alive.append(track)
    return alive


def voted_label_and_confidence(track):
    counts = {}
    sums = {}
    for label, confidence in track["history"]:
        counts[label] = counts.get(label, 0) + 1
        sums[label] = sums.get(label, 0.0) + confidence

    best_label = None
    best_count = -1
    best_sum = -1.0
    for label in counts:
        count = counts[label]
        total = sums[label]
        if count > best_count or (count == best_count and total > best_sum):
            best_label = label
            best_count = count
            best_sum = total
    return best_label, best_sum / best_count


def stable_results(tracks):
    results = []
    for track in tracks:
        if not track["confirmed"]:
            continue
        label, confidence = voted_label_and_confidence(track)
        box = track["box"]
        results.append([box[0], box[1], box[2], box[3],
                        label, confidence])
    results.sort(key=lambda item: center_xy(item)[0])
    return results


def make_uart_frame(command, data1, data2, data3):
    frame = bytearray(7)
    frame[0] = UART_FRAME_HEAD1
    frame[1] = UART_FRAME_HEAD2
    frame[2] = command & 0xFF
    frame[3] = data1 & 0xFF
    frame[4] = data2 & 0xFF
    frame[5] = data3 & 0xFF
    checksum = 0
    for value in frame[0:6]:
        checksum = (checksum + value) & 0xFF
    frame[6] = checksum
    return frame


def encode_center_x(center_x):
    # Map 0..319 to one byte. The maximum reconstruction error is below 1 px.
    if center_x < 0:
        center_x = 0
    elif center_x > 319:
        center_x = 319
    return (center_x * 255 + 159) // 319


def make_uart_snapshot(results, sequence):
    """Build one atomic BEGIN + ITEM(s) + END binary snapshot."""
    frames = bytearray()
    count = len(results)

    if count > UART_MAX_RESULTS:
        # Never truncate into a plausible but wrong result. Report overflow and
        # let STM32 wait for the next 200 ms snapshot.
        status = UART_RESULT_OVERFLOW
        wire_count = 0
    elif count == 0:
        status = UART_RESULT_EMPTY
        wire_count = 0
    else:
        status = UART_RESULT_NORMAL
        wire_count = count

    frames.extend(make_uart_frame(
        UART_CMD_SNAPSHOT_BEGIN, sequence, wire_count, status
    ))

    if status == UART_RESULT_NORMAL:
        for index, item in enumerate(results):
            center_x, _ = center_xy(item)
            digit = int(item[4])
            confidence = int(item[5] * 100 + 0.5)
            if confidence < 0:
                confidence = 0
            elif confidence > 100:
                confidence = 100
            index_digit = ((index & 0x0F) << 4) | (digit & 0x0F)
            frames.extend(make_uart_frame(
                UART_CMD_SNAPSHOT_ITEM,
                index_digit,
                confidence,
                encode_center_x(center_x)
            ))

    frames.extend(make_uart_frame(
        UART_CMD_SNAPSHOT_END, sequence, wire_count, 0
    ))
    return frames, status


def print_results(results, sequence, status, packet_bytes):
    text = ""
    for item in results:
        text += item[4]
    if not text:
        text = "none"
    print("STABLE RESULT:", text, "COUNT:", len(results))
    for index, item in enumerate(results):
        cx, cy = center_xy(item)
        print("[%d] digit=%s confidence=%.2f centerX=%d centerY=%d width=%d height=%d" %
              (index + 1, item[4], item[5], cx, cy, item[2], item[3]))
    print("UART SNAPSHOT: seq=%d status=%d bytes=%d" %
          (sequence, status, packet_bytes))


def main():
    lcd.init()
    init_camera()
    uart = init_uart()
    detector = KPU()
    model_loaded = False
    active_mode = INFERENCE_MODE

    try:
        print("LOADING:", MODEL_PATH)
        detector.load_kmodel(MODEL_PATH)
        model_loaded = True
        detector.init_yolo2(
            ANCHORS,
            anchor_num=5,
            img_w=224,
            img_h=224,
            net_w=224,
            net_h=224,
            layer_w=7,
            layer_h=7,
            threshold=CONF_THRESHOLD,
            nms_value=NMS_THRESHOLD,
            classes=8
        )

        print("MODEL 293508 READY")
        print("LABELS:", LABELS)
        print("MODE:", active_mode, "THRESHOLD:", CONF_THRESHOLD)
        clock = time.clock()
        tracks = []
        last_output_ms = time.ticks_ms() - OUTPUT_INTERVAL_MS
        single_mode_checked = False
        uart_sequence = 0

        while True:
            clock.tick()
            frame = sensor.snapshot()

            try:
                raw = collect_raw_detections(detector, frame, active_mode)
                if active_mode == "single":
                    single_mode_checked = True
            except Exception as single_error:
                if active_mode == "single" and not single_mode_checked:
                    print("SINGLE LETTERBOX UNSUPPORTED:", single_error)
                    print("AUTOMATIC FALLBACK TO DUAL WINDOWS")
                    active_mode = "dual"
                    raw = collect_raw_detections(detector, frame, active_mode)
                else:
                    raise

            tracks = update_tracks(tracks, raw)
            stable = stable_results(tracks)

            for item in stable:
                x, y, w, h, label, confidence = item
                cx, cy = center_xy(item)
                frame.draw_rectangle(x, y, w, h, color=(0, 255, 0))
                frame.draw_cross(cx, cy, color=(255, 255, 0), size=4)
                frame.draw_string(x, max(0, y - 12),
                                  "%s %.0f%%" % (label, confidence * 100),
                                  color=(255, 0, 0), scale=1)

            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_output_ms) >= OUTPUT_INTERVAL_MS:
                packet, packet_status = make_uart_snapshot(
                    stable, uart_sequence
                )
                print_results(stable, uart_sequence,
                              packet_status, len(packet))
                if uart is not None:
                    uart.write(packet)
                uart_sequence = (uart_sequence + 1) & 0xFF
                last_output_ms = now_ms

            result_text = ""
            for item in stable:
                result_text += item[4]
            if not result_text:
                result_text = "none"

            frame.draw_string(2, 2, "DIGITS:" + result_text,
                              color=(255, 255, 0), scale=1)
            frame.draw_string(2, 15, "N:%d %s T:%.2f" %
                              (len(stable), active_mode, CONF_THRESHOLD),
                              color=(0, 255, 255), scale=1)
            frame.draw_string(2, 226, "FPS:%.1f" % clock.fps(),
                              color=(255, 255, 255), scale=1)
            lcd.display(frame)

            del raw
            del stable
            gc.collect()

    except Exception as error:
        print("DIGIT DETECTOR ERROR:", error)
        raise
    finally:
        if model_loaded:
            detector.deinit()
        if uart is not None:
            try:
                uart.deinit()
            except Exception:
                pass
        gc.collect()


main()
