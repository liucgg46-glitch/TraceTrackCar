# K210 / CanMV 数字数据集统一管理器
# 从 CanMV IDE 设置 ACTION、SCENE、DIGIT、CAPTURE_COUNT 后 exec 本文件。

import gc
import os
import sys
import time


ROOT = "/sd/digit_capture"
PROJECT_DIR = "/sd/projects/medicine_digit"
IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp")

if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)


def get_setting(name, default):
    values = globals()
    if name in values:
        return values[name]
    return default


def validate(action, scene, digit, count, preview_seconds, interval_ms):
    if action not in ("status", "capture", "clear"):
        raise ValueError("ACTION must be status, capture or clear")
    if scene not in ("handheld", "ground", "background"):
        raise ValueError("SCENE must be handheld, ground or background")
    if scene != "background" and digit not in range(1, 9):
        raise ValueError("DIGIT must be 1..8")
    if count < 1 or count > 1000:
        raise ValueError("CAPTURE_COUNT must be 1..1000")
    if preview_seconds < 1 or preview_seconds > 120:
        raise ValueError("PREVIEW_SECONDS must be 1..120")
    if interval_ms < 100 or interval_ms > 30000:
        raise ValueError("INTERVAL_MS must be 100..30000")


def target_dir(scene, digit):
    if scene == "background":
        return ROOT + "/background"
    return ROOT + "/" + scene + "/" + str(digit)


def image_names(folder):
    try:
        names = os.listdir(folder)
    except OSError:
        return []
    result = []
    for name in names:
        lower = name.lower()
        for suffix in IMAGE_SUFFIXES:
            if lower.endswith(suffix):
                result.append(name)
                break
    result.sort()
    return result


def show_status(scene, digit):
    folder = target_dir(scene, digit)
    names = image_names(folder)
    print("STATUS target=", folder, "images=", len(names))


def clear_batch(scene, digit):
    folder = target_dir(scene, digit)
    names = image_names(folder)
    print("CLEAR TARGET:", folder)
    print("IMAGES TO DELETE:", len(names))
    if not names:
        print("NOTHING TO DELETE")
        return

    # ACTION="clear" 已是明确删除指令；再留 3 秒供 IDE 停止运行。
    for remain in range(3, 0, -1):
        print("DELETE IN", remain, "- stop program now to cancel")
        time.sleep(1)

    deleted = 0
    failed = 0
    for name in names:
        path = folder + "/" + name
        try:
            os.remove(path)
            deleted += 1
        except OSError as exc:
            failed += 1
            print("DELETE FAILED:", path, exc)
    gc.collect()
    print("CLEAR COMPLETE deleted=", deleted, "failed=", failed)


def capture_batch(scene, digit, count, preview_seconds, interval_ms):
    import capture_digit_dataset as capture
    capture.SCENE = scene
    capture.DIGIT = digit
    capture.CAPTURE_COUNT = count
    capture.PREVIEW_SECONDS = preview_seconds
    capture.INTERVAL_MS = interval_ms
    capture.main()


def main():
    action = str(get_setting("ACTION", "status")).lower()
    scene = str(get_setting("SCENE", "handheld")).lower()
    digit = int(get_setting("DIGIT", 1))
    count = int(get_setting("CAPTURE_COUNT", 20))
    preview_seconds = int(get_setting("PREVIEW_SECONDS", 15))
    interval_ms = int(get_setting("INTERVAL_MS", 5000))
    validate(action, scene, digit, count, preview_seconds, interval_ms)

    print("DIGIT MANAGER action=", action, "scene=", scene,
          "digit=", digit, "count=", count,
          "preview=", preview_seconds, "interval_ms=", interval_ms)
    if action == "status":
        show_status(scene, digit)
    elif action == "clear":
        clear_batch(scene, digit)
    else:
        capture_batch(scene, digit, count, preview_seconds, interval_ms)


main()
