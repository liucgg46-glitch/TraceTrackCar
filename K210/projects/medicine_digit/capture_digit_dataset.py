# LCD_CAPTURE_PATCH_V1
# K210 / CanMV 数字数据采集程序
# 修改 SCENE 和 DIGIT 后运行；程序会把 JPEG 保存到 SD 卡。

import gc
import os
import sensor
import lcd
import time


# ==================== 每轮采集前修改 ====================
SCENE = "ground"
DIGIT = 2
CAPTURE_COUNT = 60
PREVIEW_SECONDS = 15
INTERVAL_MS = 5000
JPEG_QUALITY = 90
ROOT = "/sd/digit_capture"


def ensure_dir(path):
    current = ""
    for part in path.split("/"):
        if not part:
            continue
        current += "/" + part
        try:
            os.mkdir(current)
        except OSError:
            pass


def validate_config():
    if SCENE not in ("handheld", "ground", "background"):
        raise ValueError("SCENE must be handheld, ground or background")
    if SCENE != "background" and DIGIT not in range(1, 9):
        raise ValueError("DIGIT must be 1..8")


def camera_init():
    lcd.init()
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_auto_gain(True)
    sensor.set_auto_whitebal(True)
    sensor.set_auto_exposure(True)
    sensor.run(1)
    sensor.skip_frames(time=2000)


def preview_camera():
    print("LIVE PREVIEW START - position the digit card now")
    start = time.ticks_ms()
    last_remain = -1
    while time.ticks_diff(time.ticks_ms(), start) < PREVIEW_SECONDS * 1000:
        img = sensor.snapshot()
        elapsed = time.ticks_diff(time.ticks_ms(), start)
        remain = PREVIEW_SECONDS - (elapsed // 1000)
        if remain != last_remain:
            last_remain = remain
            print("CAPTURE STARTS IN", remain)
        img.draw_string(2, 2, "PREVIEW %ds" % remain,
                        color=(255, 0, 0), scale=2)
        lcd.display(img)
        del img
        gc.collect()
    print("LIVE PREVIEW COMPLETE - CAPTURE START")


def preview_until_next_save(interval_ms):
    """Keep the IDE framebuffer live while the card is repositioned."""
    start = time.ticks_ms()
    last_remain = -1
    while time.ticks_diff(time.ticks_ms(), start) < interval_ms:
        img = sensor.snapshot()
        elapsed = time.ticks_diff(time.ticks_ms(), start)
        remain_ms = interval_ms - elapsed
        remain = (remain_ms + 999) // 1000
        if remain != last_remain:
            last_remain = remain
            print("NEXT SAVE IN", remain)
        img.draw_string(2, 2, "NEXT %ds" % remain,
                        color=(255, 0, 0), scale=2)
        lcd.display(img)
        del img
        time.sleep_ms(20)


def output_dir():
    if SCENE == "background":
        return ROOT + "/background"
    return ROOT + "/" + SCENE + "/" + str(DIGIT)


def next_index(folder):
    highest = -1
    try:
        names = os.listdir(folder)
    except OSError:
        return 0
    for name in names:
        if not name.endswith(".jpg"):
            continue
        try:
            value = int(name.split("_")[-1].split(".")[0])
            if value > highest:
                highest = value
        except Exception:
            pass
    return highest + 1


def main():
    validate_config()
    folder = output_dir()
    ensure_dir(folder)
    camera_init()

    print("OUTPUT:", folder)
    print("Move the card continuously; avoid near-identical frames.")
    preview_camera()

    start = next_index(folder)
    for i in range(CAPTURE_COUNT):
        img = sensor.snapshot()
        index = start + i
        name = "%s/%s_%06d.jpg" % (folder, SCENE, index)
        img.save(name, quality=JPEG_QUALITY)
        print("SAVED", i + 1, "/", CAPTURE_COUNT, name)
        lcd.display(img)
        del img
        if (i & 15) == 15:
            gc.collect()
        if i + 1 < CAPTURE_COUNT:
            preview_until_next_save(INTERVAL_MS)

    print("CAPTURE COMPLETE:", folder)
    gc.collect()


if __name__ == "__main__":
    main()
