# K210 / CanMV 采集照片安全清理程序
# 只删除 TARGET_DIR 当前层的图片，不递归，不删除目录。

import gc
import os


# ==================== 按需修改 ====================
SCENE = "handheld"  # "handheld"、"ground" 或 "background"
DIGIT = 1           # SCENE != "background" 时为 1～8

# 第一次保持 False 预览；确认路径和数量正确后改成 True 再运行。
CONFIRM_DELETE = False

ROOT = "/sd/digit_capture"
IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp")


def validate_config():
    if SCENE not in ("handheld", "ground", "background"):
        raise ValueError("SCENE must be handheld, ground or background")
    if SCENE != "background" and DIGIT not in range(1, 9):
        raise ValueError("DIGIT must be 1..8")


def target_dir():
    if SCENE == "background":
        return ROOT + "/background"
    return ROOT + "/" + SCENE + "/" + str(DIGIT)


def is_image(name):
    lower = name.lower()
    for suffix in IMAGE_SUFFIXES:
        if lower.endswith(suffix):
            return True
    return False


def main():
    validate_config()
    folder = target_dir()
    print("TARGET:", folder)

    try:
        names = os.listdir(folder)
    except OSError as exc:
        print("TARGET NOT FOUND:", exc)
        return

    images = []
    for name in names:
        if is_image(name):
            images.append(name)
    images.sort()

    print("IMAGES FOUND:", len(images))
    for name in images[:10]:
        print("  ", name)
    if len(images) > 10:
        print("  ...", len(images) - 10, "more")

    if not CONFIRM_DELETE:
        print("PREVIEW ONLY - NOTHING DELETED")
        print("Set CONFIRM_DELETE = True after checking TARGET and count.")
        return

    deleted = 0
    failed = 0
    for name in images:
        path = folder + "/" + name
        try:
            os.remove(path)
            deleted += 1
        except OSError as exc:
            failed += 1
            print("DELETE FAILED:", path, exc)

    gc.collect()
    print("DELETE COMPLETE: deleted=", deleted, "failed=", failed)
    print("Folder kept:", folder)


if __name__ == "__main__":
    main()
