# -*- coding: utf-8 -*-
"""Shared launcher for additional K210 digit dataset capture batches."""

import gc
import sys


PROJECT_DIR = "/sd/projects/medicine_digit"


def run(scene, digit, count):
    if PROJECT_DIR not in sys.path:
        sys.path.insert(0, PROJECT_DIR)

    gc.collect()
    import capture_digit_dataset as capture

    capture.SCENE = scene
    capture.DIGIT = digit
    capture.CAPTURE_COUNT = count
    capture.PREVIEW_SECONDS = 15
    capture.INTERVAL_MS = 2000
    capture.JPEG_QUALITY = 90

    print("REINFORCE CAPTURE scene=", scene,
          "digit=", digit, "count=", count,
          "interval_ms=", capture.INTERVAL_MS)
    capture.main()
    gc.collect()
