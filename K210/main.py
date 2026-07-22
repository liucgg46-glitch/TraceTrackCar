# -*- coding: utf-8 -*-

import gc
import time

# Wait for power, SD card and camera hardware to become stable.
time.sleep_ms(1500)
gc.collect()

print("AUTO START MEDICINE DIGIT")

# Start the stable variable-count digit detector.
exec(
    open(
        "/sd/projects/medicine_digit/model/"
        "digit_detect_stable_293508.py"
    ).read()
)
