# -*- coding: utf-8 -*-
"""Explicit launcher for the medicine digit capture project."""

import gc
import sys


PROJECT_DIR = "/sd/projects/medicine_digit"

if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

gc.collect()
print("START MEDICINE DIGIT CAPTURE")
import capture_digit_dataset

capture_digit_dataset.main()
gc.collect()
