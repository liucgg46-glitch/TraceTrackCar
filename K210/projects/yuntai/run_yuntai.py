# -*- coding: utf-8 -*-

import sys
import gc

PROJECT_DIR = "/sd/projects/yuntai/runtime"

if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

# Stop an old UART instance and clear cached project modules when rerunning.
try:
    old_uart_module = sys.modules.get("uart_protocol")
    if old_uart_module is not None:
        old_uart_module.uart.deinit()
except Exception:
    pass

for module_name in (
    "yuntai_main",
    "laser_detect",
    "rect_detect",
    "uart_protocol"
):
    try:
        del sys.modules[module_name]
    except Exception:
        pass

gc.collect()
print("START YUNTAI PROJECT")
import yuntai_main

