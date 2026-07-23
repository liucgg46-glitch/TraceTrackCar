# -*- coding: utf-8 -*-

import os
import sys
import gc

print("root:", os.listdir("/"))
print("sd:", os.listdir("/sd"))

if "/sd" not in sys.path:
    sys.path.append("/sd")

# Stop the previous UART object when this script is run again.
try:
    old_uart_module = sys.modules.get("uart_protocol")

    if old_uart_module is not None:
        old_uart_module.uart.deinit()
except Exception:
    pass

# Remove cached modules so import executes again.
module_names = (
    "yuntai_main",
    "laser_detect",
    "rect_detect",
    "uart_protocol"
)

for module_name in module_names:
    try:
        del sys.modules[module_name]
    except Exception:
        pass

gc.collect()

print("start yuntai_main")
import yuntai_main
