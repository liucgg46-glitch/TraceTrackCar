# main.py
# K210 主程序：UART1 通信 + 巡线 + 二维码识别
# 说明：
# 1) TEST_ONLY = True 时，只测试 K210 <-> STM32 串口连通性
# 2) TEST_ONLY = False 时，进入视觉识别并发送协议帧

from fpioa_manager import fm
from machine import UART
import sensor
import image
import time

import os
print(os.listdir())

from k210_protocol import send_line_offset, send_qr_center, send_text

# ==================== 配置区 ====================
UART_BAUD = 115200

# True：只测串口连通；False：进入视觉协议模式
TEST_ONLY = False

# 摄像头参数
IMG_W = 320
IMG_H = 240

# 黑线巡线阈值（灰度图，L通道）
# 需要按你场地实际情况微调
LINE_THRESHOLD = [(0, 60)]
LINE_AREA_THRESHOLD = 100

# 连接测试发送周期
PING_INTERVAL_MS = 1000

# 主循环节拍
LOOP_SLEEP_MS = 50

# ==================== UART1 初始化（外扩接口） ====================
def uart1_init():
    # Yahboom CanMV-K210 外扩接口固定使用 IO8(TX), IO6(RX)
    fm.register(8, fm.fpioa.UART1_TX, force=True)
    fm.register(6, fm.fpioa.UART1_RX, force=True)

    uart1 = UART(UART.UART1, baudrate=UART_BAUD, bits=8, parity=None, stop=1, timeout=1000)
    return uart1

# ==================== 摄像头初始化 ====================
def camera_init():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.run(1)
    sensor.skip_frames(time=2000)

# ==================== 测试 STM32 串口连通 ====================
def link_test_loop(uart1):
    last_ping = time.ticks_ms()

    while True:
        # 读 STM32 回包
        rx = uart1.read()
        if rx:
            print("STM32 RX:", rx)

        now = time.ticks_ms()
        if time.ticks_diff(now, last_ping) >= PING_INTERVAL_MS:
            last_ping = now
            send_text(uart1, b"K210_LINK_TEST\r\n")
            print("K210 TX: K210_LINK_TEST")

        time.sleep_ms(LOOP_SLEEP_MS)

# ==================== 视觉模式 ====================
# ==================== 视觉模式（改用 find_blobs，彻底解决全屏误检） ====================
def vision_loop(uart1):
    clock = time.clock()
    # 黑线灰度阈值（需要根据实际环境调整，例如线是黑色的，背景较亮）
    # 建议先用阈值编辑器获取准确数值，这里给出一个较窄的初始值
    BLACK_THRESHOLD = (0,20)   # 灰度值0~40视为黑色，可调整

    while True:
        clock.tick()
        img = sensor.snapshot()

        # 检查 STM32 回包
        rx = uart1.read()
        if rx:
            print("STM32 RX:", rx)

        # 1) 二维码识别优先
        qrs = img.find_qrcodes()
        if qrs:
            qr = qrs[0]
            img.draw_rectangle(qr.rect())
            corners = qr.corners()
            cx = (corners[0][0] + corners[2][0]) // 2
            cy = (corners[0][1] + corners[2][1]) // 2
            payload = qr.payload()
            send_qr_center(uart1, cx, cy)
            print("QR:", payload, "cx=", cx, "cy=", cy)
            continue   # 已处理二维码，跳过巡线

        # 2) 巡线识别：寻找黑色块
        # area_threshold 过滤面积太小的区域，pixels_threshold 同理
        blobs = img.find_blobs([BLACK_THRESHOLD], pixels_threshold=100, area_threshold=100, merge=True)
        if blobs:
            # 取面积最大的黑色块（假设是巡线轨迹）
            largest = max(blobs, key=lambda b: b.area())
            line_center = largest.cx()          # 黑色块的中心x坐标
            offset = line_center - (IMG_W // 2) # 偏移量：正右负左
            print("Blob area:", largest.area(), "cx:", line_center, "offset:", offset)
            send_line_offset(uart1, offset)
        else:
            # 没有检测到黑线，可选择不发送，让STM32超时处理
            print("No line detected")

        time.sleep_ms(LOOP_SLEEP_MS)

#def vision_loop(uart1):
    #clock = time.clock()

    #while True:
        #clock.tick()
        #img = sensor.snapshot()

        ## 先看 STM32 有没有回数据
        #rx = uart1.read()
        #if rx:
            #print("STM32 RX:", rx)

        ## 1) 二维码识别优先
        #qrs = img.find_qrcodes()
        #if qrs:
            #qr = qrs[0]
            #corners = qr.corners()
            #cx = (corners[0][0] + corners[2][0]) // 2
            #cy = (corners[0][1] + corners[2][1]) // 2
            #payload = qr.payload()

            #send_qr_center(uart1, cx, cy)
            #print("QR:", payload, "cx=", cx, "cy=", cy)

        #else:
            ## 2) 巡线识别
            #line = img.get_regression(LINE_THRESHOLD, area_threshold=LINE_AREA_THRESHOLD)
        #if line:
            #x1 = line.x1()
            #x2 = line.x2()
            #x_mid = (x1 + x2) // 2
            #offset = x_mid - (IMG_W // 2)

            ## 打印详细信息，帮助调试
            #print("line: x1=%d, x2=%d, x_mid=%d, offset=%d" % (x1, x2, x_mid, offset))


            ##line = img.get_regression(LINE_THRESHOLD, area_threshold=LINE_AREA_THRESHOLD)
            ##if line:
                ##x_mid = (line.x1() + line.x2()) // 2
                ##offset = x_mid - (IMG_W // 2)
                ##send_line_offset(uart1, offset)
                ##print("LINE offset:", offset)


        #time.sleep_ms(LOOP_SLEEP_MS)

# ==================== 主函数 ====================
def main():
    uart1 = uart1_init()
    print("MY_MAIN_START")
    print("UART1 INIT OK (IO8 TX, IO6 RX)")

    if TEST_ONLY:
        # 连通测试模式
        print("Entering link test mode...")
        link_test_loop(uart1)
    else:
        # 视觉识别模式
        print("Entering vision mode...")
        camera_init()
        vision_loop(uart1)

if __name__ == "__main__":
    main()
