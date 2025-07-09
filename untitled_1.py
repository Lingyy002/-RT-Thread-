from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8
from libs.Utils import *
from machine import UART, FPIOA
import _thread
import os, sys, gc, time
import ulab.numpy as np
import image

# 冷却记录与阈值
last_sent_time = {
    "person": 0,
    "fire": 0,
    "open": 0
}
cooldown_ms = 2000  # 每类消息2秒冷却

# UART配置
uart = None
fpioa = FPIOA()

def init_uart():
    global uart
    try:
        fpioa.set_function(11, fpioa.UART2_TXD)
        fpioa.set_function(12, fpioa.UART2_RXD)
    except:
        pass
    uart = UART(UART.UART2, baudrate=9600, bits=8, parity=0, stop=1)
    print("[系统] UART 初始化完成")
    while uart.any():
        uart.read()

def uart_send_hex(data_bytes):
    uart.write(data_bytes)
    print("[UART发送] hex:", data_bytes)

# 可选：UART接收线程
def uart_recv_thread():
    global uart
    while True:
        if uart.any():
            data = uart.read()
            if data:
                print("接收到:", data.hex(), ''.join([chr(b) if 32 <= b <= 126 else '.' for b in data]))
        time.sleep(0.1)

# 主程序
if __name__ == "__main__":
    init_uart()
    _thread.start_new_thread(uart_recv_thread, ())

    # 配置参数
    kmodel_path = "/data/best.kmodel"
    labels = ["person", "fire", "open"]
    model_input_size = [320, 320]
    display_mode = "lcd"
    rgb888p_size = [640, 360]
    confidence_threshold = 0.5
    nms_threshold = 0.45

    # 初始化PipeLine和YOLO
    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
    pl.create()
    display_size = pl.get_display_size()

    yolo = YOLOv8(
        task_type="detect",
        mode="video",
        kmodel_path=kmodel_path,
        labels=labels,
        rgb888p_size=rgb888p_size,
        model_input_size=model_input_size,
        display_size=display_size,
        conf_thresh=confidence_threshold,
        nms_thresh=nms_threshold,
        max_boxes_num=50,
        debug_mode=0
    )
    yolo.config_preprocess()

    # 主推理循环
    while True:
        with ScopedTiming("total", 1):
            img = pl.get_frame()
            res = yolo.run(img)
            print("res:", res)

            # 绘制结果
            yolo.draw_result(res, pl.osd_img)
            pl.show_image()

            # 统计检测结果
            person_count = 0
            fire_detected = False
            open_detected = False

            for i in range(len(res)):
                x1, y1, x2, y2, score, class_id = res[i]
                class_id = int(class_id)
                label = labels[class_id]
                print("检测到:", label, "置信度:", score)

                if label == "person":
                    person_count += 1
                elif label == "fire" and score > 0.7:
                    fire_detected = True
                elif label == "open" and score > 0.7:
                    open_detected = True

            # 执行冷却+发送逻辑
            now = time.ticks_ms()

            if person_count > 5:
                print("person_count:", person_count)
                if time.ticks_diff(now, last_sent_time["person"]) > cooldown_ms:
                    uart_send_hex(b'\x00\x01\x01\xaa')
                    last_sent_time["person"] = now

            if fire_detected:
                print("fire_detected")
                if time.ticks_diff(now, last_sent_time["fire"]) > cooldown_ms:
                    uart_send_hex(b'\x00\x01\x01\xbb')
                    last_sent_time["fire"] = now

            if open_detected:
                print("open_detected")
                if time.ticks_diff(now, last_sent_time["open"]) > cooldown_ms:
                    uart_send_hex(b'\x00\x01\x01\xcc')
                    last_sent_time["open"] = now

            gc.collect()

    yolo.deinit()
    pl.destroy()
