import cv2
import urllib.request
import numpy as np
import serial
import time
import matplotlib.pyplot as plt
from ultralytics import YOLO

# ——— 参数配置 ———
ESP_CAM_URL1    = 'http://172.20.10.3/capture'
MODEL_PATH     = "runs/detect/train28/weights/best.pt"
SERIAL_PORT    = 'COM11'
BAUDRATE       = 57600

TARGET_LABELS  = ["tennis","table tennis","badminton"]
CONF_THRESH    = 0.65
ERR_TOL        = 10    # 像素
DIST_SAFE      = 20    # cm

# PID 参数
# x 轴（左右）
Kp_x, Ki_x, Kd_x = 1.6, 0.05, 0
# y 轴（前后）
Kp_y, Ki_y, Kd_y = 1, 0, 0

# 静摩擦补偿 & 输出限制
MIN_PWM   = 130
MAX_PWM   = 255
DEAD_BAND = 3     # 小于此误差视为不动作

# 全局 PID 状态
integral_x = 0.0
integral_y = 0.0
prev_err_x = 0.0
prev_err_y = 0.0
last_pid_t = time.time()

# 打开串口 & 加载模型
arduino = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
time.sleep(2)
model   = YOLO(MODEL_PATH)

# OpenCV 窗口
winName = 'PID Track'
cv2.namedWindow(winName, cv2.WINDOW_NORMAL)
cv2.resizeWindow(winName, 960, 720)

# 用于记录曲线
times, errs_x, errs_y = [], [], []
start_t = time.time()

def send_cmd(cmd: str):
    """发送指令到 Arduino，格式例如 'TL,150' 或 'F,130' 或 'S'"""
    arduino.write((cmd + "\n").encode())

def detect_ball(frame):
    """YOLO 检测，返回 (box,label,conf,center) 或 (None,None,None,None)"""
    results = model(frame, show=False)
    if not results or not results[0].boxes:
        return None, None, None, None
    for box in results[0].boxes:
        x1,y1,x2,y2 = map(float, box.xyxy[0])
        conf        = float(box.conf[0])
        cls         = int(box.cls[0])
        label       = model.names[cls]
        if label in TARGET_LABELS and conf >= CONF_THRESH:
            cx, cy = (x1 + x2)/2, (y1 + y2)/2
            return (int(x1),int(y1),int(x2),int(y2)), label, conf, (cx,cy)
    return None, None, None, None

# ——— 主循环 ———
while True:
    # 1) 取一帧
    try:
        resp  = urllib.request.urlopen(ESP_CAM_URL, timeout=1)
        data  = np.asarray(bytearray(resp.read()), dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
    except:
        continue

    # 2) 预处理
    frame = cv2.resize(frame, (640,480))
    frame = cv2.convertScaleAbs(frame, alpha=0.8, beta=30)
    h,w = frame.shape[:2]
    cx0, cy0 = w/2, h/2

    # 3) 检测 & 误差计算
    box, label, conf, center = detect_ball(frame)
    if box:
        x1,y1,x2,y2 = box
        cx, cy = center
        err_x = cx - cx0
        err_y = cy - cy0 - 120
    else:
        err_x = err_y = None

    # 4) PID 计算
    now = time.time()
    dt = now - last_pid_t
    last_pid_t = now

    if err_x is not None:
        integral_x += err_x * dt
        deriv_x    = (err_x - prev_err_x)/dt if dt>0 else 0.0
        raw_x      = Kp_x*err_x + Ki_x*integral_x + Kd_x*deriv_x
        prev_err_x = err_x

        integral_y += err_y * dt
        deriv_y    = (err_y - prev_err_y)/dt if dt>0 else 0.0
        raw_y      = Kp_y*err_y + Ki_y*integral_y + Kd_y*deriv_y
        prev_err_y = err_y
    else:
        raw_x = raw_y = 0.0

    # 5) 存曲线数据
    times.append(now - start_t)
    errs_x.append(err_x if err_x is not None else 0)
    errs_y.append(err_y if err_y is not None else 0)

    # 6) 静摩擦补偿 + 死区 + 饱和，并发送指令
    cmd = "S"
    if err_x is not None:
        # 假设避障逻辑略过，直接跟踪
        if abs(err_x) > ERR_TOL:
            val = max(-MAX_PWM, min(MAX_PWM, raw_x))
            if abs(val) > DEAD_BAND:
                pwm = int(min(MAX_PWM, MIN_PWM + abs(val) - DEAD_BAND))
                cmd = ("TL," if val>0 else "TR,") + str(pwm)
        elif abs(err_y) > ERR_TOL:
            val = max(-MAX_PWM, min(MAX_PWM, raw_y))
            if abs(val) > DEAD_BAND:
                pwm = int(min(MAX_PWM, MIN_PWM + abs(val) - DEAD_BAND))
                cmd = ("F," if val<0 else "B,") + str(pwm)
    send_cmd(cmd)

    # 7) 画面可视化
    if box:
        cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
        cv2.putText(frame, f"{label} {conf:.2f}", (x1,y1-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2)
        cv2.circle(frame, (int(cx),int(cy)), 5, (0,0,255), -1)
    cv2.circle(frame, (int(cx0),int(cy0)), 5, (255,0,0), -1)
    txt = f"errX={err_x:.0f}, errY={err_y:.0f}" if err_x is not None else "NoBall"
    cv2.putText(frame, txt, (10,25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255),2)
    cv2.putText(frame, f"Act:{cmd}", (10,55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0),2)
    cv2.imshow(winName, frame)
    if cv2.waitKey(1)&0xFF==27:
        break

# ——— 收尾：断开串口 & 画误差曲线、参数表格 ———
arduino.close()
cv2.destroyAllWindows()

fig, ax = plt.subplots(figsize=(10,5))
ax.plot(times, errs_x, label='errX')
ax.plot(times, errs_y, label='errY')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Error (px)')
ax.set_title('Tracking Error vs Time')
ax.legend()
ax.grid(True)

# 参数表格
table_data = [
    ["Kp_x", f"{Kp_x:.2f}"],
    ["Ki_x", f"{Ki_x:.2f}"],
    ["Kd_x", f"{Kd_x:.2f}"],
    ["Kp_y", f"{Kp_y:.2f}"],
    ["Ki_y", f"{Ki_y:.2f}"],
    ["Kd_y", f"{Kd_y:.2f}"],
]
ax.table(cellText=table_data,
         colLabels=["Param","Value"],
         cellLoc="center",
         colLoc="center",
         loc='upper right',
         bbox=[0.77,0.05,0.21,0.28])

plt.tight_layout()
plt.show()
