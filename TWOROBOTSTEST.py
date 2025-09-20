###############################################################################
#           Dual ESP32-CAM  +  Two Arduino                                    #
# ───────────────────────────────────────────────────────────────────────── #
#  • v4.5-FG   : FAST-GRIP（停車即夾）                                       #
#  • v4.6-Lock : 多目標 → 鎖定最近(最大框)                                    #
#  • v4.7-HUD  : ★在畫面列出「所有目標名稱 + Confidence」並方框標註★          #
###############################################################################

import cv2, sys, time, queue, serial, urllib.request, numpy as np
import threading as th
import matplotlib.pyplot as plt
import serial.tools.list_ports as list_ports
from   collections import deque
from   ultralytics import YOLO

# ──────────────────────────── 使用者可調區 ────────────────────────────
URL_CAM1 = "http://172.20.10.3/capture"
URL_CAM2 = "http://172.20.10.5/capture"

PORT_CAM1 = "COM11"
PORT_CAM2 = "COM5"

INV_X_CAM1 , INV_Y_CAM1 = ( True,  True)   # CAM1: LR+UD 反
INV_X_CAM2 , INV_Y_CAM2 = ( True, False)   # CAM2: LR 反

OFFSET_CAM1 = (0, 122)      # +x 右 / +y 下
OFFSET_CAM2 = (30, 115)
# ---------- PID 參數 ----------
PID_CAM1 = dict(
    kp_x=0.35, ki_x=0.21, kd_x=0.002,
    kp_y=0.55, ki_y=0.18, kd_y=0.002,
    err_tol =2, err_dead=5, dead_band=2,
    pwm_min =99, pwm_max=220, pwm_scale=1,
    int_lim =70,
)
PID_CAM2 = dict(
    kp_x=0.32, ki_x=0.24, kd_x=0.002,
    kp_y=0.55, ki_y=0.19, kd_y=0.002,
    err_tol =2, err_dead=5, dead_band=2,
    pwm_min =99, pwm_max=220, pwm_scale=1,
    int_lim =70,
)

GRIP_SEC = 18.2            # 機械手臂抓取停車時間 (s)

# ─────────────────────────── YOLO / 常數 ───────────────────────────
MODEL_PATH = "runs/detect/train28/weights/best.pt"
TARGET_LBL = ["tennis", "table tennis", "badminton"]
CONF_TH    = 0.65
YOLO_MODEL = YOLO(MODEL_PATH).to("cuda:0")

IMG_W, IMG_H   = 640, 480
CAM_CX, CAM_CY = IMG_W//2, IMG_H//2
INFER_FPS      = 10
INFER_ITV      = 1/INFER_FPS
SEARCH_PWM     = 220     # 搜索時轉速

# FAST-GRIP
AVG_WIN   = 1
OK_FRAMES = 1
S2G_DELAY = 0.000

# 多目標鎖定
MISS_TOL  = 5            # 連續 miss 多少幀數後放棄鎖定

# UART
UART_TOUT = 0.6
SEND_GAP  = 0.04



# ─────────────────────────── 小工具 ───────────────────────────
def clamp(v, lo, hi):    return hi if v > hi else lo if v < lo else v
def ports():             return [p.device for p in list_ports.comports()]
def open_serial(port, baud=57600):
    try:
        s = serial.Serial(port, baud, timeout=0, write_timeout=0)
        print(f"[UART] {port} opened");  return s
    except serial.SerialException as e:
        print(f"[UART] 開啟 {port} 失敗：{e}\n可用埠：{ports()}"); return None

# ─────────────────────────── Robot 類 ───────────────────────────
class Robot:
    def __init__(self,name,url,port,pid,offset=(0,0),inv_x=False,inv_y=False):
        # ---- 基本 ----
        self.name=name; self.url=url
        self.offx,self.offy = offset
        self.inv_x,self.inv_y = inv_x,inv_y
        self.pid=pid

        # ---- UART ----
        self.ser=open_serial(port); self.sem=th.Event()
        if self.ser: th.Thread(target=self._rx,daemon=True).start()
        self.last_sent=("S",0.0)

        # ---- 佇列 & 執行緒 ----
        self.q_frame=queue.Queue(4); self.q_disp=queue.Queue(4)
        th.Thread(target=self._grab,daemon=True).start()
        th.Thread(target=self._loop,daemon=True).start()

        # ---- 狀態 ----
        self.state="SEARCH"; self.grip_t=0
        self.ok_cnt=0; self.last_inf=time.time()

        # PID 變數
        self.mv_ex=deque(maxlen=AVG_WIN); self.mv_ey=deque(maxlen=AVG_WIN)
        self.ix=self.iy=0.0; self.px=self.py=0.0; self.t_pid=time.time()

        # 鎖定
        self.lock_box=None; self.miss_cnt=0

        # Log
        self.t0=time.time(); self.lt=[]; self.lx=[]; self.ly=[]

    # ---------- UART RX ----------
    def _rx(self):
        buf=b""
        while True:
            try: data=self.ser.read(32) if self.ser else b""
            except serial.SerialException: return
            if data:
                buf+=data
                while b"\n" in buf:
                    line,buf=buf.split(b"\n",1)
                    if line.strip()==b"DONE": self.sem.set()
            time.sleep(0.005)

    def _send(self,cmd,wait=False):
        now=time.time()
        if cmd==self.last_sent[0] and now-self.last_sent[1]<SEND_GAP: return
        if not self.ser: return
        self.ser.write(cmd.encode()+b"\n"); self.last_sent=(cmd,now)
        if wait:
            self.sem.clear(); self.sem.wait(timeout=UART_TOUT)

    # ---------- Frame Grab ----------
    def _grab(self):
        while True:
            try:
                raw=urllib.request.urlopen(self.url,timeout=0.25).read()
                img=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
                if self.q_frame.full(): self.q_frame.get_nowait()
                self.q_frame.put(img)
            except: time.sleep(0.05)

    # ---------- 主迴圈 ----------
    def _loop(self):
        while True:
            frame=self.q_frame.get()
            img=self._prep(frame)

            # GRIP_WAIT
            if self.state=="GRIP_BUSY":
                if time.time()-self.grip_t>=GRIP_SEC:
                    self.state="SEARCH"; self.ok_cnt=0; self.ix=self.iy=0.0
                self._disp(img,"S")
                continue

            # FPS 篩選
            if time.time()-self.last_inf<INFER_ITV:
                self._disp(img,"S"); continue
            self.last_inf=time.time()

            best,all_dets=self._detect(img)

            # SEARCH
            if self.state=="SEARCH":
                if best is None:
                    self._send(f"TR,{SEARCH_PWM}")
                    self._disp(img,f"TR,{SEARCH_PWM}",None,all_dets)
                    continue
                self.state="TRACK"

            # TRACK
            if self.state=="TRACK":
                if best is None:
                    self.state="SEARCH"; self._disp(img,"S",None,all_dets); continue
                cmd=self._pid(best)
                if cmd is None:
                    self.grip_t=time.time(); self.state="GRIP_BUSY"
                    self._disp(img,"S",best,all_dets)
                else:
                    self._send(cmd); self._disp(img,cmd,best,all_dets)

    # ---------- 影像預處理 ----------
    def _prep(self,f):
        img=cv2.resize(f,(IMG_W,IMG_H))
        if self.inv_x: img=cv2.flip(img,1)
        if self.inv_y: img=cv2.flip(img,0)
        return cv2.convertScaleAbs(img,alpha=0.8,beta=30)#(alpha對比,beta亮度)

    # ---------- Detect + HUD ----------
    def _detect(self,img):
        """
        回傳：
          best_det  : (x1,y1,x2,y2,cx,cy) or None  → 用於鎖定 / PID
          all_dets  : [(lbl,conf,x1,y1,x2,y2,cx,cy), ...] → 用於 HUD
        """
        try: r=YOLO_MODEL(img,verbose=False)[0]
        except: return None, []

        cand=[]; all_out=[]
        for b in r.boxes:
            lbl=YOLO_MODEL.names[int(b.cls[0])]
            conf=float(b.conf[0])
            if lbl not in TARGET_LBL or conf<CONF_TH: continue
            x1,y1,x2,y2=map(int,b.xyxy[0])
            area=(x2-x1)*(y2-y1)
            cx,cy=(x1+x2)/2,(y1+y2)/2
            cand.append((area,x1,y1,x2,y2,cx,cy))
            all_out.append((lbl,conf,x1,y1,x2,y2,cx,cy))

        # --- 鎖定邏輯(避免看到多個目標物導致系統錯亂) ---
        if not cand:
            self.miss_cnt+=1
            if self.miss_cnt>=MISS_TOL: self.lock_box=None
            return None, all_out

        cand.sort(reverse=True)
        max_box=cand[0][1:]

        if self.lock_box is None:
            self.lock_box=max_box; self.miss_cnt=0; return max_box, all_out

        lx1,ly1,lx2,ly2,_,_=self.lock_box
        for _,x1,y1,x2,y2,cx,cy in cand:
            inter=max(0,min(lx2,x2)-max(lx1,x1))*max(0,min(ly2,y2)-max(ly1,y1))
            if inter:
                self.lock_box=(x1,y1,x2,y2,cx,cy); self.miss_cnt=0
                return self.lock_box, all_out

        self.miss_cnt+=1
        if self.miss_cnt>=MISS_TOL:
            self.lock_box=max_box; self.miss_cnt=0; return max_box, all_out
        return None, all_out

    # ---------- PID ----------
    def _pid(self,d):
        p=self.pid
        x1,y1,x2,y2,cx,cy=d
        ex=cx-CAM_CX-self.offx
        ey=cy-CAM_CY-self.offy
        if abs(ex)<p["err_dead"]: ex=0
        if abs(ey)<p["err_dead"]: ey=0

        self.mv_ex.append(ex); self.mv_ey.append(ey)
        ax=sum(self.mv_ex)/len(self.mv_ex); ay=sum(self.mv_ey)/len(self.mv_ey)
        self.lt.append(time.time()-self.t0); self.lx.append(ax); self.ly.append(ay)

        if abs(ax)<p["err_tol"] and abs(ay)<p["err_tol"]:
            self._send("S")
            if S2G_DELAY>0: time.sleep(S2G_DELAY)
            self._send("G",wait=False)
            return None

        dt=time.time()-self.t_pid; self.t_pid=time.time()

        if abs(ex)>p["err_dead"]:
            self.ix=clamp(self.ix+ex*dt,-p["int_lim"],p["int_lim"])
            rx=p["kp_x"]*ex + p["ki_x"]*self.ix + p["kd_x"]*((ex-self.px)/dt if dt else 0)
            self.px=ex
        else: rx=0; self.ix*=0.7

        if abs(ey)>p["err_dead"]:
            self.iy=clamp(self.iy+ey*dt,-p["int_lim"],p["int_lim"])
            ry=p["kp_y"]*ey + p["ki_y"]*self.iy + p["kd_y"]*((ey-self.py)/dt if dt else 0)
            self.py=ey
        else: ry=0; self.iy*=0.7

        if abs(rx)>=abs(ry) and abs(rx)>p["dead_band"]:
            v,move=rx,("TL" if rx>0 else "TR")
        elif abs(ry)>p["dead_band"]:
            v,move=ry,("F" if ry<0 else "B")
        else: return "S"

        pwm=int(clamp(p["pwm_min"]+p["pwm_scale"]*abs(v),
                      p["pwm_min"],p["pwm_max"]))
        return f"{move},{pwm}"

    # ---------- 顯示 ----------
    def _disp(self,img,cmd="S",best=None,all_dets=None):
        # 1. 先畫所有候選框 & 標籤
        if all_dets:
            for lbl,conf,x1,y1,x2,y2,cx,cy in all_dets:
                color=(0,180,255) if (best and (x1,y1,x2,y2,cx,cy)==best) else (0,255,0)
                cv2.rectangle(img,(x1,y1),(x2,y2),color,2)
                cv2.putText(img,f"{lbl} {conf:.2f}",(x1,y1-6),
                            cv2.FONT_HERSHEY_SIMPLEX,0.55,color,2)

            # 2. 左上角：列出目標物名稱 + 自信度conf
            hud_y=20
            for lbl,conf,_,_,_,_,_,_ in all_dets[:5]:   # 顯示前 5 筆避免太多會很亂
                cv2.putText(img,f"{lbl}: {conf:.2f}",(5,hud_y),
                            cv2.FONT_HERSHEY_PLAIN,1.2,(255,255,255),2)
                hud_y+=18

        # 3. 準心 & 系統當前狀態
        cv2.circle(img,(CAM_CX,CAM_CY),4,(255,0,0),-1)
        cv2.putText(img,f"{self.name} {self.state}",(10,IMG_H-25),
                    cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),2)
        cv2.putText(img,cmd,(10,IMG_H-55),
                    cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),2)

        if self.q_disp.full(): self.q_disp.get_nowait()
        self.q_disp.put(img)

# ─────────────────────────── 建立兩台車 ───────────────────────────
bot1=Robot("CAM1",URL_CAM1,PORT_CAM1,PID_CAM1,
           OFFSET_CAM1,INV_X_CAM1,INV_Y_CAM1)
bot2=Robot("CAM2",URL_CAM2,PORT_CAM2,PID_CAM2,
           OFFSET_CAM2,INV_X_CAM2,INV_Y_CAM2)

# ─────────────────────────── GUI Loop ───────────────────────────
cv2.namedWindow("CAM1",cv2.WINDOW_NORMAL); cv2.resizeWindow("CAM1",640,480)
cv2.namedWindow("CAM2",cv2.WINDOW_NORMAL); cv2.resizeWindow("CAM2",640,480)
try:
    while True:
        if not bot1.q_disp.empty(): cv2.imshow("CAM1",bot1.q_disp.get())
        if not bot2.q_disp.empty(): cv2.imshow("CAM2",bot2.q_disp.get())
        if cv2.waitKey(1)&0xFF==27: break
except KeyboardInterrupt: pass

for b in (bot1,bot2):
    if b.ser: b.ser.close()
cv2.destroyAllWindows()

# ─────────────────────────── Plot Error ───────────────────────────
fig,(ax1,ax2)=plt.subplots(2,1,figsize=(11,8),sharex=True)
if bot1.lt:
    ax1.plot(bot1.lt,bot1.lx,label="CAM1 errX")
    ax1.plot(bot1.lt,bot1.ly,label="CAM1 errY")
    ax1.set_ylabel("Error(px)"); ax1.set_title("CAM1 Tracking Error")
    ax1.legend(); ax1.grid(True)
if bot2.lt:
    ax2.plot(bot2.lt,bot2.lx,label="CAM2 errX")
    ax2.plot(bot2.lt,bot2.ly,label="CAM2 errY")
    ax2.set_xlabel("Time(s)"); ax2.set_ylabel("Error(px)")
    ax2.set_title("CAM2 Tracking Error")
    ax2.legend(); ax2.grid(True)
plt.tight_layout(); plt.show()
