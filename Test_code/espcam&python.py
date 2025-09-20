import cv2
from ultralytics import YOLO

# 加載 YOLOv8 模型
model = YOLO("C:\Users\陳俊言\OneDrive\桌面\esp32-cam-server\datasets\tennis\result\weights\best.pt")  # 替換為訓練好的 YOLO 模型路徑

# ESP32-CAM 的 HTTP 串流 URL
stream_url = "http://172.20.10.3:81/stream"  # 確保此 URL 指向 ESP32-CAM 的影像串流

# 開啟影像串流
cap = cv2.VideoCapture(stream_url)

# 檢查串流是否成功打開
if not cap.isOpened():
    print("無法連接到 ESP32-CAM 的影像串流")
    exit()

while True:
    # 從串流中讀取一幀影像
    ret, frame = cap.read()
    if not ret:
        print("無法從影像串流中獲取幀，嘗試重新連接...")
        cap.release()
        cap = cv2.VideoCapture(stream_url)
        continue

    # 使用 YOLOv8 模型進行物件檢測
    results = model(frame)

    # 繪製檢測結果
    annotated_frame = results[0].plot()

    # 顯示結果
    cv2.imshow("ESP32-CAM Detection", annotated_frame)

    # 按 'q' 鍵退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 釋放資源
cap.release()
cv2.destroyAllWindows()
