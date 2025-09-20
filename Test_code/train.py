import os
from ultralytics import YOLO

def main():
    # 使用正確的路徑格式設定環境變數
    os.environ['YOLO_DATASET_DIR'] = "C:\\Users\\陳俊言\\AppData\\Roaming\\Code\\User\\settings.json"
    
    # 加載模型
    model = YOLO('models\\yolov8n.pt')
    
    # 開始訓練
    model.train(
        data='C:\\Users\\陳俊言\\OneDrive\\桌面\\esp32-cam-server\\roboflow9500\\data.yaml',
        epochs=300,
        imgsz=640,
        batch=16,
    )

if __name__ == '__main__':
    main()
